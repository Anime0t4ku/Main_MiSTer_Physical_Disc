#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <errno.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/cdrom.h>
#include <scsi/sg.h>
#include <limits.h>

#include "physical_disc.h"
#include "physical_disc_acoustic.h"

#define RING_SECTORS        4096
#define LANE_COUNT          2
#define LANE_SECTORS        (RING_SECTORS / LANE_COUNT)
#define ENTRY_SIZE          (PHYSICAL_DISC_RAW + PHYSICAL_DISC_SUB)
#define LOOKAHEAD_SPAN      96
#define AUDIO_LOOKAHEAD     224
#define IO_BURST            16
#define AUDIO_SYNC_BURST    4
#define WARMUP_SECTORS      768
#define STARTUP_PRIME_SECTORS 32
#define INDEX00_ENTRY_PROBES 4
#define MAX_PREGAP_SECTORS   750
#define NEIGHBOR_PREWARM_SECTORS 128
#define NEIGHBOR_ENTRY_SPAN 32
#define STATS_PERIOD_MS     5000
#define SWAP_POLL_MS        500
#define SYNC_IO_BURST       8
#define SYNC_IO_TIMEOUT_MS  3000
#define AUDIO_SYNC_WAIT_MS  900
#define BG_IO_TIMEOUT_MS    3000
#define CAPPED_SPEED_NX     4
#define IDLE_KEEPALIVE_MS   15000

typedef struct {
	int lba;
	int sub_present;
	int unreadable;
	uint8_t data[ENTRY_SIZE];
} cache_entry_t;

typedef struct {
	int lo;
	int hi;
	int is_audio;
} track_span_t;

typedef struct {
	volatile int dev_fd;
	int leadout_lba;
	int data_lba0;
	int subch_ok;
	track_span_t span[100];
	int track_count;
	cache_entry_t *ring;
	volatile int lane_cursor[LANE_COUNT];
	volatile int lane_active[LANE_COUNT];
	volatile int alive;
	volatile int sync_busy;
	pthread_t io_thread;
	pthread_mutex_t ring_lock;
	pthread_mutex_t io_lock;
	uint32_t hit_count, miss_count;
	uint32_t bad_count;
	uint32_t bad_logged;
	double worst_wait_ms;
	double worst_io_ms;
	volatile int fail_streak;
	uint32_t reattach_count;
	volatile int watching;
	volatile int event_code;
	volatile int event_disc_type;
	volatile int event_region;
	volatile int event_initial;
	int seen_present;
	int seen_type;
	char seen_label[64];
	volatile int seen_dirty;
	volatile int swap_armed;
	volatile int swap_ready;
	volatile int mid_swap;
	volatile int active_lane;
	volatile int warmup_lba;
	volatile int warmup_end;
	volatile int swap_out;
	volatile int want_native_speed;
	volatile int rush_lane;
	volatile int rush_lba;
	volatile int neighbor_lba;
	volatile int neighbor_end;
} drive_state_t;

static drive_state_t drv = { -1, 0, -1, -1, {}, 0, NULL, {0,0}, {0,0}, 0, 0, 0,
	  PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER,
	  0, 0, 0, 0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, {0}, 0, 0, 0, 0, -1, -1, 0, 0, 0, -1, -1, -1, 0 };

#define SWAP_MARKER_PATH "/tmp/physical_disc_swapped"

static int span_index(int lba)
{
	for (int i = 0; i < drv.track_count; i++)
		if (lba < drv.span[i].hi) return i;
	return drv.track_count ? drv.track_count - 1 : -1;
}

static char preferred_dev[64] = "";
static char active_dev[64] = "";

static inline int lane_of(int lba)
{
	int t = span_index(lba);
	return (t >= 0 && drv.span[t].is_audio) ? 1 : 0;
}

static inline cache_entry_t *entry_for(int lba)
{
	return &drv.ring[lane_of(lba) * LANE_SECTORS + (lba % LANE_SECTORS)];
}

static double clock_ms()
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

static void apply_speed_cap()
{
	if (drv.dev_fd < 0) return;

	int audio = 0;
	for (int i = 0; i < drv.track_count; i++)
		if (drv.span[i].is_audio) audio = 1;

	int speed = drv.want_native_speed && drv.track_count > 0 && !audio ? 0 : CAPPED_SPEED_NX;
	if (ioctl(drv.dev_fd, CDROM_SELECT_SPEED, speed) < 0)
		printf("DISC: speed selection not supported, drive keeps its default\n");
	else if (speed)
		printf("DISC: speed capped at %dx (~%d KB/s, need 172)\n", speed, speed * 177);
	else
		printf("DISC: native drive speed enabled\n");
}

void physical_disc_native_speed(int enable)
{
	drv.want_native_speed = enable ? 1 : 0;
	if (drv.dev_fd >= 0 && drv.track_count > 0) apply_speed_cap();
}

static void silence_block_probes(const char *dev)
{
	const char *name = strrchr(dev, '/');
	name = name ? name + 1 : dev;

	char path[128];
	FILE *f;

	snprintf(path, sizeof(path), "/sys/block/%s/queue/read_ahead_kb", name);
	if ((f = fopen(path, "w")))
	{
		fputs("0", f);
		fclose(f);
	}

	snprintf(path, sizeof(path), "/sys/block/%s/events_poll_msecs", name);
	if ((f = fopen(path, "w")))
	{
		fputs("-1", f);
		fclose(f);
	}
}

static void install_udev_rule(void)
{
	static const char *path = "/etc/udev/rules.d/59-physical-disc-cdrom.rules";
	static const char *rule =
		"ACTION!=\"remove\", KERNEL==\"sr[0-9]*\", ENV{UDEV_DISABLE_PERSISTENT_STORAGE_RULES_FLAG}=\"1\"\n";

	size_t rule_len = strlen(rule);
	char current[256] = {};
	FILE *f = fopen(path, "r");
	if (f)
	{
		size_t read_len = fread(current, 1, sizeof(current) - 1, f);
		fclose(f);
		if (read_len == rule_len && !memcmp(current, rule, rule_len)) return;
	}

	f = fopen(path, "w");
	if (!f) return;
	fputs(rule, f);
	fflush(f);
	fsync(fileno(f));
	fclose(f);

	system("export PATH=/usr/sbin:/sbin:/usr/bin:/bin:$PATH; udevadm control --reload-rules 2>/dev/null || udevadm control --reload 2>/dev/null");
}

void physical_disc_prepare_environment(void)
{
	install_udev_rule();
}

static int scsi_read_cd(int lba, int count, uint8_t flags, int with_sub, uint8_t *dst, int timeout_ms)
{
	uint8_t cdb[12] = { 0 };
	uint8_t sense[32];
	struct sg_io_hdr io;
	int sector_len = PHYSICAL_DISC_RAW + (with_sub ? PHYSICAL_DISC_SUB : 0);

	cdb[0] = 0xBE;
	if (flags == 0x10) cdb[1] = 0x04;
	cdb[2] = (lba >> 24) & 0xFF;
	cdb[3] = (lba >> 16) & 0xFF;
	cdb[4] = (lba >> 8) & 0xFF;
	cdb[5] = lba & 0xFF;
	cdb[6] = (count >> 16) & 0xFF;
	cdb[7] = (count >> 8) & 0xFF;
	cdb[8] = count & 0xFF;
	cdb[9] = flags;
	cdb[10] = with_sub ? 0x01 : 0x00;

	memset(&io, 0, sizeof(io));
	io.interface_id = 'S';
	io.cmd_len = 12;
	io.cmdp = cdb;
	io.dxfer_direction = SG_DXFER_FROM_DEV;
	io.dxfer_len = count * sector_len;
	io.dxferp = dst;
	io.sbp = sense;
	io.mx_sb_len = sizeof(sense);
	io.timeout = timeout_ms;

	if (ioctl(drv.dev_fd, SG_IO, &io) < 0) return -1;
	if (io.status || io.host_status || io.driver_status) return -2;
	return 0;
}

static int ioctl_read_raw(int lba, uint8_t *dst)
{
	union {
		struct cdrom_msf msf;
		uint8_t raw[PHYSICAL_DISC_RAW];
	} req;

	int f = lba + 150;
	memset(&req, 0, sizeof(req));
	req.msf.cdmsf_min0 = f / (75 * 60);
	req.msf.cdmsf_sec0 = (f / 75) % 60;
	req.msf.cdmsf_frame0 = f % 75;

	if (ioctl(drv.dev_fd, CDROMREADRAW, &req) < 0) return -1;
	memcpy(dst, req.raw, PHYSICAL_DISC_RAW);
	return 0;
}

static int refill_ring(int lba, int count, int sync)
{
	uint8_t burst[IO_BURST * ENTRY_SIZE];

	if (count > IO_BURST) count = IO_BURST;
	if (lba < 0) lba = 0;
	if (lba + count > drv.leadout_lba) count = drv.leadout_lba - lba;
	if (count <= 0) return 0;

	int t = span_index(lba);
	if (t >= 0 && lba + count > drv.span[t].hi) count = drv.span[t].hi - lba;
	int audio = (t >= 0 && drv.span[t].is_audio);
	if (sync && audio && count > AUDIO_SYNC_BURST) count = AUDIO_SYNC_BURST;
	uint8_t flags = audio ? 0x10 : 0xF8;
	int timeout_ms = sync && audio ? AUDIO_SYNC_WAIT_MS : (sync ? SYNC_IO_TIMEOUT_MS : BG_IO_TIMEOUT_MS);

	int with_sub = (drv.subch_ok == 1);

	double io0 = clock_ms();
	pthread_mutex_lock(&drv.io_lock);
	int r = scsi_read_cd(lba, count, flags, with_sub, burst, timeout_ms);
	pthread_mutex_unlock(&drv.io_lock);

	if (r && with_sub && count > 1) {
		pthread_mutex_lock(&drv.io_lock);
		int r2 = scsi_read_cd(lba, count, flags, 0, burst, timeout_ms);
		pthread_mutex_unlock(&drv.io_lock);
		if (!r2) {
			printf("DISC: drive rejects multi-sector subchannel reads, disabling subchannel\n");
			drv.subch_ok = 0;
			with_sub = 0;
			r = 0;
		}
	}

	if (r && sync) {
		double dt = clock_ms() - io0;
		if (dt > drv.worst_io_ms) drv.worst_io_ms = dt;
		return -1;
	}

	if (r) {
		for (int i = 0; i < count; i++) {
			uint8_t one[ENTRY_SIZE];
			int rr = -1;

			for (int n = 0; n < 3 && rr; n++) {
				pthread_mutex_lock(&drv.io_lock);
				rr = scsi_read_cd(lba + i, 1, flags, 0, one, BG_IO_TIMEOUT_MS);
				pthread_mutex_unlock(&drv.io_lock);
			}
			if (rr) {
				pthread_mutex_lock(&drv.io_lock);
				rr = ioctl_read_raw(lba + i, one);
				pthread_mutex_unlock(&drv.io_lock);
			}

			if (rr) drv.fail_streak++; else drv.fail_streak = 0;
			pthread_mutex_lock(&drv.ring_lock);
			cache_entry_t *e = entry_for(lba + i);
			if (!rr) {
				memcpy(e->data, one, PHYSICAL_DISC_RAW);
				e->unreadable = 0;
			} else {
				memset(e->data, 0, PHYSICAL_DISC_RAW);
				e->unreadable = 1;
				drv.bad_count++;
				if (drv.bad_logged < 8) {
					drv.bad_logged++;
					printf("DISC: unreadable sector lba=%d%s\n", lba + i,
						drv.bad_logged == 8 ? " (further ones counted silently)" : "");
				}
			}
			memset(e->data + PHYSICAL_DISC_RAW, 0, PHYSICAL_DISC_SUB);
			e->sub_present = 0;
			e->lba = lba + i;
			pthread_mutex_unlock(&drv.ring_lock);
		}
		double d = clock_ms() - io0;
		if (d > drv.worst_io_ms) drv.worst_io_ms = d;
		return 0;
	}

	double d = clock_ms() - io0;
	if (d > drv.worst_io_ms) drv.worst_io_ms = d;

	drv.fail_streak = 0;

	int sector_len = PHYSICAL_DISC_RAW + (with_sub ? PHYSICAL_DISC_SUB : 0);
	pthread_mutex_lock(&drv.ring_lock);
	for (int i = 0; i < count; i++) {
		cache_entry_t *e = entry_for(lba + i);
		memcpy(e->data, burst + i * sector_len, sector_len);
		if (!with_sub) memset(e->data + PHYSICAL_DISC_RAW, 0, PHYSICAL_DISC_SUB);
		e->sub_present = with_sub;
		e->lba = lba + i;
	}
	pthread_mutex_unlock(&drv.ring_lock);
	return 0;
}

static void write_stats_log()
{
	FILE *f = fopen("/tmp/physical_disc_stats.log", "w");
	if (f) {
		fprintf(f, "hit %u miss %u  hitrate %.1f%%  worst miss %.0f ms\n",
			drv.hit_count, drv.miss_count,
			(drv.hit_count + drv.miss_count) ? 100.0 * drv.hit_count / (drv.hit_count + drv.miss_count) : 0.0,
			drv.worst_wait_ms);
		fprintf(f, "BAD %u sectors served as zeros  worst drive io %.0f ms\n",
			drv.bad_count, drv.worst_io_ms);
		fprintf(f, "REATTACH %u  (device %s)\n", drv.reattach_count, active_dev);
		fprintf(f, "data  window: active %d cursor %d\n", drv.lane_active[0], drv.lane_cursor[0]);
		fprintf(f, "cdda  window: active %d cursor %d\n", drv.lane_active[1], drv.lane_cursor[1]);
		fclose(f);
	}
	drv.hit_count = drv.miss_count = 0;
	drv.worst_wait_ms = 0.0;
	drv.worst_io_ms = 0.0;
}

static int find_drive(char *out, int outsz);

static int drive_missing(void)
{
	if (drv.dev_fd < 0) return 1;
	if (ioctl(drv.dev_fd, CDROM_DRIVE_STATUS, CDSL_CURRENT) >= 0) return 0;
	return (errno == ENODEV || errno == ENXIO || errno == EIO || errno == ESHUTDOWN);
}

static void reattach_drive(void)
{
	char newdev[64] = "";

	pthread_mutex_lock(&drv.io_lock);
	if (drv.dev_fd >= 0) { close(drv.dev_fd); drv.dev_fd = -1; }

	int fd = find_drive(newdev, sizeof(newdev));
	if (fd >= 0) {
		drv.dev_fd = fd;
		snprintf(active_dev, 64, "%s", newdev);
		drv.reattach_count++;
		drv.fail_streak = 0;
		silence_block_probes(active_dev);
		apply_speed_cap();
		printf("DISC: drive re-attached as %s (recovered from a usb reset)\n", newdev);
	}
	else {
		printf("DISC: drive still missing, will retry\n");
	}
	pthread_mutex_unlock(&drv.io_lock);
}

static void *ring_worker_main(void *arg)
{
	(void)arg;
	int rr = 0;
	double last_stats = clock_ms();
	double last_reattach = 0;
	double last_io = clock_ms();

	double last_watch = 0;
	int was_present = -1;

	double last_swap_check = 0;
	int swap_was_present = -1;
	int swap_ejected = 0;

	while (drv.alive) {
		int target = -1;
		int rushed = 0;
		int neighbor_fill = 0;

		if (drv.watching) {
			if (clock_ms() - last_watch >= 2000) {
				last_watch = clock_ms();

				if (drive_missing() && clock_ms() - last_reattach > 5000) {
					last_reattach = clock_ms();
					reattach_drive();
					was_present = -1;
				}

				int present = physical_disc_disc_present();
				int changed = physical_disc_media_changed();

				if (present && (was_present != 1 || changed)) {
					physical_disc_forget_disc();

					physical_disc_disc_t t = physical_disc_identify();
					physical_disc_region_t r = physical_disc_region();

					if (t == PHYSICAL_DISC_DISC_NONE) {
						printf("DISC: disc present but unreadable, retrying\n");
					}
					else {
						char lbl[64];
						if (!physical_disc_disc_label(lbl, sizeof(lbl)))
							physical_disc_disc_serial(lbl, sizeof(lbl));
						pthread_mutex_lock(&drv.ring_lock);
						snprintf(drv.seen_label, sizeof(drv.seen_label), "%s", lbl);
						drv.seen_type = (int)t;
						drv.seen_present = 1;

						drv.seen_dirty = 1;
						pthread_mutex_unlock(&drv.ring_lock);

						drv.event_disc_type = (int)t;
						drv.event_region = (int)r;
						drv.event_initial = (was_present < 0) ? 1 : 0;
						drv.event_code = (int)PHYSICAL_DISC_EV_DISC_IN;
						printf("DISC: disc detected: %s%s%s%s%s\n",
							physical_disc_disc_name(t),
							*physical_disc_region_name(r) ? " region " : "",
							physical_disc_region_name(r),
							lbl[0] ? " - " : "", lbl);
						was_present = 1;
					}
				}
				else if (!present && was_present == 1) {
					pthread_mutex_lock(&drv.ring_lock);
					if (drv.seen_present) drv.seen_dirty = 1;
					drv.seen_present = 0;
					drv.seen_label[0] = 0;
					pthread_mutex_unlock(&drv.ring_lock);

					drv.event_code = (int)PHYSICAL_DISC_EV_DISC_OUT;
					printf("DISC: disc removed\n");
					was_present = 0;
				}
				else if (present) was_present = 1;
				else was_present = 0;
			}
			struct timespec ts = { 0, 50 * 1000 * 1000 };
			nanosleep(&ts, NULL);
			continue;
		}

		if (drv.swap_armed && (drv.leadout_lba > 0 || swap_ejected)
		    && clock_ms() - last_swap_check >= SWAP_POLL_MS) {
			last_swap_check = clock_ms();
			int present = physical_disc_disc_present();
			if (swap_was_present == 1 && !present) {
				swap_ejected = 1;
				drv.swap_out = 1;
			}
			else if (swap_ejected && present) {
				toc_t scratch;
				drv.mid_swap = 1;
				pthread_mutex_lock(&drv.io_lock);
				physical_disc_forget_disc();
				int ok = !physical_disc_load_toc(&scratch);
				pthread_mutex_unlock(&drv.io_lock);

				if (ok) {
					double w0 = clock_ms();
					int warm = 0;
					int wlba = scratch.tracks[0].start;
					while (clock_ms() - w0 < 8000) {
						if (!refill_ring(wlba, SYNC_IO_BURST, 1)) { warm = 1; break; }
					}
					if (warm) {
						for (int i = 1; i < 8; i++)
							if (refill_ring(wlba + i * SYNC_IO_BURST, SYNC_IO_BURST, 1)) break;
						if (drv.span[0].is_audio) { drv.lane_cursor[1] = wlba; drv.lane_active[1] = 1; }
						printf("DISC: disc swap - drive awake in %.0f ms\n", clock_ms() - w0);
					}
					ok = warm;
				}
				drv.mid_swap = 0;
				if (ok) {
					FILE *sf = fopen(SWAP_MARKER_PATH, "w");
					if (sf) fclose(sf);

					drv.swap_out = 0;
					__sync_synchronize();
					drv.swap_ready = 1;
					swap_ejected = 0;
					printf("DISC: disc swap - new toc loaded\n");
				}
			}
			swap_was_present = present;
		}

		if (drv.warmup_lba >= 0) {
			if (drv.active_lane >= 0 || drv.warmup_lba >= drv.warmup_end
			    || drv.fail_streak >= 8 || drv.leadout_lba <= 0) {
				drv.warmup_lba = -1;
			} else if (!drv.sync_busy) {
				int pw = drv.warmup_lba;
				refill_ring(pw, IO_BURST, 0);
				drv.warmup_lba = pw + IO_BURST;
				last_io = clock_ms();
				continue;
			}
		}

		pthread_mutex_lock(&drv.ring_lock);
		if (drv.rush_lba >= 0 && drv.rush_lane >= 0) {
			int rlba = drv.rush_lba;
			int rlane = drv.rush_lane;
			if (rlba < drv.leadout_lba && lane_of(rlba) == rlane && entry_for(rlba)->lba != rlba) {
				target = rlba;
				rushed = 1;
			} else {
				drv.rush_lba = -1;
				drv.rush_lane = -1;
			}
		}
		for (int n = 0; n < LANE_COUNT && target < 0; n++) {
			int w = (drv.active_lane >= 0 && n == 0) ? drv.active_lane : (rr + n) % LANE_COUNT;
			if (!drv.lane_active[w]) continue;
			int pos = drv.lane_cursor[w];
			int span = w ? AUDIO_LOOKAHEAD : LOOKAHEAD_SPAN;
			for (int i = 0; i < span; i++) {
				int lba = pos + i;
				if (lba >= drv.leadout_lba) break;
				if (lane_of(lba) != w) break;
				if (entry_for(lba)->lba != lba) { target = lba; break; }
			}
		}
		pthread_mutex_unlock(&drv.ring_lock);
		rr = (rr + 1) % LANE_COUNT;

		if (clock_ms() - last_stats >= STATS_PERIOD_MS) {
			if (drv.hit_count || drv.miss_count) write_stats_log();
			last_stats = clock_ms();
		}

		if (drv.sync_busy) {
			struct timespec ts = { 0, 2 * 1000 * 1000 };
			nanosleep(&ts, NULL);
			continue;
		}

		if (drv.fail_streak >= 8 && clock_ms() - last_reattach > 5000) {
			last_reattach = clock_ms();
			if (drive_missing()) reattach_drive();
			else drv.fail_streak = 0;
		}

		if (target < 0 && drv.neighbor_lba >= 0) {
			if (drv.neighbor_lba >= drv.neighbor_end || drv.leadout_lba <= 0) {
				drv.neighbor_lba = -1;
			} else {
				target = drv.neighbor_lba;
				neighbor_fill = 1;
			}
		}

		if (target < 0) {
			int lw = drv.active_lane;
			int klba = (lw >= 0) ? drv.lane_cursor[lw] : (drv.leadout_lba > 0 ? drv.lane_cursor[0] : -1);
			if (klba >= drv.leadout_lba) klba = drv.leadout_lba - 1;
			if (drv.leadout_lba > 0 && klba >= 0
			    && clock_ms() - last_io >= IDLE_KEEPALIVE_MS && !drv.sync_busy) {
				uint8_t sc[ENTRY_SIZE];
				int t = span_index(klba);
				uint8_t fl = (t >= 0 && drv.span[t].is_audio) ? 0x10 : 0xF8;
				pthread_mutex_lock(&drv.io_lock);
				scsi_read_cd(klba, 1, fl, 0, sc, BG_IO_TIMEOUT_MS);
				pthread_mutex_unlock(&drv.io_lock);
				last_io = clock_ms();
			}

			struct timespec ts = { 0, 20 * 1000 * 1000 };
			nanosleep(&ts, NULL);
			continue;
		}
		refill_ring(target, IO_BURST, 0);
		if (rushed) {
			drv.rush_lba = -1;
			drv.rush_lane = -1;
		}
		if (neighbor_fill) drv.neighbor_lba += IO_BURST;
		last_io = clock_ms();
	}
	return NULL;
}

void physical_disc_set_device(const char *dev)
{
	if (dev && *dev) snprintf(preferred_dev, sizeof(preferred_dev), "%s", dev);
	else preferred_dev[0] = 0;
}

static int find_drive(char *out, int outsz)
{
	char path[64];
	int spare = -1;

	if (preferred_dev[0]) {
		int fd = open(preferred_dev, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
		if (fd >= 0) { snprintf(out, outsz, "%s", preferred_dev); return fd; }
		printf("DISC: %s not available (%s), scanning\n", preferred_dev, strerror(errno));
	}

	for (int i = 0; i < 8; i++) {
		snprintf(path, sizeof(path), "/dev/sr%d", i);
		int fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
		if (fd < 0) continue;

		if (ioctl(fd, CDROM_DRIVE_STATUS, CDSL_CURRENT) == CDS_DISC_OK) {
			snprintf(out, outsz, "%s", path);
			if (spare >= 0) close(spare);
			return fd;
		}

		if (spare < 0) { spare = fd; snprintf(out, outsz, "%s", path); }
		else close(fd);
	}
	return spare;
}

int physical_disc_open(const char *dev)
{
	if (dev && *dev) physical_disc_set_device(dev);

	if (drv.dev_fd >= 0) {
		if (!preferred_dev[0] || !strcmp(preferred_dev, active_dev)) return 0;
		physical_disc_close();
	}

	drv.dev_fd = find_drive(active_dev, sizeof(active_dev));
	if (drv.dev_fd < 0) {
		active_dev[0] = 0;
		printf("DISC: no cd-rom drive found (looked at /dev/sr0../dev/sr7)\n");
		return -1;
	}

	drv.ring = (cache_entry_t *)malloc(sizeof(cache_entry_t) * RING_SECTORS);
	if (!drv.ring) { close(drv.dev_fd); drv.dev_fd = -1; return -1; }
	for (int i = 0; i < RING_SECTORS; i++) drv.ring[i].lba = -1;

	silence_block_probes(active_dev);
	apply_speed_cap();

	drv.leadout_lba = 0;
	drv.track_count = 0;
	drv.subch_ok = -1;
	for (int w = 0; w < LANE_COUNT; w++) { drv.lane_cursor[w] = 0; drv.lane_active[w] = 0; }
	drv.active_lane = -1;
	drv.warmup_lba = -1;
	drv.hit_count = drv.miss_count = 0;
	drv.worst_wait_ms = 0.0;
	drv.alive = 1;
	pthread_create(&drv.io_thread, NULL, ring_worker_main, NULL);

	cpu_set_t set;
	CPU_ZERO(&set);
	CPU_SET(0, &set);
	CPU_SET(1, &set);
	pthread_setaffinity_np(drv.io_thread, sizeof(set), &set);

	printf("\x1b[32mDISC: opened %s\n\x1b[0m", active_dev);
	return 0;
}

int physical_disc_disc_present()
{
	if (drv.dev_fd < 0) return 0;
	return ioctl(drv.dev_fd, CDROM_DRIVE_STATUS, CDSL_CURRENT) == CDS_DISC_OK;
}

int physical_disc_media_changed()
{
	if (drv.dev_fd < 0) return 0;
	return ioctl(drv.dev_fd, CDROM_MEDIA_CHANGED, CDSL_CURRENT) > 0;
}

int physical_disc_drive_busy()
{
	return drv.dev_fd >= 0;
}

int physical_disc_swap_ejected(void)
{
	return drv.swap_armed && drv.swap_out;
}

int physical_disc_swap_happened(void)
{
	return unlink(SWAP_MARKER_PATH) == 0;
}

void physical_disc_swap_enable(int enable)
{
	drv.swap_armed = enable ? 1 : 0;
	if (enable) unlink(SWAP_MARKER_PATH);
	if (!enable) drv.swap_out = 0;
	if (!enable) drv.swap_ready = 0;
}

int physical_disc_swap_consume(void)
{
	int r = drv.swap_ready;
	drv.swap_ready = 0;
	if (r) __sync_synchronize();
	return r;
}

static void detect_subchannel_support(int lba)
{
	uint8_t buf[ENTRY_SIZE];
	int t = span_index(lba);
	uint8_t flags = (t >= 0 && drv.span[t].is_audio) ? 0x10 : 0xF8;

	if (!scsi_read_cd(lba, 1, flags, 1, buf, BG_IO_TIMEOUT_MS)) { drv.subch_ok = 1; return; }
	if (!scsi_read_cd(lba, 1, flags, 0, buf, BG_IO_TIMEOUT_MS)) { drv.subch_ok = 0; return; }
	drv.subch_ok = -1;
}

static int scsi_read_raw_subq(int lba, uint8_t *sub)
{
	uint8_t cdb[12] = { 0 };
	uint8_t sense[32];
	struct sg_io_hdr io;

	cdb[0] = 0xBE;
	cdb[2] = (lba >> 24) & 0xFF;
	cdb[3] = (lba >> 16) & 0xFF;
	cdb[4] = (lba >> 8) & 0xFF;
	cdb[5] = lba & 0xFF;
	cdb[8] = 1;
	cdb[10] = 0x01;

	memset(&io, 0, sizeof(io));
	io.interface_id = 'S';
	io.cmd_len = 12;
	io.cmdp = cdb;
	io.dxfer_direction = SG_DXFER_FROM_DEV;
	io.dxfer_len = PHYSICAL_DISC_SUB;
	io.dxferp = sub;
	io.sbp = sense;
	io.mx_sb_len = sizeof(sense);
	io.timeout = BG_IO_TIMEOUT_MS;

	if (ioctl(drv.dev_fd, SG_IO, &io) < 0) return -1;
	if (io.status || io.host_status || io.driver_status) return -1;
	return 0;
}

static void raw_subq_decode(const uint8_t *raw, uint8_t *q)
{
	for (int b = 0; b < 12; b++) {
		uint8_t v = 0;
		for (int bit = 0; bit < 8; bit++)
			v = (uint8_t)((v << 1) | ((raw[b * 8 + bit] >> 6) & 1));
		q[b] = v;
	}
}

static int bcd_value(uint8_t v)
{
	int hi = (v >> 4) & 0x0F;
	int lo = v & 0x0F;
	if (hi > 9 || lo > 9) return -1;
	return hi * 10 + lo;
}

static uint16_t subq_crc16(const uint8_t *q)
{
	uint16_t crc = 0;
	for (int i = 0; i < 10; i++) {
		crc ^= (uint16_t)q[i] << 8;
		for (int b = 0; b < 8; b++)
			crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
	}
	return (uint16_t)~crc;
}

static int subq_crc_valid(const uint8_t *q)
{
	uint16_t expected = ((uint16_t)q[10] << 8) | q[11];
	return subq_crc16(q) == expected;
}

static int physical_disc_q_index00(int lba, int track)
{
	if (lba < 0 || drv.dev_fd < 0) return 0;

	uint8_t raw[PHYSICAL_DISC_SUB];
	uint8_t q[12];
	int r = scsi_read_raw_subq(lba, raw);
	if (r) return -1;

	raw_subq_decode(raw, q);
	if (!subq_crc_valid(q)) return 0;
	if ((q[0] & 0x0F) != 1) return 0;

	int am = bcd_value(q[7]);
	int as = bcd_value(q[8]);
	int af = bcd_value(q[9]);
	if (am < 0 || as < 0 || af < 0 || as >= 60 || af >= 75) return 0;
	int q_lba = (am * 60 + as) * 75 + af - 150;
	if (q_lba < lba - 1 || q_lba > lba + 1) return 0;

	return bcd_value(q[1]) == track && bcd_value(q[2]) == 0;
}

int physical_disc_psx_enrich_toc(toc_t *toc)
{
	if (!toc || !toc->phys || toc->last < 2 || drv.dev_fd < 0) return 0;

	int found = 0;
	int q_supported = 0;

	pthread_mutex_lock(&drv.io_lock);

	for (int i = 1; i < toc->last; i++) {
		int index1 = toc->tracks[i].start;
		int track = i + 1;
		int state = 0;
		int last_inside = -1;
		for (int back = 1; back <= INDEX00_ENTRY_PROBES && index1 - back >= 0; back++) {
			state = physical_disc_q_index00(index1 - back, track);
			if (state < 0) break;
			q_supported = 1;
			if (state == 1) { last_inside = index1 - back; break; }
		}
		if (state < 0) {
			if (!q_supported && !found)
				printf("DISC: PSX pregap scan unavailable on this drive, using basic TOC\n");
			break;
		}
		if (last_inside < 0) continue;

		int lower = toc->tracks[i - 1].start;
		int first_outside = lower - 1;
		int distance = 2;

		while (index1 - distance >= lower) {
			int probe = index1 - distance;
			state = physical_disc_q_index00(probe, track);
			if (state != 1) {
				first_outside = probe;
				break;
			}
			last_inside = probe;
			if (probe == lower) break;
			if (distance > (index1 - lower) / 2) distance = index1 - lower;
			else distance *= 2;
			if (!distance) break;
		}

		int lo = first_outside + 1;
		int hi = last_inside;
		while (lo < hi) {
			int mid = lo + (hi - lo) / 2;
			state = physical_disc_q_index00(mid, track);
			if (state == 1) hi = mid;
			else lo = mid + 1;
		}

		if (lo >= index1 || physical_disc_q_index00(lo, track) != 1) continue;

		int pregap = index1 - lo;
		if (pregap <= 0 || pregap > MAX_PREGAP_SECTORS || lo <= toc->tracks[i - 1].start) continue;
		toc->tracks[i].start = lo;
		toc->tracks[i].indexes[1] = pregap;
		toc->tracks[i - 1].end = lo;
		drv.span[i].lo = lo;
		drv.span[i - 1].hi = lo;
		found++;
		printf("DISC: PSX track %02d INDEX 00 at LBA %d, INDEX 01 at %d (%d sectors)\n",
			track, lo, index1, pregap);
	}

	pthread_mutex_unlock(&drv.io_lock);

	if (found && drv.ring) {
		pthread_mutex_lock(&drv.ring_lock);
		for (int i = 0; i < RING_SECTORS; i++) drv.ring[i].lba = -1;
		pthread_mutex_unlock(&drv.ring_lock);
	}

	return found;
}

int physical_disc_load_toc(toc_t *toc)
{
	struct cdrom_tochdr hdr;
	if (drv.dev_fd < 0 || !physical_disc_disc_present()) return -1;
	if (ioctl(drv.dev_fd, CDROMREADTOCHDR, &hdr) < 0) return -1;

	memset(toc, 0, sizeof(toc_t));
	drv.leadout_lba = 0;
	drv.data_lba0 = -1;
	drv.track_count = 0;

	int n = 0;
	for (int t = hdr.cdth_trk0; t <= hdr.cdth_trk1 && n < 99; t++, n++) {
		struct cdrom_tocentry e;
		memset(&e, 0, sizeof(e));
		e.cdte_track = t;
		e.cdte_format = CDROM_LBA;
		if (ioctl(drv.dev_fd, CDROMREADTOCENTRY, &e) < 0) return -1;

		cd_track_t *out_trk = &toc->tracks[n];
		out_trk->start = e.cdte_addr.lba;
		out_trk->type = (e.cdte_ctrl & CDROM_DATA_TRACK) ? TT_MODE1 : TT_CDDA;
		out_trk->sector_size = PHYSICAL_DISC_RAW;
		out_trk->offset = 0;
		out_trk->index_num = 2;
		out_trk->indexes[0] = 0;
		out_trk->indexes[1] = 0;

		drv.span[n].lo = out_trk->start;
		drv.span[n].is_audio = (out_trk->type == TT_CDDA);

		if (out_trk->type != TT_CDDA && drv.data_lba0 < 0)
			drv.data_lba0 = out_trk->start;
		if (n > 0) {
			toc->tracks[n - 1].end = out_trk->start;
			drv.span[n - 1].hi = out_trk->start;
		}
	}

	if (n < 1) {
		printf("DISC: drive reported no tracks (%d-%d)\n", hdr.cdth_trk0, hdr.cdth_trk1);
		return -1;
	}

	struct cdrom_tocentry lead;
	memset(&lead, 0, sizeof(lead));
	lead.cdte_track = CDROM_LEADOUT;
	lead.cdte_format = CDROM_LBA;
	if (ioctl(drv.dev_fd, CDROMREADTOCENTRY, &lead) < 0) return -1;

	toc->tracks[n - 1].end = lead.cdte_addr.lba;
	toc->last = n;
	toc->end = lead.cdte_addr.lba;
	toc->sectorSize = PHYSICAL_DISC_RAW;
	toc->phys = 1;

	drv.span[n - 1].hi = lead.cdte_addr.lba;
	drv.track_count = n;

	pthread_mutex_lock(&drv.ring_lock);
	for (int i = 0; i < RING_SECTORS; i++) drv.ring[i].lba = -1;
	pthread_mutex_unlock(&drv.ring_lock);

	apply_speed_cap();

	drv.subch_ok = -1;
	detect_subchannel_support(toc->tracks[0].start + 16);

	for (int w = 0; w < LANE_COUNT; w++) { drv.lane_cursor[w] = 0; drv.lane_active[w] = 0; }
	drv.lane_cursor[0] = toc->tracks[0].start;
	drv.active_lane = -1;
	drv.rush_lane = -1;
	drv.rush_lba = -1;
	drv.neighbor_lba = -1;
	drv.neighbor_end = 0;
	drv.hit_count = drv.miss_count = drv.bad_count = drv.bad_logged = 0;
	drv.worst_wait_ms = drv.worst_io_ms = 0.0;

	drv.leadout_lba = lead.cdte_addr.lba;

	drv.warmup_lba = -1;
	if (!drv.mid_swap && drv.track_count) {
		int prime = STARTUP_PRIME_SECTORS;
		int available = drv.span[0].hi - toc->tracks[0].start;
		if (prime > available) prime = available;
		if (prime > 0) {
			int pr = refill_ring(toc->tracks[0].start, prime, 1);
			printf("DISC: startup prime %s (%d sectors)\n", pr ? "incomplete" : "ready", prime);
		}
		int pw_end = toc->tracks[0].start + WARMUP_SECTORS;
		if (pw_end > drv.span[0].hi) pw_end = drv.span[0].hi;
		if (pw_end > drv.leadout_lba) pw_end = drv.leadout_lba;

		int w = lane_of(toc->tracks[0].start);
		drv.lane_cursor[w] = toc->tracks[0].start;
		drv.lane_active[w] = 1;
		drv.warmup_end = pw_end;
		__sync_synchronize();
		drv.warmup_lba = toc->tracks[0].start;
	}

	printf("\x1b[32mDISC: toc loaded, %d tracks, leadout %d, subchannel %s\n\x1b[0m",
		n, drv.leadout_lba,
		drv.subch_ok == 1 ? "yes" : drv.subch_ok == 0 ? "no" : "unknown");
	return 0;
}

int physical_disc_current_toc(toc_t *toc)
{
	if (!toc || drv.dev_fd < 0 || drv.mid_swap || drv.track_count < 1 || drv.leadout_lba <= 0) return -1;

	memset(toc, 0, sizeof(toc_t));
	for (int i = 0; i < drv.track_count; i++) {
		cd_track_t *out_trk = &toc->tracks[i];
		out_trk->start = drv.span[i].lo;
		out_trk->end = drv.span[i].hi;
		out_trk->type = drv.span[i].is_audio ? TT_CDDA : TT_MODE1;
		out_trk->sector_size = PHYSICAL_DISC_RAW;
		out_trk->offset = 0;
		out_trk->index_num = 2;
		out_trk->indexes[0] = 0;
		out_trk->indexes[1] = 0;
	}
	toc->last = drv.track_count;
	toc->end = drv.leadout_lba;
	toc->sectorSize = PHYSICAL_DISC_RAW;
	toc->phys = 1;
	return 0;
}

int physical_disc_toc_audio_only(const toc_t *toc)
{
	if (!toc || toc->last <= 0) return 0;
	for (int i = 0; i < toc->last; i++)
	{
		if (toc->tracks[i].type != TT_CDDA) return 0;
	}
	return 1;
}

void physical_disc_prewarm_blocking(void)
{
	if (drv.dev_fd < 0 || drv.track_count < 1 || drv.leadout_lba <= 0) return;
	int lba = drv.span[0].lo;
	double start = clock_ms();
	int ready = 0;
	while (clock_ms() - start < 8000)
	{
		if (!refill_ring(lba, SYNC_IO_BURST, 1))
		{
			ready = 1;
			break;
		}
	}
	if (!ready) return;
	int remaining = drv.leadout_lba - lba;
	if (remaining > 8 * SYNC_IO_BURST)
	{
		static const int points[] = { 2, 3, 1 };
		for (unsigned i = 0; i < sizeof(points) / sizeof(points[0]); i++)
		{
			int target = lba + (int)(((int64_t)remaining * points[i]) / 4);
			if (target + SYNC_IO_BURST > drv.leadout_lba) target = drv.leadout_lba - SYNC_IO_BURST;
			refill_ring(target, SYNC_IO_BURST, 1);
		}
	}
	for (int i = 1; i < 8; i++)
	{
		if (refill_ring(lba + i * SYNC_IO_BURST, SYNC_IO_BURST, 1)) break;
	}
	drv.lane_cursor[0] = lba;
	drv.lane_active[0] = 1;
}

void physical_disc_seek_hint(int lba)
{
	if (lba < 0 || !drv.track_count || lba >= drv.leadout_lba) return;

	int w = lane_of(lba);
	drv.lane_cursor[w] = lba;
	drv.lane_active[w] = 1;
	drv.active_lane = w;
	drv.rush_lane = w;
	drv.rush_lba = lba;

	int t = span_index(lba);
	if (t >= 0 && drv.span[t].is_audio && (lba - drv.span[t].lo) < NEIGHBOR_ENTRY_SPAN) {
		int nt = t + 1;
		if (nt >= drv.track_count || !drv.span[nt].is_audio) {
			nt = -1;
			for (int i = 0; i < drv.track_count; i++) {
				if (drv.span[i].is_audio) { nt = i; break; }
			}
		}
		if (nt >= 0 && nt != t) {
			drv.neighbor_lba = drv.span[nt].lo;
			int end = drv.span[nt].lo + NEIGHBOR_PREWARM_SECTORS;
			if (end > drv.span[nt].hi) end = drv.span[nt].hi;
			drv.neighbor_end = end;
		}
	}
}

static int fetch_sector(int lba, uint8_t *dst, uint8_t *sub96, int *sub_valid, int mark_active)
{
	if (sub_valid) *sub_valid = 0;

	if (drv.dev_fd < 0 || drv.mid_swap || lba < 0 || lba >= drv.leadout_lba) {
		memset(dst, 0, PHYSICAL_DISC_RAW);
		if (sub96) memset(sub96, 0, PHYSICAL_DISC_SUB);
		return -1;
	}

	int w = lane_of(lba);
	drv.lane_active[w] = 1;
	if (mark_active) drv.active_lane = w;

	pthread_mutex_lock(&drv.ring_lock);
	cache_entry_t *e = entry_for(lba);
	int hit = (e->lba == lba);
	int unreadable = hit && e->unreadable;
	if (hit && !unreadable) {
		memcpy(dst, e->data, PHYSICAL_DISC_RAW);
		if (sub96) memcpy(sub96, e->data + PHYSICAL_DISC_RAW, PHYSICAL_DISC_SUB);
		if (sub_valid) *sub_valid = e->sub_present;
	}
	pthread_mutex_unlock(&drv.ring_lock);

	if (hit) {
		if (lba >= drv.lane_cursor[w]) drv.lane_cursor[w] = lba + 1;
		drv.hit_count++;
		if (unreadable) {
			memset(dst, 0, PHYSICAL_DISC_RAW);
			if (sub96) memset(sub96, 0, PHYSICAL_DISC_SUB);
			return -1;
		}
		return 0;
	}

	double t0 = clock_ms();
	drv.sync_busy++;
	int request_count = w ? AUDIO_SYNC_BURST : SYNC_IO_BURST;
	int fr = refill_ring(lba, request_count, 1);
	drv.sync_busy--;

	drv.miss_count++;
	double d = clock_ms() - t0;
	if (d > drv.worst_wait_ms) drv.worst_wait_ms = d;

	if (!fr) {
		pthread_mutex_lock(&drv.ring_lock);
		e = entry_for(lba);
		hit = (e->lba == lba);
		unreadable = hit && e->unreadable;
		if (hit && !unreadable) {
			memcpy(dst, e->data, PHYSICAL_DISC_RAW);
			if (sub96) memcpy(sub96, e->data + PHYSICAL_DISC_RAW, PHYSICAL_DISC_SUB);
			if (sub_valid) *sub_valid = e->sub_present;
		}
		pthread_mutex_unlock(&drv.ring_lock);

		if (hit) {
			drv.lane_cursor[w] = lba + 1;
			if (unreadable) {
				memset(dst, 0, PHYSICAL_DISC_RAW);
				if (sub96) memset(sub96, 0, PHYSICAL_DISC_SUB);
				return -1;
			}
			return 0;
		}
	}

	memset(dst, 0, PHYSICAL_DISC_RAW);
	if (sub96) memset(sub96, 0, PHYSICAL_DISC_SUB);
	drv.bad_count++;
	drv.lane_cursor[w] = lba;
	return -1;
}

int physical_disc_read_sector(int lba, uint8_t *dst, uint8_t *sub96)
{
	return fetch_sector(lba, dst, sub96, NULL, 1);
}

int physical_disc_probe_sector(int lba, uint8_t *dst)
{
	return fetch_sector(lba, dst, NULL, NULL, 0);
}

int physical_disc_read_sector_sub(int lba, uint8_t *dst, uint8_t *sub96)
{
	int valid = 0;
	if (fetch_sector(lba, dst, sub96, &valid, 1)) return 0;
	return valid;
}

int physical_disc_read_data2048(int lba, uint8_t *dst)
{
	uint8_t raw[PHYSICAL_DISC_RAW];
	if (physical_disc_read_sector(lba, raw, NULL)) return -1;

	int off = (raw[15] == 2) ? 24 : 16;
	memcpy(dst, raw + off, 2048);
	return 0;
}

static uint32_t iso_le32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int iso_root_features(int base, int *has_mdplus, int *has_snes)
{
	uint8_t pvd[2048];
	*has_mdplus = 0;
	*has_snes = 0;
	if (physical_disc_read_data2048(base + 16, pvd)) return 0;
	if (pvd[0] != 1 || memcmp(pvd + 1, "CD001", 5)) return 0;
	const uint8_t *root = pvd + 156;
	if (root[0] < 34) return 0;
	uint32_t extent = iso_le32(root + 2);
	uint32_t size = iso_le32(root + 10);
	if (!extent || !size) return 0;
	char md_names[64][128];
	char cue_names[64][128];
	int md_count = 0, cue_count = 0;
	uint32_t done = 0;
	while (done < size && done < 1024 * 1024) {
		uint8_t sec[2048];
		if (physical_disc_read_data2048(base + extent + done / 2048, sec)) break;
		int off = 0;
		while (off < 2048 && done + off < size) {
			int len = sec[off];
			if (!len) break;
			if (off + len > 2048 || len < 34) break;
			int nlen = sec[off + 32];
			if (nlen > 0 && nlen < 120 && off + 33 + nlen <= 2048) {
				char name[128];
				memcpy(name, sec + off + 33, nlen);
				name[nlen] = 0;
				char *semi = strchr(name, ';');
				if (semi) *semi = 0;
				char *dot = strrchr(name, '.');
				if (dot && (!strcasecmp(dot, ".sfc") || !strcasecmp(dot, ".smc"))) *has_snes = 1;
				if (dot && !strcasecmp(dot, ".md") && md_count < 64) { *dot = 0; snprintf(md_names[md_count++], 128, "%s", name); }
				else if (dot && !strcasecmp(dot, ".cue") && cue_count < 64) { *dot = 0; snprintf(cue_names[cue_count++], 128, "%s", name); }
			}
			off += len;
		}
		done += 2048;
	}
	for (int i = 0; i < md_count; i++) for (int j = 0; j < cue_count; j++) if (!strcasecmp(md_names[i], cue_names[j])) *has_mdplus = 1;
	return 1;
}

physical_disc_disc_t physical_disc_identify()
{
	uint8_t raw[PHYSICAL_DISC_RAW * 2];

	if (!physical_disc_disc_present()) return PHYSICAL_DISC_DISC_NONE;
	if (drv.data_lba0 < 0) {
		toc_t tmp;
		if (physical_disc_load_toc(&tmp)) return PHYSICAL_DISC_DISC_NONE;
		if (drv.data_lba0 < 0) return PHYSICAL_DISC_DISC_AUDIO;
	}

	int base = drv.data_lba0;
	int has_mdplus = 0, has_snes = 0;
	iso_root_features(base, &has_mdplus, &has_snes);
	if (has_mdplus) return PHYSICAL_DISC_DISC_MDPLUS;

	if (!physical_disc_read_sector(base, raw, NULL)) {
		if (!memcmp(raw + 16, "SEGADISCSYSTEM", 14)) return PHYSICAL_DISC_DISC_MEGACD;
		if (!memcmp(raw + 16, "SEGA SEGASATURN", 15)) return PHYSICAL_DISC_DISC_SATURN;
		if (raw[16] == 0x01 && raw[17] == 0x5A && raw[18] == 0x5A && raw[19] == 0x5A && raw[20] == 0x5A && raw[21] == 0x5A) return PHYSICAL_DISC_DISC_3DO;
	}

	if (!physical_disc_read_sector(base + 16, raw, NULL)) {
		uint8_t *iso = raw + 16;
		if (memcmp(iso + 1, "CD001", 5) && memcmp(iso + 1, "CD-I", 4)) iso = raw + 24;
		if (!memcmp(iso + 1, "CD001", 5)) {
			if (!memcmp(iso + 8, "PLAYSTATION", 11)) return PHYSICAL_DISC_DISC_PSX;
			if (!memcmp(iso + 8, "NGCD", 4)) return PHYSICAL_DISC_DISC_NEOGEO;
		}
		if (!memcmp(iso + 1, "CD-I", 4)) return PHYSICAL_DISC_DISC_CDI;
	}

	for (int s = 16; s <= 40; s++) {
		uint8_t user[2048];
		if (physical_disc_read_data2048(base + s, user)) continue;
		if (memmem(user, sizeof(user), "IPL.TXT", 7)) return PHYSICAL_DISC_DISC_NEOGEO;
		if (memmem(user, sizeof(user), "CDI_APPL", 8)) return PHYSICAL_DISC_DISC_CDI;
	}

	if (!physical_disc_read_sector(base, raw, NULL) && !physical_disc_read_sector(base + 1, raw + PHYSICAL_DISC_RAW, NULL)) {
		for (int off = 0; off < (int)sizeof(raw) - 24; off++) if (!memcmp(raw + off, "PC Engine CD-ROM SYSTEM", 23)) return PHYSICAL_DISC_DISC_PCECD;
	}

	if (has_snes) return PHYSICAL_DISC_DISC_SNES;
	return PHYSICAL_DISC_DISC_UNKNOWN;
}

physical_disc_region_t physical_disc_region_from_md_header(const uint8_t *hdr, int len)
{
	if (!hdr || len < 0x1F3) return PHYSICAL_DISC_REGION_UNKNOWN;

	if (memcmp(hdr + 0x100, "SEGA", 4)) return PHYSICAL_DISC_REGION_UNKNOWN;

	const char *f = (const char *)hdr + 0x1F0;
	int has_j = 0, has_u = 0, has_e = 0, junk = 0;
	for (int i = 0; i < 3; i++) {
		char c = f[i];
		if (c == 'J') has_j = 1;
		else if (c == 'U') has_u = 1;
		else if (c == 'E') has_e = 1;
		else if (c != ' ' && c != 0) junk = 1;
	}

	if (!junk && (has_j || has_u || has_e)) {
		if (has_u) return PHYSICAL_DISC_REGION_US;
		if (has_e) return PHYSICAL_DISC_REGION_EU;
		return PHYSICAL_DISC_REGION_JP;
	}

	int v = -1;
	if (f[0] >= '0' && f[0] <= '9') v = f[0] - '0';
	else if (f[0] >= 'A' && f[0] <= 'F') v = f[0] - 'A' + 10;
	if (v > 0) {
		if (v & 4) return PHYSICAL_DISC_REGION_US;
		if (v & 8) return PHYSICAL_DISC_REGION_EU;
		if (v & 1) return PHYSICAL_DISC_REGION_JP;
	}

	return PHYSICAL_DISC_REGION_UNKNOWN;
}

physical_disc_region_t physical_disc_region()
{
	uint8_t user[2048];

	if (!physical_disc_disc_present()) return PHYSICAL_DISC_REGION_UNKNOWN;
	if (drv.data_lba0 < 0) {
		toc_t tmp;
		if (physical_disc_load_toc(&tmp)) return PHYSICAL_DISC_REGION_UNKNOWN;
		if (drv.data_lba0 < 0) return PHYSICAL_DISC_REGION_UNKNOWN;
	}

	if (physical_disc_read_data2048(drv.data_lba0, user)) return PHYSICAL_DISC_REGION_UNKNOWN;
	return physical_disc_region_from_md_header(user, sizeof(user));
}

const char *physical_disc_region_name(physical_disc_region_t r)
{
	switch (r) {
	case PHYSICAL_DISC_REGION_JP: return "JP";
	case PHYSICAL_DISC_REGION_US: return "US";
	case PHYSICAL_DISC_REGION_EU: return "EU";
	default:               return "";
	}
}

int physical_disc_disc_label(char *out, int outsz)
{
	uint8_t user[2048];

	if (!out || outsz < 2) return 0;
	out[0] = 0;

	if (!physical_disc_disc_present()) return 0;
	if (drv.data_lba0 < 0) {
		toc_t tmp;
		if (physical_disc_load_toc(&tmp)) return 0;
		if (drv.data_lba0 < 0) return 0;
	}

	if (physical_disc_read_data2048(drv.data_lba0 + 16, user)) return 0;
	if (user[0] != 1 || memcmp(user + 1, "CD001", 5)) return 0;

	char lbl[33];
	memcpy(lbl, user + 40, 32);
	lbl[32] = 0;

	int end = 32;
	while (end > 0 && (lbl[end - 1] == ' ' || lbl[end - 1] == 0)) end--;
	lbl[end] = 0;

	for (int i = 0; i < end; i++) {
		if (lbl[i] == '_') lbl[i] = ' ';
		else if (lbl[i] < 0x20 || (uint8_t)lbl[i] > 0x7E) lbl[i] = ' ';
	}

	if (!strcasecmp(lbl, "PLAYSTATION")) return 0;

	snprintf(out, outsz, "%s", lbl);
	return strlen(out);
}

int physical_disc_disc_serial(char *out, int outsz)
{
	static const char *pfx[] = {
		"SCES","SLES","SCUS","SLUS","SCPS","SLPS","SLPM","SCPM",
		"SIPS","SCED","SLED","SCZS","PAPX","PCPX","PEPX","PUPX"
	};
	if (!out || outsz < 2) return 0;
	out[0] = 0;
	if (drv.data_lba0 < 0) return 0;

	for (int s = 16; s <= 64; s++) {
		uint8_t user[2048];
		if (physical_disc_read_data2048(drv.data_lba0 + s, user)) continue;

		for (int p = 0; p < (int)(sizeof(pfx) / sizeof(pfx[0])); p++) {
			uint8_t *m = (uint8_t *)memmem(user, sizeof(user), pfx[p], 4);
			if (!m) continue;

			char *start = (char *)m;
			char *semi = (char *)memmem(start, sizeof(user) - (start - (char *)user), ";", 1);
			if (!semi) continue;
			int len = (int)(semi - start);
			if (len < 8 || len > 11) continue;

			char id[16];
			memcpy(id, start, len);
			id[len] = 0;
			if (id[4] == '_') id[4] = '-';
			char *dot = strchr(id, '.');
			if (dot) memmove(dot, dot + 1, strlen(dot));
			snprintf(out, outsz, "%s", id);
			return (int)strlen(out);
		}
	}
	return 0;
}

static int sanitize_name(const char *src, int len, char *out, int outsz)
{
	if (!src || !out || outsz < 2) return 0;
	int n = 0;
	int pending = 0;
	for (int i = 0; i < len && src[i]; i++)
	{
		unsigned char c = (unsigned char)src[i];
		if (c == ' ' || c == '_' || c == '/' || c == '\\' || c == ':' || c == ';' || c == ',')
		{
			if (n) pending = 1;
			continue;
		}
		if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || c == '-' || c == '.')) continue;
		if (pending && n < outsz - 1) out[n++] = '_';
		pending = 0;
		if (n < outsz - 1) out[n++] = (char)c;
	}
	while (n && (out[n - 1] == '_' || out[n - 1] == '.')) n--;
	out[n] = 0;
	return n;
}

static uint64_t fnv_mix64(uint64_t h, uint64_t v)
{
	for (int i = 0; i < 8; i++)
	{
		h ^= (uint8_t)(v >> (i * 8));
		h *= 1099511628211ULL;
	}
	return h;
}

static int derive_toc_uuid(physical_disc_disc_t type, char *out, int outsz)
{
	if (!out || outsz < 37) return 0;
	toc_t toc;
	if (physical_disc_current_toc(&toc) || !toc.last)
	{
		if (physical_disc_load_toc(&toc) || !toc.last) return 0;
	}
	uint64_t h1 = 1469598103934665603ULL;
	uint64_t h2 = 1099511628211ULL;
	h1 = fnv_mix64(h1, (uint64_t)type);
	h2 = fnv_mix64(h2, (uint64_t)type ^ 0x9E3779B97F4A7C15ULL);
	h1 = fnv_mix64(h1, (uint64_t)toc.last);
	h2 = fnv_mix64(h2, (uint64_t)toc.end);
	for (int i = 0; i < toc.last; i++)
	{
		uint64_t v = ((uint64_t)(uint32_t)toc.tracks[i].start << 32) |
			((uint64_t)(uint16_t)toc.tracks[i].type << 16) |
			(uint16_t)(toc.tracks[i].end - toc.tracks[i].start);
		h1 = fnv_mix64(h1, v);
		h2 = fnv_mix64(h2, v ^ ((uint64_t)i << 56));
	}
	uint8_t b[16];
	for (int i = 0; i < 8; i++) b[i] = (uint8_t)(h1 >> (56 - i * 8));
	for (int i = 0; i < 8; i++) b[8 + i] = (uint8_t)(h2 >> (56 - i * 8));
	b[6] = (b[6] & 0x0F) | 0x50;
	b[8] = (b[8] & 0x3F) | 0x80;
	snprintf(out, outsz,
		"%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
		b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7],
		b[8], b[9], b[10], b[11], b[12], b[13], b[14], b[15]);
	return 36;
}

int physical_disc_save_name(physical_disc_disc_t type, char *out, int outsz)
{
	if (!out || outsz < 2) return 0;
	out[0] = 0;
	uint8_t user[2048];
	char id[64] = {};
	if (drv.data_lba0 < 0)
	{
		toc_t toc;
		if (physical_disc_load_toc(&toc) || drv.data_lba0 < 0) return derive_toc_uuid(type, out, outsz);
	}
	if (type == PHYSICAL_DISC_DISC_PSX)
	{
		if (physical_disc_disc_serial(id, sizeof(id)) && sanitize_name(id, strlen(id), out, outsz)) return strlen(out);
	}
	else if (type == PHYSICAL_DISC_DISC_SATURN && !physical_disc_read_data2048(drv.data_lba0, user))
	{
		if (!memcmp(user, "SEGA SEGASATURN", 15) && sanitize_name((char *)user + 0x20, 10, out, outsz)) return strlen(out);
	}
	else if (type == PHYSICAL_DISC_DISC_MEGACD && !physical_disc_read_data2048(drv.data_lba0, user))
	{
		if (!memcmp(user, "SEGADISCSYSTEM", 14) && sanitize_name((char *)user + 0x180, 14, out, outsz)) return strlen(out);
	}
	if (physical_disc_disc_label(id, sizeof(id)) && sanitize_name(id, strlen(id), out, outsz)) return strlen(out);
	return derive_toc_uuid(type, out, outsz);
}

const char *physical_disc_console_name(physical_disc_disc_t t)
{
	switch (t) {
	case PHYSICAL_DISC_DISC_MEGACD: return "Mega CD";
	case PHYSICAL_DISC_DISC_SATURN: return "Saturn";
	case PHYSICAL_DISC_DISC_PSX:    return "PlayStation";
	case PHYSICAL_DISC_DISC_PCECD:  return "TurboGrafx-CD";
	case PHYSICAL_DISC_DISC_NEOGEO: return "Neo Geo CD";
	case PHYSICAL_DISC_DISC_3DO:    return "3DO";
	default:                 return physical_disc_disc_name(t);
	}
}

const char *physical_disc_disc_name(physical_disc_disc_t t)
{
	switch (t) {
	case PHYSICAL_DISC_DISC_MEGACD: return "MegaCD";
	case PHYSICAL_DISC_DISC_SATURN: return "Saturn";
	case PHYSICAL_DISC_DISC_PSX:    return "PSX";
	case PHYSICAL_DISC_DISC_PCECD:  return "TurboGrafx CD";
	case PHYSICAL_DISC_DISC_NEOGEO: return "NeoGeo CD";
	case PHYSICAL_DISC_DISC_3DO:    return "3DO";
	case PHYSICAL_DISC_DISC_CDI:    return "CD-i";
	case PHYSICAL_DISC_DISC_MDPLUS: return "MD+";
	case PHYSICAL_DISC_DISC_SNES:   return "SNES MSU-1";
	case PHYSICAL_DISC_DISC_AUDIO:  return "Audio CD";
	case PHYSICAL_DISC_DISC_NONE:   return "No Disc";
	default:                 return "Unknown";
	}
}

int physical_disc_watch_start(void)
{
	physical_disc_acoustic_pause();
	if (physical_disc_open(NULL)) return -1;
	drv.event_code = (int)PHYSICAL_DISC_EV_NONE;
	drv.watching = 1;
	printf("DISC: watching %s for a disc\n", active_dev);
	return 0;
}

void physical_disc_watch_stop(void)
{
	drv.watching = 0;
	drv.event_code = (int)PHYSICAL_DISC_EV_NONE;

	pthread_mutex_lock(&drv.ring_lock);
	drv.seen_present = 0;
	drv.seen_type = 0;
	drv.seen_label[0] = 0;
	drv.seen_dirty = 1;
	pthread_mutex_unlock(&drv.ring_lock);

	physical_disc_close();

	physical_disc_acoustic_resume();
}

int physical_disc_watching(void)
{
	return drv.watching;
}

int physical_disc_menu_status(char *name, int namesz, physical_disc_disc_t *type)
{
	pthread_mutex_lock(&drv.ring_lock);
	int present = drv.seen_present;
	int t = drv.seen_type;

	if (name && namesz > 0) snprintf(name, namesz, "%s", drv.seen_label);
	pthread_mutex_unlock(&drv.ring_lock);

	if (type) *type = (physical_disc_disc_t)t;
	return present;
}

int physical_disc_menu_dirty(void)
{
	pthread_mutex_lock(&drv.ring_lock);
	int d = drv.seen_dirty;
	drv.seen_dirty = 0;
	pthread_mutex_unlock(&drv.ring_lock);
	return d;
}

physical_disc_event_t physical_disc_poll_event(physical_disc_disc_t *type, physical_disc_region_t *region, int *initial)
{
	physical_disc_event_t e = (physical_disc_event_t)drv.event_code;
	if (e == PHYSICAL_DISC_EV_NONE) return e;

	if (type) *type = (physical_disc_disc_t)drv.event_disc_type;
	if (region) *region = (physical_disc_region_t)drv.event_region;
	if (initial) *initial = drv.event_initial;
	drv.event_code = (int)PHYSICAL_DISC_EV_NONE;
	return e;
}

void physical_disc_forget_disc(void)
{
	drv.data_lba0 = -1;
	drv.leadout_lba = 0;
	drv.track_count = 0;
	if (drv.ring) {
		pthread_mutex_lock(&drv.ring_lock);
		for (int i = 0; i < RING_SECTORS; i++) drv.ring[i].lba = -1;
		pthread_mutex_unlock(&drv.ring_lock);
	}
}

void physical_disc_close()
{
	if (drv.dev_fd < 0) return;
	drv.alive = 0;
	pthread_join(drv.io_thread, NULL);
	free(drv.ring);
	drv.ring = NULL;

	// A Linux optical-device open or a previous filesystem mount may have
	// asserted the kernel tray lock. Every completed physical-disc session
	// must leave the drive ejectable. Ignore ENOTTY/unsupported bridges.
	ioctl(drv.dev_fd, CDROM_LOCKDOOR, 0);
	close(drv.dev_fd);
	drv.dev_fd = -1;
	drv.leadout_lba = 0;
	drv.track_count = 0;
	drv.data_lba0 = -1;
	drv.want_native_speed = 0;
	drv.subch_ok = -1;
	drv.sync_busy = 0;
	drv.fail_streak = 0;
	drv.active_lane = -1;
	drv.warmup_lba = -1;
	drv.warmup_end = 0;
	drv.rush_lane = -1;
	drv.rush_lba = -1;
	drv.neighbor_lba = -1;
	drv.neighbor_end = 0;
	for (int w = 0; w < LANE_COUNT; w++) { drv.lane_cursor[w] = 0; drv.lane_active[w] = 0; }
	active_dev[0] = 0;
}
