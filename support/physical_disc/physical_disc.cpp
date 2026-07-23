











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



#define CACHE_SECTORS 4096            
#define NWIN 2                        
#define WIN_SECTORS (CACHE_SECTORS / NWIN)
#define SLOT_SIZE (PHYSICAL_DISC_RAW + PHYSICAL_DISC_SUB)
#define READAHEAD 96                  
#define BURST 16                      
#define PREWARM_SECTORS 768           
#define STATS_MS 5000
#define SWAP_CHECK_MS 500             















#define SYNC_BURST 8
#define SYNC_TIMEOUT_MS 3000
#define BG_TIMEOUT_MS 3000





















#define PHYSICAL_DISC_SPEED_NX 4





















#define KEEPALIVE_MS 15000












typedef struct {
	int lba;                      
	int has_sub;
	int bad;                      
	uint8_t data[SLOT_SIZE];
} slot_t;

typedef struct {
	int start;
	int end;
	int audio;
} phys_trk_t;

static struct {
	volatile int fd;              
	int leadout;                  
	int first_data_lba;
	int sub_ok;                   
	phys_trk_t trk[100];
	int ntrk;
	slot_t *cache;
	volatile int cursor[NWIN];    
	volatile int wactive[NWIN];
	volatile int running;
	volatile int sync_pending;    
	pthread_t thread;
	pthread_mutex_t lock;         
	pthread_mutex_t io;           
	uint32_t st_hit, st_miss;     
	uint32_t st_bad;              
	uint32_t st_bad_logged;
	double st_worst_ms;           
	double st_worst_io_ms;        
	volatile int consec_fail;     
	uint32_t st_reattach;         
	volatile int watch_mode;      
	volatile int ev;              
	volatile int ev_type;
	volatile int ev_region;
	volatile int ev_initial;      
	int watch_present;            
	int watch_type;
	char watch_label[64];
	volatile int watch_dirty;     
	volatile int swap_enable;     
	volatile int swap_ready;      
	volatile int swapping;        
	volatile int last_win;        
	volatile int prewarm;         
	volatile int prewarm_end;     
	volatile int swap_ejected;    
	volatile int native_speed;
} pcd = { -1, 0, -1, -1, {}, 0, NULL, {0,0}, {0,0}, 0, 0, 0,
	  PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER,
	  0, 0, 0, 0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, {0}, 0, 0, 0, 0, -1, -1, 0, 0, 0 };




#define PHYSICAL_DISC_SWAPPED_MARKER "/tmp/physical_disc_swapped"

static int track_of(int lba)
{
	for (int i = 0; i < pcd.ntrk; i++)
		if (lba < pcd.trk[i].end) return i;
	return pcd.ntrk ? pcd.ntrk - 1 : -1;
}

static char pref_dev[64] = "";       
static char cur_dev[64] = "";        

static inline int win_of(int lba)
{
	int t = track_of(lba);
	return (t >= 0 && pcd.trk[t].audio) ? 1 : 0;
}

static inline slot_t *slot_for(int lba)
{
	return &pcd.cache[win_of(lba) * WIN_SECTORS + (lba % WIN_SECTORS)];
}

static double now_ms()
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}




static void set_speed_cap()
{
	if (pcd.fd < 0) return;

	int audio = 0;
	for (int i = 0; i < pcd.ntrk; i++)
		if (pcd.trk[i].audio) audio = 1;

	int speed = pcd.native_speed && pcd.ntrk > 0 && !audio ? 0 : PHYSICAL_DISC_SPEED_NX;
	if (ioctl(pcd.fd, CDROM_SELECT_SPEED, speed) < 0)
		printf("DISC: speed selection not supported, drive keeps its default\n");
	else if (speed)
		printf("DISC: speed capped at %dx (~%d KB/s, need 172)\n", speed, speed * 177);
	else
		printf("DISC: native drive speed enabled\n");
}

void physical_disc_native_speed(int enable)
{
	pcd.native_speed = enable ? 1 : 0;
	if (pcd.fd >= 0 && pcd.ntrk > 0) set_speed_cap();
}













static void quiet_block_probes(const char *dev)
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

static void install_physical_disc_rule(void)
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
	install_physical_disc_rule();
}



static int sg_read_cd(int lba, int count, uint8_t flags, int with_sub, uint8_t *dst, int timeout_ms)
{
	uint8_t cdb[12] = { 0 };
	uint8_t sense[32];
	struct sg_io_hdr io;
	int sector_len = PHYSICAL_DISC_RAW + (with_sub ? PHYSICAL_DISC_SUB : 0);

	cdb[0] = 0xBE;                          
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

	if (ioctl(pcd.fd, SG_IO, &io) < 0) return -1;
	if (io.status || io.host_status || io.driver_status) return -2;
	return 0;
}



static int cooked_read_raw(int lba, uint8_t *dst)
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

	if (ioctl(pcd.fd, CDROMREADRAW, &req) < 0) return -1;
	memcpy(dst, req.raw, PHYSICAL_DISC_RAW);
	return 0;
}





static int fill_cache(int lba, int count, int sync)
{
	uint8_t burst[BURST * SLOT_SIZE];     

	if (count > BURST) count = BURST;
	if (lba < 0) lba = 0;
	if (lba + count > pcd.leadout) count = pcd.leadout - lba;
	if (count <= 0) return 0;

	int t = track_of(lba);
	if (t >= 0 && lba + count > pcd.trk[t].end) count = pcd.trk[t].end - lba;
	uint8_t flags = (t >= 0 && pcd.trk[t].audio) ? 0x10 : 0xF8;

	int with_sub = (pcd.sub_ok == 1);

	


	double io0 = now_ms();
	pthread_mutex_lock(&pcd.io);
	int r = sg_read_cd(lba, count, flags, with_sub, burst,
		sync ? SYNC_TIMEOUT_MS : BG_TIMEOUT_MS);
	pthread_mutex_unlock(&pcd.io);

	if (r && with_sub && count > 1) {
		








		pthread_mutex_lock(&pcd.io);
		int r2 = sg_read_cd(lba, count, flags, 0, burst,
			sync ? SYNC_TIMEOUT_MS : BG_TIMEOUT_MS);
		pthread_mutex_unlock(&pcd.io);
		if (!r2) {
			printf("DISC: drive rejects multi-sector subchannel reads, disabling subchannel\n");
			pcd.sub_ok = 0;
			with_sub = 0;
			r = 0;
		}
	}

	if (r && sync) {
		








		double dt = now_ms() - io0;
		if (dt > pcd.st_worst_io_ms) pcd.st_worst_io_ms = dt;
		return -1;
	}

	if (r) {
		for (int i = 0; i < count; i++) {
			uint8_t one[SLOT_SIZE];
			int rr = -1;
			

			for (int n = 0; n < 3 && rr; n++) {
				pthread_mutex_lock(&pcd.io);
				rr = sg_read_cd(lba + i, 1, flags, 0, one, BG_TIMEOUT_MS);
				pthread_mutex_unlock(&pcd.io);
			}
			if (rr) {
				pthread_mutex_lock(&pcd.io);
				rr = cooked_read_raw(lba + i, one);
				pthread_mutex_unlock(&pcd.io);
			}
			

			if (rr) pcd.consec_fail++; else pcd.consec_fail = 0;
			pthread_mutex_lock(&pcd.lock);
			slot_t *s = slot_for(lba + i);
			if (!rr) {
				memcpy(s->data, one, PHYSICAL_DISC_RAW);
				s->bad = 0;
			} else {
				




				memset(s->data, 0, PHYSICAL_DISC_RAW);
				s->bad = 1;
				pcd.st_bad++;
				


				if (pcd.st_bad_logged < 8) {
					pcd.st_bad_logged++;
					printf("DISC: unreadable sector lba=%d%s\n", lba + i,
						pcd.st_bad_logged == 8 ? " (further ones counted silently)" : "");
				}
			}
			memset(s->data + PHYSICAL_DISC_RAW, 0, PHYSICAL_DISC_SUB);
			s->has_sub = 0;
			s->lba = lba + i;
			pthread_mutex_unlock(&pcd.lock);
		}
		double d = now_ms() - io0;
		if (d > pcd.st_worst_io_ms) pcd.st_worst_io_ms = d;
		return 0;
	}

	double d = now_ms() - io0;
	if (d > pcd.st_worst_io_ms) pcd.st_worst_io_ms = d;

	pcd.consec_fail = 0;          

	int sector_len = PHYSICAL_DISC_RAW + (with_sub ? PHYSICAL_DISC_SUB : 0);
	pthread_mutex_lock(&pcd.lock);
	for (int i = 0; i < count; i++) {
		slot_t *s = slot_for(lba + i);
		memcpy(s->data, burst + i * sector_len, sector_len);
		if (!with_sub) memset(s->data + PHYSICAL_DISC_RAW, 0, PHYSICAL_DISC_SUB);
		s->has_sub = with_sub;
		s->lba = lba + i;
	}
	pthread_mutex_unlock(&pcd.lock);
	return 0;
}



static void stats_report()
{
	FILE *f = fopen("/tmp/physical_disc_stats.log", "w");
	if (f) {
		fprintf(f, "hit %u miss %u  hitrate %.1f%%  worst miss %.0f ms\n",
			pcd.st_hit, pcd.st_miss,
			(pcd.st_hit + pcd.st_miss) ? 100.0 * pcd.st_hit / (pcd.st_hit + pcd.st_miss) : 0.0,
			pcd.st_worst_ms);
		


		fprintf(f, "BAD %u sectors served as zeros  worst drive io %.0f ms\n",
			pcd.st_bad, pcd.st_worst_io_ms);
		

		fprintf(f, "REATTACH %u  (device %s)\n", pcd.st_reattach, cur_dev);
		fprintf(f, "data  window: active %d cursor %d\n", pcd.wactive[0], pcd.cursor[0]);
		fprintf(f, "cdda  window: active %d cursor %d\n", pcd.wactive[1], pcd.cursor[1]);
		fclose(f);
	}
	pcd.st_hit = pcd.st_miss = 0;
	pcd.st_worst_ms = 0.0;
	pcd.st_worst_io_ms = 0.0;
	

}

static int open_drive(char *out, int outsz);   













static int device_gone(void)
{
	if (pcd.fd < 0) return 1;
	if (ioctl(pcd.fd, CDROM_DRIVE_STATUS, CDSL_CURRENT) >= 0) return 0;
	return (errno == ENODEV || errno == ENXIO || errno == EIO || errno == ESHUTDOWN);
}

static void try_reattach(void)
{
	char newdev[64] = "";

	pthread_mutex_lock(&pcd.io);
	if (pcd.fd >= 0) { close(pcd.fd); pcd.fd = -1; }

	int fd = open_drive(newdev, sizeof(newdev));
	if (fd >= 0) {
		pcd.fd = fd;
		snprintf(cur_dev, 64, "%s", newdev);
		pcd.st_reattach++;
		pcd.consec_fail = 0;
		quiet_block_probes(cur_dev);   
		set_speed_cap();
		printf("DISC: drive re-attached as %s (recovered from a usb reset)\n", newdev);
	}
	else {
		printf("DISC: drive still missing, will retry\n");
	}
	pthread_mutex_unlock(&pcd.io);
}

static void *prefetch_thread(void *arg)
{
	(void)arg;
	int rr = 0;                                
	double last_stats = now_ms();
	double last_reattach = 0;
	double last_io = now_ms();

	double last_watch = 0;
	int was_present = -1;

	double last_swap_check = 0;
	int swap_was_present = -1;
	int swap_ejected = 0;

	while (pcd.running) {
		int target = -1;

		


		if (pcd.watch_mode) {
			if (now_ms() - last_watch >= 2000) {
				last_watch = now_ms();

				



				if (device_gone() && now_ms() - last_reattach > 5000) {
					last_reattach = now_ms();
					try_reattach();
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
						pthread_mutex_lock(&pcd.lock);
						snprintf(pcd.watch_label, sizeof(pcd.watch_label), "%s", lbl);
						pcd.watch_type = (int)t;
						pcd.watch_present = 1;
						


						pcd.watch_dirty = 1;
						pthread_mutex_unlock(&pcd.lock);

						pcd.ev_type = (int)t;
						pcd.ev_region = (int)r;
						pcd.ev_initial = (was_present < 0) ? 1 : 0;
						pcd.ev = (int)PHYSICAL_DISC_EV_DISC_IN;
						printf("DISC: disc detected: %s%s%s%s%s\n",
							physical_disc_disc_name(t),
							*physical_disc_region_name(r) ? " region " : "",
							physical_disc_region_name(r),
							lbl[0] ? " - " : "", lbl);
						was_present = 1;
					}
				}
				else if (!present && was_present == 1) {
					pthread_mutex_lock(&pcd.lock);
					if (pcd.watch_present) pcd.watch_dirty = 1;
					pcd.watch_present = 0;
					pcd.watch_label[0] = 0;
					pthread_mutex_unlock(&pcd.lock);

					pcd.ev = (int)PHYSICAL_DISC_EV_DISC_OUT;
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

		





		


		if (pcd.swap_enable && (pcd.leadout > 0 || swap_ejected)
		    && now_ms() - last_swap_check >= SWAP_CHECK_MS) {
			last_swap_check = now_ms();
			int present = physical_disc_disc_present();
			if (swap_was_present == 1 && !present) {
				swap_ejected = 1;                 
				pcd.swap_ejected = 1;             
			}
			else if (swap_ejected && present) {
				




				toc_t scratch;
				pcd.swapping = 1;
				pthread_mutex_lock(&pcd.io);
				physical_disc_forget_disc();
				int ok = !physical_disc_load_toc(&scratch);
				pthread_mutex_unlock(&pcd.io);
				




				if (ok) {
					










					double w0 = now_ms();
					int warm = 0;
					int wlba = scratch.tracks[0].start;
					while (now_ms() - w0 < 8000) {
						if (!fill_cache(wlba, SYNC_BURST, 1)) { warm = 1; break; }
					}
					if (warm) {
						



						for (int i = 1; i < 8; i++)
							if (fill_cache(wlba + i * SYNC_BURST, SYNC_BURST, 1)) break;
						if (pcd.trk[0].audio) { pcd.cursor[1] = wlba; pcd.wactive[1] = 1; }
						printf("DISC: disc swap - drive awake in %.0f ms\n", now_ms() - w0);
					}
					ok = warm;
				}
				pcd.swapping = 0;
				if (ok) {
					



					FILE *sf = fopen(PHYSICAL_DISC_SWAPPED_MARKER, "w");
					if (sf) fclose(sf);

					



					pcd.swap_ejected = 0;
					__sync_synchronize();
					pcd.swap_ready = 1;
					swap_ejected = 0;
					printf("DISC: disc swap - new toc loaded\n");
				}
				


			}
			swap_was_present = present;
		}

		





		if (pcd.prewarm >= 0) {
			if (pcd.last_win >= 0 || pcd.prewarm >= pcd.prewarm_end
			    || pcd.consec_fail >= 8 || pcd.leadout <= 0) {
				pcd.prewarm = -1;   
			} else if (!pcd.sync_pending) {
				int pw = pcd.prewarm;
				fill_cache(pw, BURST, 0);
				pcd.prewarm = pw + BURST;
				last_io = now_ms();
				continue;
			}
		}

		

		pthread_mutex_lock(&pcd.lock);
		for (int n = 0; n < NWIN && target < 0; n++) {
			int w = (rr + n) % NWIN;
			if (!pcd.wactive[w]) continue;
			int pos = pcd.cursor[w];
			for (int i = 0; i < READAHEAD; i++) {
				int lba = pos + i;
				if (lba >= pcd.leadout) break;
				if (win_of(lba) != w) break;   
				if (slot_for(lba)->lba != lba) { target = lba; break; }
			}
		}
		pthread_mutex_unlock(&pcd.lock);
		rr = (rr + 1) % NWIN;

		if (now_ms() - last_stats >= STATS_MS) {
			if (pcd.st_hit || pcd.st_miss) stats_report();
			last_stats = now_ms();
		}

		

		if (pcd.sync_pending) {
			struct timespec ts = { 0, 2 * 1000 * 1000 };
			nanosleep(&ts, NULL);
			continue;
		}

		


		if (pcd.consec_fail >= 8 && now_ms() - last_reattach > 5000) {
			last_reattach = now_ms();
			if (device_gone()) try_reattach();
			else pcd.consec_fail = 0;   
		}

		if (target < 0) {
			






			int lw = pcd.last_win;
			





			int klba = (lw >= 0) ? pcd.cursor[lw] : (pcd.leadout > 0 ? pcd.cursor[0] : -1);
			if (klba >= pcd.leadout) klba = pcd.leadout - 1;  
			if (pcd.leadout > 0 && klba >= 0
			    && now_ms() - last_io >= KEEPALIVE_MS && !pcd.sync_pending) {
				uint8_t sc[SLOT_SIZE];
				int t = track_of(klba);
				uint8_t fl = (t >= 0 && pcd.trk[t].audio) ? 0x10 : 0xF8;
				pthread_mutex_lock(&pcd.io);
				sg_read_cd(klba, 1, fl, 0, sc, BG_TIMEOUT_MS);
				pthread_mutex_unlock(&pcd.io);
				last_io = now_ms();
			}

			
			struct timespec ts = { 0, 20 * 1000 * 1000 };
			nanosleep(&ts, NULL);
			continue;
		}
		fill_cache(target, BURST, 0);
		last_io = now_ms();
	}
	return NULL;
}




void physical_disc_set_device(const char *dev)
{
	if (dev && *dev) snprintf(pref_dev, sizeof(pref_dev), "%s", dev);
	else pref_dev[0] = 0;
}







static int open_drive(char *out, int outsz)
{
	char path[64];
	int spare = -1;

	if (pref_dev[0]) {
		int fd = open(pref_dev, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
		if (fd >= 0) { snprintf(out, outsz, "%s", pref_dev); return fd; }
		printf("DISC: %s not available (%s), scanning\n", pref_dev, strerror(errno));
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

	if (pcd.fd >= 0) {
		
		if (!pref_dev[0] || !strcmp(pref_dev, cur_dev)) return 0;
		physical_disc_close();
	}

	pcd.fd = open_drive(cur_dev, sizeof(cur_dev));
	if (pcd.fd < 0) {
		cur_dev[0] = 0;
		printf("DISC: no cd-rom drive found (looked at /dev/sr0../dev/sr7)\n");
		return -1;
	}

	pcd.cache = (slot_t *)malloc(sizeof(slot_t) * CACHE_SECTORS);
	if (!pcd.cache) { close(pcd.fd); pcd.fd = -1; return -1; }
	for (int i = 0; i < CACHE_SECTORS; i++) pcd.cache[i].lba = -1;

	quiet_block_probes(cur_dev);
	set_speed_cap();

	pcd.leadout = 0;
	pcd.ntrk = 0;
	pcd.sub_ok = -1;
	for (int w = 0; w < NWIN; w++) { pcd.cursor[w] = 0; pcd.wactive[w] = 0; }
	pcd.last_win = -1;
	pcd.prewarm = -1;
	pcd.st_hit = pcd.st_miss = 0;
	pcd.st_worst_ms = 0.0;
	pcd.running = 1;
	pthread_create(&pcd.thread, NULL, prefetch_thread, NULL);

	

	cpu_set_t set;
	CPU_ZERO(&set);
	CPU_SET(0, &set);
	CPU_SET(1, &set);
	pthread_setaffinity_np(pcd.thread, sizeof(set), &set);

	printf("\x1b[32mDISC: opened %s\n\x1b[0m", cur_dev);
	return 0;
}

int physical_disc_disc_present()
{
	if (pcd.fd < 0) return 0;
	return ioctl(pcd.fd, CDROM_DRIVE_STATUS, CDSL_CURRENT) == CDS_DISC_OK;
}

int physical_disc_media_changed()
{
	if (pcd.fd < 0) return 0;
	return ioctl(pcd.fd, CDROM_MEDIA_CHANGED, CDSL_CURRENT) > 0;
}




int physical_disc_drive_busy()
{
	return pcd.fd >= 0;
}

int physical_disc_swap_ejected(void)
{
	


	return pcd.swap_enable && pcd.swap_ejected;
}






int physical_disc_swap_happened(void)
{
	return unlink(PHYSICAL_DISC_SWAPPED_MARKER) == 0;
}



void physical_disc_swap_enable(int enable)
{
	pcd.swap_enable = enable ? 1 : 0;
	if (enable) unlink(PHYSICAL_DISC_SWAPPED_MARKER);  
	if (!enable) pcd.swap_ejected = 0;
	if (!enable) pcd.swap_ready = 0;
}




int physical_disc_swap_consume(void)
{
	int r = pcd.swap_ready;
	pcd.swap_ready = 0;
	


	if (r) __sync_synchronize();
	return r;
}




static void probe_subchannel(int lba)
{
	uint8_t buf[SLOT_SIZE];
	int t = track_of(lba);
	uint8_t flags = (t >= 0 && pcd.trk[t].audio) ? 0x10 : 0xF8;

	if (!sg_read_cd(lba, 1, flags, 1, buf, BG_TIMEOUT_MS)) { pcd.sub_ok = 1; return; }
	if (!sg_read_cd(lba, 1, flags, 0, buf, BG_TIMEOUT_MS)) { pcd.sub_ok = 0; return; }
	pcd.sub_ok = -1;                  
}

int physical_disc_load_toc(toc_t *toc)
{
	struct cdrom_tochdr hdr;
	if (pcd.fd < 0 || !physical_disc_disc_present()) return -1;
	if (ioctl(pcd.fd, CDROMREADTOCHDR, &hdr) < 0) return -1;

	memset(toc, 0, sizeof(toc_t));
	pcd.leadout = 0;              
	pcd.first_data_lba = -1;
	pcd.ntrk = 0;

	int n = 0;
	for (int t = hdr.cdth_trk0; t <= hdr.cdth_trk1 && n < 99; t++, n++) {
		struct cdrom_tocentry e;
		memset(&e, 0, sizeof(e));
		e.cdte_track = t;
		e.cdte_format = CDROM_LBA;
		if (ioctl(pcd.fd, CDROMREADTOCENTRY, &e) < 0) return -1;

		cd_track_t *trk = &toc->tracks[n];
		trk->start = e.cdte_addr.lba;
		trk->type = (e.cdte_ctrl & CDROM_DATA_TRACK) ? TT_MODE1 : TT_CDDA;
		trk->sector_size = PHYSICAL_DISC_RAW;   
		trk->offset = 0;
		trk->index_num = 2;
		trk->indexes[0] = 0;
		trk->indexes[1] = 0;             

		pcd.trk[n].start = trk->start;
		pcd.trk[n].audio = (trk->type == TT_CDDA);

		if (trk->type != TT_CDDA && pcd.first_data_lba < 0)
			pcd.first_data_lba = trk->start;
		if (n > 0) {
			toc->tracks[n - 1].end = trk->start;
			pcd.trk[n - 1].end = trk->start;
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
	if (ioctl(pcd.fd, CDROMREADTOCENTRY, &lead) < 0) return -1;

	toc->tracks[n - 1].end = lead.cdte_addr.lba;
	toc->last = n;
	toc->end = lead.cdte_addr.lba;
	toc->sectorSize = PHYSICAL_DISC_RAW;
	toc->phys = 1;                            

	pcd.trk[n - 1].end = lead.cdte_addr.lba;
	pcd.ntrk = n;

	
	pthread_mutex_lock(&pcd.lock);
	for (int i = 0; i < CACHE_SECTORS; i++) pcd.cache[i].lba = -1;
	pthread_mutex_unlock(&pcd.lock);

	set_speed_cap();      

	pcd.sub_ok = -1;
	probe_subchannel(toc->tracks[0].start + 16);

	





	for (int w = 0; w < NWIN; w++) { pcd.cursor[w] = 0; pcd.wactive[w] = 0; }
	pcd.cursor[0] = toc->tracks[0].start;
	pcd.last_win = -1;
	pcd.st_hit = pcd.st_miss = pcd.st_bad = pcd.st_bad_logged = 0;
	pcd.st_worst_ms = pcd.st_worst_io_ms = 0.0;

	pcd.leadout = lead.cdte_addr.lba;         

	
















	pcd.prewarm = -1;
	if (!pcd.swapping && pcd.ntrk && pcd.trk[0].audio) {
		int pw_end = toc->tracks[0].start + PREWARM_SECTORS;
		if (pw_end > pcd.trk[0].end) pw_end = pcd.trk[0].end;  
		if (pw_end > pcd.leadout)    pw_end = pcd.leadout;
		




		pcd.cursor[1] = toc->tracks[0].start;
		pcd.wactive[1] = 1;
		pcd.prewarm_end = pw_end;
		__sync_synchronize();
		pcd.prewarm = toc->tracks[0].start;
	}

	printf("\x1b[32mDISC: toc loaded, %d tracks, leadout %d, subchannel %s\n\x1b[0m",
		n, pcd.leadout,
		pcd.sub_ok == 1 ? "yes" : pcd.sub_ok == 0 ? "no" : "unknown");
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

int physical_disc_current_toc(toc_t *toc)
{
	
	if (!toc || pcd.fd < 0 || pcd.swapping || pcd.ntrk < 1 || pcd.leadout <= 0) return -1;

	memset(toc, 0, sizeof(toc_t));
	for (int i = 0; i < pcd.ntrk; i++) {
		cd_track_t *trk = &toc->tracks[i];
		trk->start = pcd.trk[i].start;
		trk->end = pcd.trk[i].end;
		trk->type = pcd.trk[i].audio ? TT_CDDA : TT_MODE1;
		trk->sector_size = PHYSICAL_DISC_RAW;   
		trk->offset = 0;
		trk->index_num = 2;
		trk->indexes[0] = 0;
		trk->indexes[1] = 0;
	}
	toc->last = pcd.ntrk;
	toc->end = pcd.leadout;
	toc->sectorSize = PHYSICAL_DISC_RAW;
	toc->phys = 1;
	return 0;
}

void physical_disc_prewarm_blocking(void)
{
	if (pcd.fd < 0 || pcd.ntrk < 1 || pcd.leadout <= 0) return;
	int lba = pcd.trk[0].start;
	double start = now_ms();
	int ready = 0;
	while (now_ms() - start < 8000)
	{
		if (!fill_cache(lba, SYNC_BURST, 1))
		{
			ready = 1;
			break;
		}
	}
	if (!ready) return;
	int span = pcd.leadout - lba;
	if (span > 8 * SYNC_BURST)
	{
		static const int points[] = { 2, 3, 1 };
		for (unsigned i = 0; i < sizeof(points) / sizeof(points[0]); i++)
		{
			int target = lba + (int)(((int64_t)span * points[i]) / 4);
			if (target + SYNC_BURST > pcd.leadout) target = pcd.leadout - SYNC_BURST;
			fill_cache(target, SYNC_BURST, 1);
		}
	}
	for (int i = 1; i < 8; i++)
	{
		if (fill_cache(lba + i * SYNC_BURST, SYNC_BURST, 1)) break;
	}
	pcd.cursor[0] = lba;
	pcd.wactive[0] = 1;
}

void physical_disc_seek_hint(int lba)
{
	if (lba < 0 || !pcd.ntrk) return;

	
	int w = win_of(lba);
	pcd.cursor[w] = lba;
	pcd.wactive[w] = 1;
}



static int read_sector_impl(int lba, uint8_t *dst, uint8_t *sub96, int *sub_valid)
{
	if (sub_valid) *sub_valid = 0;

	if (pcd.fd < 0 || pcd.swapping || lba < 0 || lba >= pcd.leadout) {
		




		memset(dst, 0, PHYSICAL_DISC_RAW);
		if (sub96) memset(sub96, 0, PHYSICAL_DISC_SUB);
		return -1;
	}

	int w = win_of(lba);
	pcd.wactive[w] = 1;
	pcd.last_win = w;        

	pthread_mutex_lock(&pcd.lock);
	slot_t *s = slot_for(lba);
	int hit = (s->lba == lba);
	if (hit) {
		memcpy(dst, s->data, PHYSICAL_DISC_RAW);
		if (sub96) memcpy(sub96, s->data + PHYSICAL_DISC_RAW, PHYSICAL_DISC_SUB);
		if (sub_valid) *sub_valid = s->has_sub;
	}
	pthread_mutex_unlock(&pcd.lock);

	if (hit) {
		
		if (lba >= pcd.cursor[w]) pcd.cursor[w] = lba + 1;
		pcd.st_hit++;
		return 0;
	}

	
	double t0 = now_ms();
	pcd.sync_pending++;
	int fr = fill_cache(lba, SYNC_BURST, 1);
	pcd.sync_pending--;

	pcd.st_miss++;
	double d = now_ms() - t0;
	if (d > pcd.st_worst_ms) pcd.st_worst_ms = d;

	if (!fr) {
		pthread_mutex_lock(&pcd.lock);
		s = slot_for(lba);
		hit = (s->lba == lba);
		if (hit) {
			memcpy(dst, s->data, PHYSICAL_DISC_RAW);
			if (sub96) memcpy(sub96, s->data + PHYSICAL_DISC_RAW, PHYSICAL_DISC_SUB);
			if (sub_valid) *sub_valid = s->has_sub;
		}
		pthread_mutex_unlock(&pcd.lock);

		if (hit) {
			pcd.cursor[w] = lba + 1;
			return 0;
		}
	}

	



	memset(dst, 0, PHYSICAL_DISC_RAW);
	if (sub96) memset(sub96, 0, PHYSICAL_DISC_SUB);
	pcd.st_bad++;
	pcd.cursor[w] = lba;
	return 0;
}

int physical_disc_read_sector(int lba, uint8_t *dst, uint8_t *sub96)
{
	return read_sector_impl(lba, dst, sub96, NULL);
}

int physical_disc_read_sector_sub(int lba, uint8_t *dst, uint8_t *sub96)
{
	int valid = 0;
	if (read_sector_impl(lba, dst, sub96, &valid)) return 0;
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



physical_disc_disc_t physical_disc_identify()
{
	uint8_t raw[PHYSICAL_DISC_RAW * 2];

	if (!physical_disc_disc_present()) return PHYSICAL_DISC_DISC_NONE;
	if (pcd.first_data_lba < 0) {
		
		toc_t tmp;
		if (physical_disc_load_toc(&tmp)) return PHYSICAL_DISC_DISC_NONE;
		if (pcd.first_data_lba < 0) return PHYSICAL_DISC_DISC_AUDIO;
	}

	int base = pcd.first_data_lba;

	if (!physical_disc_read_sector(base, raw, NULL)) {
		if (!memcmp(raw + 16, "SEGADISCSYSTEM", 14)) return PHYSICAL_DISC_DISC_MEGACD;
		if (!memcmp(raw + 16, "SEGA SEGASATURN", 15)) return PHYSICAL_DISC_DISC_SATURN;
		

		if (raw[16] == 0x01 && raw[17] == 0x5A && raw[18] == 0x5A
			&& raw[19] == 0x5A && raw[20] == 0x5A && raw[21] == 0x5A)
			return PHYSICAL_DISC_DISC_3DO;
	}

	if (!physical_disc_read_sector(base + 16, raw, NULL)) {
		uint8_t *iso = raw + 16;
		if (memcmp(iso + 1, "CD001", 5)) iso = raw + 24;   
		if (!memcmp(iso + 1, "CD001", 5)) {
			if (!memcmp(iso + 8, "PLAYSTATION", 11)) return PHYSICAL_DISC_DISC_PSX;
			if (!memcmp(iso + 8, "NGCD", 4)) return PHYSICAL_DISC_DISC_NEOGEO;
		}
	}

	



	for (int s = 16; s <= 40; s++) {
		uint8_t user[2048];
		if (physical_disc_read_data2048(base + s, user)) continue;
		if (memmem(user, sizeof(user), "IPL.TXT", 7)) return PHYSICAL_DISC_DISC_NEOGEO;
	}

	if (!physical_disc_read_sector(base, raw, NULL) &&
	    !physical_disc_read_sector(base + 1, raw + PHYSICAL_DISC_RAW, NULL)) {
		for (int off = 0; off < (int)sizeof(raw) - 24; off++)
			if (!memcmp(raw + off, "PC Engine CD-ROM SYSTEM", 23))
				return PHYSICAL_DISC_DISC_PCECD;
	}

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
	if (pcd.first_data_lba < 0) {
		toc_t tmp;
		if (physical_disc_load_toc(&tmp)) return PHYSICAL_DISC_REGION_UNKNOWN;
		if (pcd.first_data_lba < 0) return PHYSICAL_DISC_REGION_UNKNOWN;
	}

	if (physical_disc_read_data2048(pcd.first_data_lba, user)) return PHYSICAL_DISC_REGION_UNKNOWN;
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
	if (pcd.first_data_lba < 0) {
		toc_t tmp;
		if (physical_disc_load_toc(&tmp)) return 0;
		if (pcd.first_data_lba < 0) return 0;   
	}

	
	if (physical_disc_read_data2048(pcd.first_data_lba + 16, user)) return 0;
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
	if (pcd.first_data_lba < 0) return 0;

	

	for (int s = 16; s <= 64; s++) {
		uint8_t user[2048];
		if (physical_disc_read_data2048(pcd.first_data_lba + s, user)) continue;

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


static int physical_disc_sanitize_name(const char *src, int len, char *out, int outsz)
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

static uint64_t physical_disc_hash64(uint64_t h, uint64_t v)
{
	for (int i = 0; i < 8; i++)
	{
		h ^= (uint8_t)(v >> (i * 8));
		h *= 1099511628211ULL;
	}
	return h;
}

static int physical_disc_toc_uuid(physical_disc_disc_t type, char *out, int outsz)
{
	if (!out || outsz < 37) return 0;
	toc_t toc;
	if (physical_disc_current_toc(&toc) || !toc.last)
	{
		if (physical_disc_load_toc(&toc) || !toc.last) return 0;
	}
	uint64_t h1 = 1469598103934665603ULL;
	uint64_t h2 = 1099511628211ULL;
	h1 = physical_disc_hash64(h1, (uint64_t)type);
	h2 = physical_disc_hash64(h2, (uint64_t)type ^ 0x9E3779B97F4A7C15ULL);
	h1 = physical_disc_hash64(h1, (uint64_t)toc.last);
	h2 = physical_disc_hash64(h2, (uint64_t)toc.end);
	for (int i = 0; i < toc.last; i++)
	{
		uint64_t v = ((uint64_t)(uint32_t)toc.tracks[i].start << 32) |
			((uint64_t)(uint16_t)toc.tracks[i].type << 16) |
			(uint16_t)(toc.tracks[i].end - toc.tracks[i].start);
		h1 = physical_disc_hash64(h1, v);
		h2 = physical_disc_hash64(h2, v ^ ((uint64_t)i << 56));
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
	if (pcd.first_data_lba < 0)
	{
		toc_t toc;
		if (physical_disc_load_toc(&toc) || pcd.first_data_lba < 0) return physical_disc_toc_uuid(type, out, outsz);
	}
	if (type == PHYSICAL_DISC_DISC_PSX)
	{
		if (physical_disc_disc_serial(id, sizeof(id)) && physical_disc_sanitize_name(id, strlen(id), out, outsz)) return strlen(out);
	}
	else if (type == PHYSICAL_DISC_DISC_SATURN && !physical_disc_read_data2048(pcd.first_data_lba, user))
	{
		if (!memcmp(user, "SEGA SEGASATURN", 15) && physical_disc_sanitize_name((char *)user + 0x20, 10, out, outsz)) return strlen(out);
	}
	else if (type == PHYSICAL_DISC_DISC_MEGACD && !physical_disc_read_data2048(pcd.first_data_lba, user))
	{
		if (!memcmp(user, "SEGADISCSYSTEM", 14) && physical_disc_sanitize_name((char *)user + 0x180, 14, out, outsz)) return strlen(out);
	}
	if (physical_disc_disc_label(id, sizeof(id)) && physical_disc_sanitize_name(id, strlen(id), out, outsz)) return strlen(out);
	return physical_disc_toc_uuid(type, out, outsz);
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
	case PHYSICAL_DISC_DISC_AUDIO:  return "Audio CD";
	case PHYSICAL_DISC_DISC_NONE:   return "No Disc";
	default:                 return "Unknown";
	}
}

int physical_disc_watch_start(void)
{
	

	physical_disc_acoustic_pause();
	if (physical_disc_open(NULL)) return -1;
	pcd.ev = (int)PHYSICAL_DISC_EV_NONE;
	pcd.watch_mode = 1;
	printf("DISC: watching %s for a disc\n", cur_dev);
	return 0;
}

void physical_disc_watch_stop(void)
{
	pcd.watch_mode = 0;
	pcd.ev = (int)PHYSICAL_DISC_EV_NONE;

	

	pthread_mutex_lock(&pcd.lock);
	pcd.watch_present = 0;
	pcd.watch_type = 0;
	pcd.watch_label[0] = 0;
	pcd.watch_dirty = 1;
	pthread_mutex_unlock(&pcd.lock);

	physical_disc_close();

	


	physical_disc_acoustic_resume();
}

int physical_disc_watching(void)
{
	return pcd.watch_mode;
}





int physical_disc_menu_status(char *name, int namesz, physical_disc_disc_t *type)
{
	pthread_mutex_lock(&pcd.lock);
	int present = pcd.watch_present;
	int t = pcd.watch_type;
	


	if (name && namesz > 0) snprintf(name, namesz, "%s", pcd.watch_label);
	pthread_mutex_unlock(&pcd.lock);

	if (type) *type = (physical_disc_disc_t)t;
	return present;
}




int physical_disc_menu_dirty(void)
{
	pthread_mutex_lock(&pcd.lock);
	int d = pcd.watch_dirty;
	pcd.watch_dirty = 0;
	pthread_mutex_unlock(&pcd.lock);
	return d;
}

physical_disc_event_t physical_disc_poll_event(physical_disc_disc_t *type, physical_disc_region_t *region, int *initial)
{
	physical_disc_event_t e = (physical_disc_event_t)pcd.ev;
	if (e == PHYSICAL_DISC_EV_NONE) return e;

	

	if (type) *type = (physical_disc_disc_t)pcd.ev_type;
	if (region) *region = (physical_disc_region_t)pcd.ev_region;
	if (initial) *initial = pcd.ev_initial;
	pcd.ev = (int)PHYSICAL_DISC_EV_NONE;
	return e;
}



void physical_disc_forget_disc(void)
{
	pcd.first_data_lba = -1;
	pcd.leadout = 0;
	pcd.ntrk = 0;
	if (pcd.cache) {
		pthread_mutex_lock(&pcd.lock);
		for (int i = 0; i < CACHE_SECTORS; i++) pcd.cache[i].lba = -1;
		pthread_mutex_unlock(&pcd.lock);
	}
}

void physical_disc_close()
{
	if (pcd.fd < 0) return;
	pcd.running = 0;
	pthread_join(pcd.thread, NULL);
	free(pcd.cache);
	pcd.cache = NULL;
	close(pcd.fd);
	pcd.fd = -1;
	pcd.leadout = 0;
	pcd.ntrk = 0;
	pcd.first_data_lba = -1;
	pcd.native_speed = 0;
	cur_dev[0] = 0;
}
