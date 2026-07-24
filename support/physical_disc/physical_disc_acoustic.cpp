#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <climits>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/cdrom.h>
#include <scsi/sg.h>

#include "physical_disc.h"
#include "physical_disc_acoustic.h"

#define ACTIVITY_WINDOW_MS       600
#define SPINDOWN_IDLE_MS         4000
#define REPOSITION_JUMP          90
#define IDLE_SEEK_PERIOD_MS      400
#define RATE_SAMPLE_MS           200
#define BURST_MIN                2
#define BURST_MAX                32
#define END_GUARD_SECTORS        32
#define READ_FAIL_LIMIT          6
#define REOPEN_FAIL_LIMIT        8
#define SEEK_FAIL_LIMIT          4
#define GAME_SPAN_MAX            360000
#define DRIVE_SPEED_MIN          2
#define DRIVE_SPEED_MAX          12
#define DRIVE_SPEED_STEP         2
#define SPEED_CHANGE_DEBOUNCE_MS 300
#define SEEK_CMD_TIMEOUT_MS      4000
#define READ_CMD_TIMEOUT_MS      4000

typedef struct {
	volatile int on;
	volatile int held;
	volatile int alive;
	volatile int disabled_perm;
	volatile int no_read;
	volatile int game_pos;
	volatile unsigned touch_count;
	int dev_fd;
	int disc_span;
	int span_lo, span_hi;
	pthread_t worker;
} decoy_state_t;

static decoy_state_t decoy = { 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0 };

static uint8_t burst_buf[BURST_MAX * 2048];

static double clock_ms(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

static int decoy_seek(int fd, int lba)
{
	uint8_t cdb[10] = { 0 };
	uint8_t sense[32];
	struct sg_io_hdr io;

	cdb[0] = 0x2B;
	cdb[2] = (lba >> 24) & 0xFF;
	cdb[3] = (lba >> 16) & 0xFF;
	cdb[4] = (lba >> 8) & 0xFF;
	cdb[5] = lba & 0xFF;

	memset(&io, 0, sizeof(io));
	io.interface_id = 'S';
	io.cmd_len = 10;
	io.cmdp = cdb;
	io.dxfer_direction = SG_DXFER_NONE;
	io.sbp = sense;
	io.mx_sb_len = sizeof(sense);
	io.timeout = SEEK_CMD_TIMEOUT_MS;

	if (ioctl(fd, SG_IO, &io) < 0) return -1;
	if (io.status || io.host_status || io.driver_status) return -2;
	return 0;
}

static int decoy_read(int fd, int lba, int blocks)
{
	uint8_t cdb[10] = { 0 };
	uint8_t sense[32];
	struct sg_io_hdr io;

	cdb[0] = 0x28;
	cdb[2] = (lba >> 24) & 0xFF;
	cdb[3] = (lba >> 16) & 0xFF;
	cdb[4] = (lba >> 8) & 0xFF;
	cdb[5] = lba & 0xFF;
	cdb[7] = (blocks >> 8) & 0xFF;
	cdb[8] = blocks & 0xFF;

	memset(&io, 0, sizeof(io));
	io.interface_id = 'S';
	io.cmd_len = 10;
	io.cmdp = cdb;
	io.dxfer_direction = SG_DXFER_FROM_DEV;
	io.dxfer_len = blocks * 2048;
	io.dxferp = burst_buf;
	io.sbp = sense;
	io.mx_sb_len = sizeof(sense);
	io.timeout = READ_CMD_TIMEOUT_MS;

	if (ioctl(fd, SG_IO, &io) < 0) return -1;
	if (io.status || io.host_status || io.driver_status) return -2;
	return 0;
}

static void decoy_spin(int fd, int start)
{
	uint8_t cdb[6] = { 0 };
	uint8_t sense[32];
	struct sg_io_hdr io;

	cdb[0] = 0x1B;
	cdb[1] = 0x01;
	cdb[4] = start ? 0x01 : 0x00;

	memset(&io, 0, sizeof(io));
	io.interface_id = 'S';
	io.cmd_len = 6;
	io.cmdp = cdb;
	io.dxfer_direction = SG_DXFER_NONE;
	io.sbp = sense;
	io.mx_sb_len = sizeof(sense);
	io.timeout = SEEK_CMD_TIMEOUT_MS;

	ioctl(fd, SG_IO, &io);
}

static void decoy_set_speed(int fd, int nx)
{
	ioctl(fd, CDROM_SELECT_SPEED, nx);
}

static int project_lba(int game_lba)
{
	int lo = decoy.span_lo, hi = decoy.span_hi;
	if (hi <= lo) return lo;
	if (game_lba < 0) game_lba = 0;
	if (game_lba > GAME_SPAN_MAX) game_lba = GAME_SPAN_MAX;
	int p = lo + (int)((int64_t)game_lba * (hi - lo) / GAME_SPAN_MAX);
	if (p < lo) p = lo;
	if (p > hi) p = hi;
	return p;
}

static int decoy_acquire(void)
{
	if (physical_disc_drive_busy()) return -1;

	for (int i = 0; i < 8; i++) {
		char path[32];
		snprintf(path, sizeof(path), "/dev/sr%d", i);
		int fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
		if (fd < 0) continue;

		if (ioctl(fd, CDROM_DRIVE_STATUS, CDSL_CURRENT) != CDS_DISC_OK) { close(fd); continue; }

		struct cdrom_tochdr hdr;
		struct cdrom_tocentry e;
		if (ioctl(fd, CDROMREADTOCHDR, &hdr) < 0) { close(fd); continue; }
		memset(&e, 0, sizeof(e));
		e.cdte_track = CDROM_LEADOUT;
		e.cdte_format = CDROM_LBA;
		if (ioctl(fd, CDROMREADTOCENTRY, &e) < 0) { close(fd); continue; }

		decoy.disc_span = e.cdte_addr.lba;
		decoy.span_lo = 0;
		decoy.span_hi = decoy.disc_span - BURST_MAX - END_GUARD_SECTORS;
		if (decoy.span_hi < decoy.span_lo) decoy.span_hi = decoy.span_lo;
		decoy.no_read = 0;
		decoy.dev_fd = fd;

		static int last_logged_span = -1;
		if (decoy.disc_span != last_logged_span) {
			printf("physical_disc_acoustic: prop disc on %s, %d sectors\n", path, decoy.disc_span);
			last_logged_span = decoy.disc_span;
		}
		return 0;
	}
	return -1;
}

static void decoy_release(void)
{
	if (decoy.dev_fd >= 0) { close(decoy.dev_fd); decoy.dev_fd = -1; }
	decoy.disc_span = 0;
}

static void *decoy_worker_main(void *arg)
{
	(void)arg;
	unsigned seq_seen = 0, seq_base = 0;
	double active_at = 0, open_attempt_at = 0, io_at = 0;
	double rate_at = 0, rate = 0;
	int prior_lba = -1000000;
	int seek_faults = 0, read_faults = 0, reopen_faults = 0;
	int spun_down = 0;
	int applied_speed = 0;
	double speed_changed_at = 0;

	while (decoy.alive) {
		if (!decoy.on || decoy.held || decoy.disabled_perm) {
			decoy_release();
			struct timespec ts = { 0, 150 * 1000 * 1000 };
			nanosleep(&ts, NULL);
			continue;
		}

		unsigned seq = decoy.touch_count;
		double now = clock_ms();
		if (seq != seq_seen) { seq_seen = seq; active_at = now; spun_down = 0; }

		if (now - active_at >= ACTIVITY_WINDOW_MS) {
			if (!spun_down && decoy.dev_fd >= 0 && !decoy.no_read
			    && now - active_at >= SPINDOWN_IDLE_MS) {
				decoy_spin(decoy.dev_fd, 0);
				spun_down = 1;
				applied_speed = 0;
			}
			struct timespec ts = { 0, 80 * 1000 * 1000 };
			nanosleep(&ts, NULL);
			continue;
		}

		if (physical_disc_drive_busy()) {
			decoy_release();
			struct timespec ts = { 0, 200 * 1000 * 1000 };
			nanosleep(&ts, NULL);
			continue;
		}

		if (decoy.dev_fd < 0) {
			if (now - open_attempt_at < 1000) {
				struct timespec ts = { 0, 100 * 1000 * 1000 };
				nanosleep(&ts, NULL);
				continue;
			}
			open_attempt_at = now;
			if (decoy_acquire()) {
				struct timespec ts = { 0, 100 * 1000 * 1000 };
				nanosleep(&ts, NULL);
				continue;
			}
			prior_lba = -1000000;
			applied_speed = 0;
		}

		if (now - rate_at >= RATE_SAMPLE_MS) {
			unsigned d = seq - seq_base;
			double dt = now - rate_at;
			double inst = dt > 0 ? d * 1000.0 / dt : 0;
			rate += (inst - rate) * 0.3;
			seq_base = seq;
			rate_at = now;
		}
		int burst = 2 + (int)((rate - 8) / 6);
		if (burst < BURST_MIN) burst = BURST_MIN;
		if (burst > BURST_MAX) burst = BURST_MAX;
		double gap = 120.0 - rate;
		if (gap < 10.0) gap = 10.0;
		if (gap > 120.0) gap = 120.0;

		if (!decoy.no_read) {
			double target = DRIVE_SPEED_MIN + rate / 12.0;
			if (target > DRIVE_SPEED_MAX) target = DRIVE_SPEED_MAX;
			int want = applied_speed;
			if (applied_speed < DRIVE_SPEED_MIN) want = DRIVE_SPEED_MIN;
			else if (target >= applied_speed + DRIVE_SPEED_STEP) want = applied_speed + DRIVE_SPEED_STEP;
			else if (target <= applied_speed - DRIVE_SPEED_STEP) want = applied_speed - DRIVE_SPEED_STEP;
			if (want > DRIVE_SPEED_MAX) want = DRIVE_SPEED_MAX;
			if (want < DRIVE_SPEED_MIN) want = DRIVE_SPEED_MIN;
			if (want != applied_speed && now - speed_changed_at >= SPEED_CHANGE_DEBOUNCE_MS) {
				decoy_set_speed(decoy.dev_fd, want);
				applied_speed = want;
				speed_changed_at = now;
			}
		}

		int lba = project_lba(decoy.game_pos);
		int jump = lba > prior_lba ? lba - prior_lba : prior_lba - lba;

		if (!decoy.no_read) {
			if (jump >= REPOSITION_JUMP || now - io_at >= gap) {
				int r = decoy_read(decoy.dev_fd, lba, burst);
				if (r == 0) {
					read_faults = 0; seek_faults = 0; reopen_faults = 0;
					io_at = now; prior_lba = lba;
				} else if (r == -1) {
					decoy_release(); read_faults = 0;
					if (++reopen_faults >= REOPEN_FAIL_LIMIT) {
						printf("physical_disc_acoustic: drive keeps dropping, disabling for this session\n");
						decoy.disabled_perm = 1;
					}
				} else {
					if (++read_faults >= READ_FAIL_LIMIT) {
						printf("physical_disc_acoustic: drive rejects READ(10), seek-only fallback\n");
						decoy.no_read = 1;
					} else {
						decoy_seek(decoy.dev_fd, lba);
					}
					io_at = now; prior_lba = lba;
				}
			}
		} else if (!decoy.disabled_perm) {
			if (jump >= REPOSITION_JUMP || now - io_at >= IDLE_SEEK_PERIOD_MS) {
				int r = decoy_seek(decoy.dev_fd, lba);
				if (r == -1) {
					decoy_release(); seek_faults = 0;
					if (++reopen_faults >= REOPEN_FAIL_LIMIT) {
						printf("physical_disc_acoustic: drive keeps dropping, disabling for this session\n");
						decoy.disabled_perm = 1;
					}
				} else if (r < 0) {
					if (++seek_faults >= SEEK_FAIL_LIMIT) {
						printf("physical_disc_acoustic: drive does not accept SEEK, disabling for this session\n");
						decoy.disabled_perm = 1;
						decoy_release();
					}
				} else {
					seek_faults = 0; reopen_faults = 0;
					io_at = now; prior_lba = lba;
				}
			}
		}

		struct timespec ts = { 0, 20 * 1000 * 1000 };
		nanosleep(&ts, NULL);
	}

	decoy_release();
	return NULL;
}

void physical_disc_acoustic_config(int enabled)
{
	decoy.on = enabled ? 1 : 0;
	if (decoy.on && !decoy.alive) {
		decoy.alive = 1;
		decoy.held = 0;
		if (pthread_create(&decoy.worker, NULL, decoy_worker_main, NULL)) {
			decoy.alive = 0;
			printf("physical_disc_acoustic: could not start thread\n");
			return;
		}
		printf("physical_disc_acoustic: enabled - put a spare data disc in the drive\n");
	}
}

void physical_disc_acoustic_hint(int lba)
{
	if (!decoy.on || decoy.held) return;
	decoy.game_pos = lba;
	decoy.touch_count++;
}

void physical_disc_acoustic_pause(void)
{
	decoy.held = 1;
}

void physical_disc_acoustic_resume(void)
{
	decoy.held = 0;
}
