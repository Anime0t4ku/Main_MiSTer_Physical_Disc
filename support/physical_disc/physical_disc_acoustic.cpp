





























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

#define ACTIVE_MS       600       
#define DEEP_IDLE_MS    4000      
#define JUMP_THRESHOLD  90        
#define KEEPALIVE_MS    400       
#define SAMPLE_MS       200       
#define MIN_BURST       2         
#define MAX_BURST       32        
#define READ_GUARD      32        
#define READ_GIVEUP     6         
#define REOPEN_GIVEUP   8         
#define GAME_LBA_MAX    360000    
#define SPEED_MIN       2         
#define SPEED_MAX       12        
#define SPEED_STEP      2         
#define SPEED_DEBOUNCE_MS 300     
#define SEEK_TIMEOUT_MS 4000
#define READ_TIMEOUT_MS 4000

static struct {
	volatile int enabled;
	volatile int paused;
	volatile int running;
	volatile int unsupported;      
	volatile int read_unsupported; 
	volatile int target_lba;
	volatile unsigned seq;         
	int fd;                        
	int prop_max;                  
	int read_lo, read_hi;          
	pthread_t thread;
} ac = { 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0 };



static uint8_t rbuf[MAX_BURST * 2048];

static double now_ms(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}



static int sg_seek(int fd, int lba)
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
	io.timeout = SEEK_TIMEOUT_MS;

	if (ioctl(fd, SG_IO, &io) < 0) return -1;
	if (io.status || io.host_status || io.driver_status) return -2;
	return 0;
}





static int sg_read10(int fd, int lba, int blocks)
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
	io.dxferp = rbuf;
	io.sbp = sense;
	io.mx_sb_len = sizeof(sense);
	io.timeout = READ_TIMEOUT_MS;

	if (ioctl(fd, SG_IO, &io) < 0) return -1;
	if (io.status || io.host_status || io.driver_status) return -2;
	return 0;
}



static void sg_start_stop(int fd, int start)
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
	io.timeout = SEEK_TIMEOUT_MS;

	ioctl(fd, SG_IO, &io);
}





static void set_speed(int fd, int nx)
{
	ioctl(fd, CDROM_SELECT_SPEED, nx);
}




static int map_lba(int game_lba)
{
	int lo = ac.read_lo, hi = ac.read_hi;
	if (hi <= lo) return lo;
	if (game_lba < 0) game_lba = 0;
	if (game_lba > GAME_LBA_MAX) game_lba = GAME_LBA_MAX;
	int p = lo + (int)((int64_t)game_lba * (hi - lo) / GAME_LBA_MAX);
	if (p < lo) p = lo;
	if (p > hi) p = hi;
	return p;
}




static int open_prop(void)
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

		ac.prop_max = e.cdte_addr.lba;
		
		ac.read_lo = 0;
		ac.read_hi = ac.prop_max - MAX_BURST - READ_GUARD;
		if (ac.read_hi < ac.read_lo) ac.read_hi = ac.read_lo;
		ac.read_unsupported = 0;   
		ac.fd = fd;
		
		static int last_logged_max = -1;
		if (ac.prop_max != last_logged_max) {
			printf("physical_disc_acoustic: prop disc on %s, %d sectors\n", path, ac.prop_max);
			last_logged_max = ac.prop_max;
		}
		return 0;
	}
	return -1;
}

static void close_prop(void)
{
	if (ac.fd >= 0) { close(ac.fd); ac.fd = -1; }
	ac.prop_max = 0;
}

static void *acoustic_thread(void *arg)
{
	(void)arg;
	unsigned last_seq = 0, seq_prev = 0;
	double last_active = 0, last_open_try = 0, last_read = 0;
	double t_sample = 0, rate = 0;
	int last_pos = -1000000;         
	int seek_fails = 0, read_fails = 0, reopen_fails = 0;
	int idle_stopped = 0;
	int cur_speed = 0;               
	double last_speed_ms = 0;

	while (ac.running) {
		
		
		if (!ac.enabled || ac.paused || ac.unsupported) {
			close_prop();
			struct timespec ts = { 0, 150 * 1000 * 1000 };
			nanosleep(&ts, NULL);
			continue;
		}

		unsigned seq = ac.seq;
		double now = now_ms();
		if (seq != last_seq) { last_seq = seq; last_active = now; idle_stopped = 0; }

		
		
		if (now - last_active >= ACTIVE_MS) {
			if (!idle_stopped && ac.fd >= 0 && !ac.read_unsupported
			    && now - last_active >= DEEP_IDLE_MS) {
				sg_start_stop(ac.fd, 0);
				idle_stopped = 1;
				cur_speed = 0;   
			}
			struct timespec ts = { 0, 80 * 1000 * 1000 };
			nanosleep(&ts, NULL);
			continue;
		}

		
		if (physical_disc_drive_busy()) {
			close_prop();
			struct timespec ts = { 0, 200 * 1000 * 1000 };
			nanosleep(&ts, NULL);
			continue;
		}

		if (ac.fd < 0) {
			
			if (now - last_open_try < 1000) {
				struct timespec ts = { 0, 100 * 1000 * 1000 };
				nanosleep(&ts, NULL);
				continue;
			}
			last_open_try = now;
			if (open_prop()) {
				struct timespec ts = { 0, 100 * 1000 * 1000 };
				nanosleep(&ts, NULL);
				continue;
			}
			last_pos = -1000000;   
			cur_speed = 0;         
		}

		
		
		
		
		if (now - t_sample >= SAMPLE_MS) {
			unsigned d = seq - seq_prev;
			double dt = now - t_sample;
			double inst = dt > 0 ? d * 1000.0 / dt : 0;
			rate += (inst - rate) * 0.3;   
			seq_prev = seq;
			t_sample = now;
		}
		int burst = 2 + (int)((rate - 8) / 6);
		if (burst < MIN_BURST) burst = MIN_BURST;
		if (burst > MAX_BURST) burst = MAX_BURST;
		double gap = 120.0 - rate;
		if (gap < 10.0) gap = 10.0;
		if (gap > 120.0) gap = 120.0;

		
		
		
		
		
		if (!ac.read_unsupported) {
			double target = SPEED_MIN + rate / 12.0;
			if (target > SPEED_MAX) target = SPEED_MAX;
			int want = cur_speed;
			if (cur_speed < SPEED_MIN) want = SPEED_MIN;                
			else if (target >= cur_speed + SPEED_STEP) want = cur_speed + SPEED_STEP;
			else if (target <= cur_speed - SPEED_STEP) want = cur_speed - SPEED_STEP;
			if (want > SPEED_MAX) want = SPEED_MAX;
			if (want < SPEED_MIN) want = SPEED_MIN;
			if (want != cur_speed && now - last_speed_ms >= SPEED_DEBOUNCE_MS) {
				set_speed(ac.fd, want);
				cur_speed = want;
				last_speed_ms = now;
			}
		}

		int lba = map_lba(ac.target_lba);
		int jump = lba > last_pos ? lba - last_pos : last_pos - lba;

		if (!ac.read_unsupported) {
			
			if (jump >= JUMP_THRESHOLD || now - last_read >= gap) {
				int r = sg_read10(ac.fd, lba, burst);
				if (r == 0) {
					read_fails = 0; seek_fails = 0; reopen_fails = 0;
					last_read = now; last_pos = lba;
				} else if (r == -1) {
					
					
					close_prop(); read_fails = 0;
					if (++reopen_fails >= REOPEN_GIVEUP) {
						printf("physical_disc_acoustic: drive keeps dropping, disabling for this session\n");
						ac.unsupported = 1;
					}
				} else {
					
					
					
					if (++read_fails >= READ_GIVEUP) {
						printf("physical_disc_acoustic: drive rejects READ(10), seek-only fallback\n");
						ac.read_unsupported = 1;
					} else {
						sg_seek(ac.fd, lba);
					}
					last_read = now; last_pos = lba;
				}
			}
		} else if (!ac.unsupported) {
			
			if (jump >= JUMP_THRESHOLD || now - last_read >= KEEPALIVE_MS) {
				int r = sg_seek(ac.fd, lba);
				if (r == -1) {
					close_prop(); seek_fails = 0;
					if (++reopen_fails >= REOPEN_GIVEUP) {
						printf("physical_disc_acoustic: drive keeps dropping, disabling for this session\n");
						ac.unsupported = 1;
					}
				} else if (r < 0) {
					if (++seek_fails >= 4) {
						printf("physical_disc_acoustic: drive does not accept SEEK, disabling for this session\n");
						ac.unsupported = 1;
						close_prop();
					}
				} else {
					seek_fails = 0; reopen_fails = 0;
					last_read = now; last_pos = lba;
				}
			}
		}

		struct timespec ts = { 0, 20 * 1000 * 1000 };
		nanosleep(&ts, NULL);
	}

	close_prop();
	return NULL;
}

void physical_disc_acoustic_config(int enabled)
{
	ac.enabled = enabled ? 1 : 0;
	if (ac.enabled && !ac.running) {
		ac.running = 1;
		ac.paused = 0;
		if (pthread_create(&ac.thread, NULL, acoustic_thread, NULL)) {
			ac.running = 0;
			printf("physical_disc_acoustic: could not start thread\n");
			return;
		}
		printf("physical_disc_acoustic: enabled - put a spare data disc in the drive\n");
	}
}






void physical_disc_acoustic_hint(int lba)
{
	if (!ac.enabled || ac.paused) return;
	ac.target_lba = lba;
	ac.seq++;
}

void physical_disc_acoustic_pause(void)
{
	ac.paused = 1;
}

void physical_disc_acoustic_resume(void)
{
	ac.paused = 0;
}
