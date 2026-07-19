
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>
#include <errno.h>
#include <climits>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/cdrom.h>
#include <pthread.h>
#include <sched.h>

#include "megacd.h"
#include "../../hardware.h"
#include "../chd/mister_chd.h"

cdd_t cdd;

#define MCD_PHYSICAL_DEVICE "/dev/sr0"
#define MCD_PHYSICAL_SECTOR_SIZE 2352
#define MCD_PHYSICAL_SUBCODE_SIZE 96
#define MCD_PHYSICAL_FRAME_SIZE MCD_PHYSICAL_SECTOR_SIZE
#define MCD_PHYSICAL_CACHE_SLOTS 2
#define MCD_PHYSICAL_CACHE_SECTORS 16
#define MCD_PHYSICAL_PACKET_SECTORS 32

struct mcd_physical_cache_t {
	int start_lba;
	int valid;
	uint8_t data[MCD_PHYSICAL_CACHE_SECTORS * MCD_PHYSICAL_FRAME_SIZE];
};

static int mcd_physical_fd = -1;
static toc_t *mcd_physical_toc = NULL;
static mcd_physical_cache_t mcd_physical_cache[MCD_PHYSICAL_CACHE_SLOTS] = {};
static pthread_mutex_t mcd_physical_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t mcd_physical_cond = PTHREAD_COND_INITIALIZER;
static pthread_t mcd_physical_thread;
static int mcd_physical_thread_started = 0;
static int mcd_physical_busy = 0;
static int mcd_physical_request_slot = 0;
static int mcd_physical_request_lba = 0;
static pthread_mutex_t mcd_stats_mutex = PTHREAD_MUTEX_INITIALIZER;
static uint64_t mcd_stats_packet_calls = 0;
static uint64_t mcd_stats_packet_sectors = 0;
static uint64_t mcd_stats_packet_ms = 0;
static uint32_t mcd_stats_packet_max_ms = 0;
static uint64_t mcd_stats_packet_failures = 0;
static uint64_t mcd_stats_cache_hits = 0;
static uint64_t mcd_stats_cache_misses = 0;
static uint64_t mcd_stats_cache_wait_ms = 0;
static uint32_t mcd_stats_cache_wait_max_ms = 0;
static uint64_t mcd_stats_fpga_wait_data = 0;
static uint64_t mcd_stats_fpga_wait_audio = 0;
static uint32_t mcd_stats_timer = 0;

static uint32_t mcd_stats_now_ms()
{
	struct timespec tp;
	clock_gettime(CLOCK_MONOTONIC, &tp);
	return (uint32_t)(tp.tv_sec * 1000ULL + tp.tv_nsec / 1000000);
}

void mcd_physical_stats_reset()
{
	pthread_mutex_lock(&mcd_stats_mutex);
	mcd_stats_packet_calls = mcd_stats_packet_sectors = mcd_stats_packet_ms = 0;
	mcd_stats_packet_max_ms = 0;
	mcd_stats_packet_failures = 0;
	mcd_stats_cache_hits = mcd_stats_cache_misses = mcd_stats_cache_wait_ms = 0;
	mcd_stats_cache_wait_max_ms = 0;
	mcd_stats_fpga_wait_data = mcd_stats_fpga_wait_audio = 0;
	mcd_stats_timer = GetTimer(5000);
	pthread_mutex_unlock(&mcd_stats_mutex);
	FILE *f = fopen("/tmp/megacd_physical_stats.log", "w");
	if (f) {
		fprintf(f, "Mega CD physical-disc telemetry started (v0.11 behavior)\n");
		fclose(f);
	}
}

void mcd_physical_note_fpga_wait(uint8_t type)
{
	pthread_mutex_lock(&mcd_stats_mutex);
	if (type) mcd_stats_fpga_wait_data++;
	else mcd_stats_fpga_wait_audio++;
	pthread_mutex_unlock(&mcd_stats_mutex);
}

void mcd_physical_stats_poll()
{
	if (!mcd_stats_timer || !CheckTimer(mcd_stats_timer)) return;
	mcd_stats_timer = GetTimer(5000);
	pthread_mutex_lock(&mcd_stats_mutex);
	uint64_t calls = mcd_stats_packet_calls;
	uint64_t sectors = mcd_stats_packet_sectors;
	uint64_t packet_ms = mcd_stats_packet_ms;
	uint32_t packet_max = mcd_stats_packet_max_ms;
	uint64_t failures = mcd_stats_packet_failures;
	uint64_t hits = mcd_stats_cache_hits;
	uint64_t misses = mcd_stats_cache_misses;
	uint64_t wait_ms = mcd_stats_cache_wait_ms;
	uint32_t wait_max = mcd_stats_cache_wait_max_ms;
	uint64_t fpga_data = mcd_stats_fpga_wait_data;
	uint64_t fpga_audio = mcd_stats_fpga_wait_audio;
	pthread_mutex_unlock(&mcd_stats_mutex);
	FILE *f = fopen("/tmp/megacd_physical_stats.log", "a");
	if (f) {
		fprintf(f, "t=%lu packets=%llu sectors=%llu packet_avg=%.1fms packet_max=%ums failures=%llu cache_hit=%llu cache_miss=%llu wait_avg=%.1fms wait_max=%ums fpga_wait_data=%llu fpga_wait_audio=%llu\n",
			GetTimer(0) / 1000, (unsigned long long)calls, (unsigned long long)sectors,
			calls ? (double)packet_ms / calls : 0.0, packet_max, (unsigned long long)failures,
			(unsigned long long)hits, (unsigned long long)misses,
			misses ? (double)wait_ms / misses : 0.0, wait_max,
			(unsigned long long)fpga_data, (unsigned long long)fpga_audio);
		fclose(f);
	}
}

static int mcd_physical_open()
{
	if (mcd_physical_fd >= 0) return 1;
	mcd_physical_fd = open(MCD_PHYSICAL_DEVICE, O_RDONLY | O_NONBLOCK);
	if (mcd_physical_fd < 0) {
		printf("MCD-CD: cannot open %s: %s\n", MCD_PHYSICAL_DEVICE, strerror(errno));
		return 0;
	}
	return 1;
}

static void mcd_physical_set_max_speed()
{
	if (mcd_physical_fd < 0) return;
	if (ioctl(mcd_physical_fd, CDROM_SELECT_SPEED, 0) < 0)
		printf("MCD-CD: CDROM_SELECT_SPEED maximum request failed: %s\n", strerror(errno));

	struct cdrom_generic_command cgc = {};
	struct request_sense sense = {};
	cgc.cmd[0] = GPCMD_SET_SPEED;
	cgc.cmd[2] = 0xff;
	cgc.cmd[3] = 0xff;
	cgc.cmd[4] = 0xff;
	cgc.cmd[5] = 0xff;
	cgc.sense = &sense;
	cgc.data_direction = CGC_DATA_NONE;
	cgc.quiet = 1;
	cgc.timeout = 3000;
	if (ioctl(mcd_physical_fd, CDROM_SEND_PACKET, &cgc) < 0)
		printf("MCD-CD: MMC maximum speed request not supported: %s\n", strerror(errno));
	else
		printf("MCD-CD: maximum MMC read speed requested\n");
}

int cdd_t::PhysicalDiscPresent()
{
	return mcd_physical_open() && ioctl(mcd_physical_fd, CDROM_DRIVE_STATUS, CDSL_CURRENT) == CDS_DISC_OK;
}

static int mcd_physical_track_for_lba(int lba)
{
	if (!mcd_physical_toc) return -1;
	for (int i = 0; i < mcd_physical_toc->last; i++) {
		if (lba >= (int)mcd_physical_toc->tracks[i].start && lba < (int)mcd_physical_toc->tracks[i].end) return i;
	}
	return -1;
}

static int mcd_physical_read_packet(uint8_t *buffer, int lba, int count, int track)
{
	struct cdrom_generic_command cgc = {};
	struct request_sense sense = {};
	if (mcd_physical_fd < 0 || count < 1 || track < 0) return 0;
	cgc.cmd[0] = GPCMD_READ_CD;
	cgc.cmd[2] = (lba >> 24) & 0xff;
	cgc.cmd[3] = (lba >> 16) & 0xff;
	cgc.cmd[4] = (lba >> 8) & 0xff;
	cgc.cmd[5] = lba & 0xff;
	cgc.cmd[6] = (count >> 16) & 0xff;
	cgc.cmd[7] = (count >> 8) & 0xff;
	cgc.cmd[8] = count & 0xff;
	cgc.cmd[9] = mcd_physical_toc->tracks[track].type == TT_CDDA ? 0x10 : 0xf8;
	// Diagnostic isolation: request sector data only, without raw P-W data.
	cgc.cmd[10] = 0x00;
	cgc.buffer = buffer;
	cgc.buflen = count * MCD_PHYSICAL_FRAME_SIZE;
	cgc.sense = &sense;
	cgc.data_direction = CGC_DATA_READ;
	cgc.quiet = 1;
	cgc.timeout = 3000;
	uint32_t started = mcd_stats_now_ms();
	int ok = ioctl(mcd_physical_fd, CDROM_SEND_PACKET, &cgc) >= 0;
	uint32_t elapsed = mcd_stats_now_ms() - started;
	pthread_mutex_lock(&mcd_stats_mutex);
	mcd_stats_packet_calls++;
	mcd_stats_packet_sectors += count;
	mcd_stats_packet_ms += elapsed;
	if (elapsed > mcd_stats_packet_max_ms) mcd_stats_packet_max_ms = elapsed;
	if (!ok) mcd_stats_packet_failures++;
	pthread_mutex_unlock(&mcd_stats_mutex);
	return ok;
}

static void mcd_physical_read_uncached(uint8_t *buffer, int lba, int count)
{
	while (count > 0) {
		int track = mcd_physical_track_for_lba(lba);
		if (track < 0) {
			memset(buffer, 0, MCD_PHYSICAL_FRAME_SIZE);
			buffer += MCD_PHYSICAL_FRAME_SIZE;
			lba++;
			count--;
			continue;
		}
		int chunk = count;
		int remaining = mcd_physical_toc->tracks[track].end - lba;
		if (chunk > remaining) chunk = remaining;
		if (chunk > MCD_PHYSICAL_PACKET_SECTORS) chunk = MCD_PHYSICAL_PACKET_SECTORS;
		if (!mcd_physical_read_packet(buffer, lba, chunk, track)) {
			printf("MCD-CD: READ CD failed at LBA %d (%d sectors): %s\n", lba, chunk, strerror(errno));
			memset(buffer, 0, chunk * MCD_PHYSICAL_FRAME_SIZE);
		}
		buffer += chunk * MCD_PHYSICAL_FRAME_SIZE;
		lba += chunk;
		count -= chunk;
	}
}

static void mcd_physical_cache_invalidate()
{
	pthread_mutex_lock(&mcd_physical_mutex);
	while (mcd_physical_busy) pthread_cond_wait(&mcd_physical_cond, &mcd_physical_mutex);
	for (int i = 0; i < MCD_PHYSICAL_CACHE_SLOTS; i++) mcd_physical_cache[i].valid = 0;
	pthread_mutex_unlock(&mcd_physical_mutex);
}

static void *mcd_physical_worker(void *)
{
	cpu_set_t cpus;
	CPU_ZERO(&cpus);
	CPU_SET(0, &cpus);
	CPU_SET(1, &cpus);
	pthread_setaffinity_np(pthread_self(), sizeof(cpus), &cpus);
	while (1) {
		pthread_mutex_lock(&mcd_physical_mutex);
		while (!mcd_physical_busy) pthread_cond_wait(&mcd_physical_cond, &mcd_physical_mutex);
		int slot = mcd_physical_request_slot;
		int lba = mcd_physical_request_lba;
		pthread_mutex_unlock(&mcd_physical_mutex);
		mcd_physical_read_uncached(mcd_physical_cache[slot].data, lba, MCD_PHYSICAL_CACHE_SECTORS);
		pthread_mutex_lock(&mcd_physical_mutex);
		mcd_physical_cache[slot].start_lba = lba;
		mcd_physical_cache[slot].valid = mcd_physical_toc && cdd.IsPhysical();
		mcd_physical_busy = 0;
		pthread_cond_broadcast(&mcd_physical_cond);
		pthread_mutex_unlock(&mcd_physical_mutex);
	}
	return NULL;
}

static void mcd_physical_cache_start()
{
	if (mcd_physical_thread_started) return;
	if (!pthread_create(&mcd_physical_thread, NULL, mcd_physical_worker, NULL)) {
		mcd_physical_thread_started = 1;
		printf("MCD-CD: asynchronous read-ahead enabled (%d x %d sectors)\n", MCD_PHYSICAL_CACHE_SLOTS, MCD_PHYSICAL_CACHE_SECTORS);
	}
}

static int mcd_physical_cache_find(int lba, int count)
{
	for (int i = 0; i < MCD_PHYSICAL_CACHE_SLOTS; i++) {
		if (mcd_physical_cache[i].valid && lba >= mcd_physical_cache[i].start_lba &&
			(lba + count) <= (mcd_physical_cache[i].start_lba + MCD_PHYSICAL_CACHE_SECTORS)) return i;
	}
	return -1;
}

static void mcd_physical_cache_queue(int slot, int lba)
{
	mcd_physical_cache[slot].valid = 0;
	mcd_physical_request_slot = slot;
	mcd_physical_request_lba = lba;
	mcd_physical_busy = 1;
	pthread_cond_signal(&mcd_physical_cond);
}

static void mcd_physical_read(uint8_t *buffer, int lba, int count)
{
	uint32_t wait_started = mcd_stats_now_ms();
	if (!mcd_physical_thread_started) {
		mcd_physical_read_uncached(buffer, lba, count);
		return;
	}
	pthread_mutex_lock(&mcd_physical_mutex);
	int slot = mcd_physical_cache_find(lba, count);
	int missed = slot < 0;
	while (slot < 0) {
		while (mcd_physical_busy) pthread_cond_wait(&mcd_physical_cond, &mcd_physical_mutex);
		slot = mcd_physical_cache_find(lba, count);
		if (slot >= 0) break;
		int target = (!mcd_physical_cache[1].valid ||
			(mcd_physical_cache[0].valid && mcd_physical_cache[0].start_lba <= mcd_physical_cache[1].start_lba)) ? 1 : 0;
		mcd_physical_cache_queue(target, lba);
	}
	for (int i = 0; i < count; i++) {
		memcpy(buffer + i * MCD_PHYSICAL_SECTOR_SIZE,
			mcd_physical_cache[slot].data + (lba - mcd_physical_cache[slot].start_lba + i) * MCD_PHYSICAL_FRAME_SIZE,
			MCD_PHYSICAL_SECTOR_SIZE);
	}
	int next_lba = mcd_physical_cache[slot].start_lba + MCD_PHYSICAL_CACHE_SECTORS;
	if (mcd_physical_toc && next_lba < (int)mcd_physical_toc->end && !mcd_physical_busy && !mcd_physical_cache_find(next_lba, 1))
		mcd_physical_cache_queue(slot ^ 1, next_lba);
	pthread_mutex_unlock(&mcd_physical_mutex);
	uint32_t waited = mcd_stats_now_ms() - wait_started;
	pthread_mutex_lock(&mcd_stats_mutex);
	if (missed) {
		mcd_stats_cache_misses++;
		mcd_stats_cache_wait_ms += waited;
		if (waited > mcd_stats_cache_wait_max_ms) mcd_stats_cache_wait_max_ms = waited;
	} else {
		mcd_stats_cache_hits++;
	}
	pthread_mutex_unlock(&mcd_stats_mutex);
}

static void mcd_physical_read_subcode(uint8_t *buffer, int lba)
{
	(void)lba;
	memset(buffer, 0, MCD_PHYSICAL_SUBCODE_SIZE);
}

cdd_t::cdd_t() {
	latency = 10;
	loaded = 0;
	index = 0;
	lba = 0;
	scanOffset = 0;
	isData = 1;
	status = CD_STAT_NO_DISC;
	audioLength = 0;
	audioOffset = 0;
	chd_hunkbuf = NULL;
	chd_hunknum = -1;
	physical = 0;
	SendData = NULL;
	CanSendData = NULL;

	stat[0] = 0xB;
	stat[1] = 0x0;
	stat[2] = 0x0;
	stat[3] = 0x0;
	stat[4] = 0x0;
	stat[5] = 0x0;
	stat[6] = 0x0;
	stat[7] = 0x0;
	stat[8] = 0x0;
	stat[9] = 0x4;
}

static int sgets(char *out, int sz, char **in)
{
	*out = 0;
	do
	{
		char *instr = *in;
		int cnt = 0;

		while (*instr && *instr != 10)
		{
			if (*instr == 13)
			{
				instr++;
				continue;
			}

			if (cnt < sz - 1)
			{
				out[cnt++] = *instr;
				out[cnt] = 0;
			}

			instr++;
		}

		if(*instr == 10) instr++;
		*in = instr;
	}
	while (!*out && **in);

	return *out;
}


int cdd_t::LoadCUE(const char* filename) {
	static char fname[1024 + 10];
	static char line[128];
	char *ptr, *lptr;
	static char header[1024];
	static char toc[100 * 1024];

	strcpy(fname, filename);

	memset(toc, 0, sizeof(toc));
	if (!FileLoad(fname, toc, sizeof(toc) - 1)) return 1;

	printf("\x1b[32mMCD: Open CUE: %s\n\x1b[0m", fname);

	int mm, ss, bb, pregap = 0;

	char *buf = toc;
	while (sgets(line, sizeof(line), &buf))
	{
		lptr = line;
		while (*lptr == 0x20) lptr++;

		/* decode FILE commands */
		if (!(memcmp(lptr, "FILE", 4)))
		{
			ptr = fname + strlen(fname) - 1;
			while ((ptr - fname) && (*ptr != '/') && (*ptr != '\\')) ptr--;
			if (ptr - fname) ptr++;

			lptr += 4;
			while (*lptr == 0x20) lptr++;

			if (*lptr == '\"')
			{
				lptr++;
				while ((*lptr != '\"') && (lptr <= (line + 128)) && (ptr < (fname + 1023)))
					*ptr++ = *lptr++;
			}
			else
			{
				while ((*lptr != 0x20) && (lptr <= (line + 128)) && (ptr < (fname + 1023)))
					*ptr++ = *lptr++;
			}
			*ptr = 0;

			if(!FileOpen(&this->toc.tracks[this->toc.last].f, fname)) return -1;

			printf("\x1b[32mMCD: Open track file: %s\n\x1b[0m", fname);

			pregap = 0;

			this->toc.tracks[this->toc.last].offset = 0;

			if (!strstr(lptr, "BINARY") && !strstr(lptr, "MOTOROLA") && !strstr(lptr, "WAVE"))
			{
				FileClose(&this->toc.tracks[this->toc.last].f);
				printf("\x1b[32mMCD: unsupported file: %s\n\x1b[0m", fname);

				return -1;
			}
		}

		/* decode TRACK commands */
		else if ((sscanf(lptr, "TRACK %02d %*s", &bb)) || (sscanf(lptr, "TRACK %d %*s", &bb)))
		{
			if (bb != (this->toc.last + 1))
			{
				FileClose(&this->toc.tracks[this->toc.last].f);
				printf("\x1b[32mMCD: missing tracks: %s\n\x1b[0m", fname);
				break;
			}

			if (!this->toc.last)
			{
				if (strstr(lptr, "MODE1/2048"))
				{
					this->toc.tracks[0].sector_size = 2048;
				}
				else if (strstr(lptr, "MODE1/2352"))
				{
					this->toc.tracks[0].sector_size = 2352;

					FileSeek(&this->toc.tracks[0].f, 0x10, SEEK_SET);
				}

				if (this->toc.tracks[0].sector_size)
				{
					this->toc.tracks[0].type = TT_MODE1;

					FileReadAdv(&this->toc.tracks[0].f, header, 0x210);
					FileSeek(&this->toc.tracks[0].f, 0, SEEK_SET);
				}
			}
			else
			{
				if (!this->toc.tracks[this->toc.last].f.opened())
				{
					this->toc.tracks[this->toc.last - 1].end = 0;
				}
			}
		}

		/* decode PREGAP commands */
		else if (sscanf(lptr, "PREGAP %02d:%02d:%02d", &mm, &ss, &bb) == 3)
		{
			pregap += bb + ss * 75 + mm * 60 * 75;
		}

		/* decode INDEX commands */
		else if ((sscanf(lptr, "INDEX 00 %02d:%02d:%02d", &mm, &ss, &bb) == 3) ||
			(sscanf(lptr, "INDEX 0 %02d:%02d:%02d", &mm, &ss, &bb) == 3))
		{
			if (this->toc.last && !this->toc.tracks[this->toc.last - 1].end)
			{
				this->toc.tracks[this->toc.last - 1].end = bb + ss * 75 + mm * 60 * 75 + pregap;
			}
		}
		else if ((sscanf(lptr, "INDEX 01 %02d:%02d:%02d", &mm, &ss, &bb) == 3) ||
			(sscanf(lptr, "INDEX 1 %02d:%02d:%02d", &mm, &ss, &bb) == 3))
		{
			this->toc.tracks[this->toc.last].offset += pregap * 2352;

			if (!this->toc.tracks[this->toc.last].f.opened())
			{
				FileOpen(&this->toc.tracks[this->toc.last].f, fname);
				this->toc.tracks[this->toc.last].start = bb + ss * 75 + mm * 60 * 75 + pregap;
				if (this->toc.last && !this->toc.tracks[this->toc.last - 1].end)
				{
					this->toc.tracks[this->toc.last - 1].end = this->toc.tracks[this->toc.last].start;
				}
			}
			else
			{
				FileSeek(&this->toc.tracks[this->toc.last].f, 0, SEEK_SET);

				this->toc.tracks[this->toc.last].start = this->toc.end + pregap;
				this->toc.tracks[this->toc.last].offset += this->toc.end * 2352;

				int sectorSize = 2352;
				if (this->toc.tracks[this->toc.last].type) sectorSize = this->toc.tracks[0].sector_size;
				this->toc.tracks[this->toc.last].end = this->toc.tracks[this->toc.last].start + ((this->toc.tracks[this->toc.last].f.size + sectorSize - 1) / sectorSize);

				this->toc.tracks[this->toc.last].start += (bb + ss * 75 + mm * 60 * 75);
				this->toc.end = this->toc.tracks[this->toc.last].end;
			}

			printf("\x1b[32mMCD: Track = %u, start = %u, end = %u, offset = %u, type = %u\n\x1b[0m", cdd.toc.last, cdd.toc.tracks[cdd.toc.last].start, cdd.toc.tracks[cdd.toc.last].end, cdd.toc.tracks[cdd.toc.last].offset, cdd.toc.tracks[cdd.toc.last].type);

			this->toc.last++;
			if (this->toc.last == 99) break;
		}
	}

	if (this->toc.last && !this->toc.tracks[this->toc.last - 1].end)
	{
		this->toc.end += pregap;
		this->toc.tracks[this->toc.last - 1].end = this->toc.end;
	}

        memcpy(&fname[strlen(fname) - 4], ".sub", 4);
        FileOpen(&this->toc.sub, getFullPath(fname));

	FileClose(&this->toc.tracks[this->toc.last].f);
	return 0;
}

int cdd_t::Load(const char *filename)
{
	//char fname[1024 + 10];
	static char header[32];
	fileTYPE *fd_img;

	Unload();

	const char *ext = filename+strlen(filename)-4;
	if (!strncasecmp(".cue", ext, 4))
	{
		if (LoadCUE(filename)) {
			return (-1);
		}
	} else if (!strncasecmp(".chd", ext, 4))  {
		chd_error err = mister_load_chd(filename, &this->toc);
		if (err != CHDERR_NONE)
		{
			printf("ERROR %s\n", chd_error_string(err));
			return -1;
		}

		if (this->chd_hunkbuf)
		{
			free(this->chd_hunkbuf);
		}

		this->chd_hunkbuf = (uint8_t *)malloc(this->toc.chd_hunksize);
		this->chd_hunknum = -1;
 	} else {
		return (-1);

	}

	if (this->toc.chd_f)
	{
		mister_chd_read_sector(this->toc.chd_f, 0, 0, 0, 0x10, (uint8_t *)header, this->chd_hunkbuf, &this->chd_hunknum);
	} else {
		fd_img = &this->toc.tracks[0].f;

		FileSeek(fd_img, 0, SEEK_SET);
		FileReadAdv(fd_img, header, 0x10);
	}

	if (this->toc.tracks[0].sector_size)
	{
		this->sectorSize = this->toc.tracks[0].sector_size;
	}
	else if (!memcmp("SEGADISCSYSTEM", header, 14))
	{
		this->sectorSize = 2048;
	}
	else
	{
		this->sectorSize = 2352;
	}

	printf("\x1b[32mMCD: Sector size = %u, Track 0 end = %u\n\x1b[0m", this->sectorSize, this->toc.tracks[0].end);

	if (this->toc.last)
	{
		this->toc.tracks[this->toc.last].start = this->toc.end;
		this->loaded = 1;

		printf("\x1b[32mMCD: CD mounted , last track = %u\n\x1b[0m", this->toc.last);

		return 1;
	}

	return 0;
}

int cdd_t::LoadPhysical()
{
	Unload();
	if (!PhysicalDiscPresent()) return 0;
	struct cdrom_tochdr header = {};
	if (ioctl(mcd_physical_fd, CDROMREADTOCHDR, &header) < 0) return 0;
	int count = header.cdth_trk1 - header.cdth_trk0 + 1;
	if (count < 1 || count > 99) return 0;
	struct cdrom_tocentry entries[100] = {};
	for (int i = 0; i < count; i++) {
		entries[i].cdte_track = header.cdth_trk0 + i;
		entries[i].cdte_format = CDROM_LBA;
		if (ioctl(mcd_physical_fd, CDROMREADTOCENTRY, &entries[i]) < 0) return 0;
	}
	entries[count].cdte_track = CDROM_LEADOUT;
	entries[count].cdte_format = CDROM_LBA;
	if (ioctl(mcd_physical_fd, CDROMREADTOCENTRY, &entries[count]) < 0) return 0;
	for (int i = 0; i < count; i++) {
		this->toc.tracks[i].start = entries[i].cdte_addr.lba;
		this->toc.tracks[i].end = entries[i + 1].cdte_addr.lba;
		this->toc.tracks[i].sector_size = MCD_PHYSICAL_SECTOR_SIZE;
		this->toc.tracks[i].type = (entries[i].cdte_ctrl & CDROM_DATA_TRACK) ? TT_MODE1 : TT_CDDA;
		printf("MCD-CD: track %d start=%u end=%u type=%s\n", i + 1,
			this->toc.tracks[i].start, this->toc.tracks[i].end,
			this->toc.tracks[i].type ? "data" : "audio");
	}
	this->toc.last = count;
	this->toc.end = entries[count].cdte_addr.lba;
	this->toc.tracks[count].start = this->toc.end;
	this->sectorSize = MCD_PHYSICAL_SECTOR_SIZE;
	this->physical = 1;
	this->loaded = 1;
	mcd_physical_toc = &this->toc;
	mcd_physical_cache_start();
	mcd_physical_cache_invalidate();
	mcd_physical_set_max_speed();
	mcd_physical_stats_reset();
	return 1;
}

int cdd_t::IsPhysical() const { return physical; }

void cdd_t::Unload()
{
	mcd_physical_cache_invalidate();
	mcd_physical_toc = NULL;
	if (this->loaded)
	{
		if (this->toc.chd_f)
		{
			chd_close(this->toc.chd_f);
		}

		if (this->chd_hunkbuf)
		{
			free(this->chd_hunkbuf);
			this->chd_hunkbuf = NULL;
		}

		for (int i = 0; i < this->toc.last; i++)
		{
			if (this->toc.tracks[i].f.opened())
			{
				FileClose(&this->toc.tracks[i].f);
			}
		}

		if (this->toc.sub.opened()) FileClose(&this->toc.sub);

		this->loaded = 0;
	}

	memset(&this->toc, 0x00, sizeof(this->toc));
	this->sectorSize = 0;
	this->physical = 0;
}

void cdd_t::Reset() {
	latency = 10;
	index = 0;
	lba = 0;
	scanOffset = 0;
	isData = 1;
	status = CD_STAT_STOP;
	audioLength = 0;
	audioOffset = 0;
	chd_audio_read_lba = 0;

	stat[0] = 0x0;
	stat[1] = 0x0;
	stat[2] = 0x0;
	stat[3] = 0x0;
	stat[4] = 0x0;
	stat[5] = 0x0;
	stat[6] = 0x0;
	stat[7] = 0x0;
	stat[8] = 0x0;
	stat[9] = 0xF;
}

void cdd_t::Update() {
	if (this->status == CD_STAT_STOP || this->status == CD_STAT_TRAY || this->status == CD_STAT_OPEN)
	{
		if (this->latency > 0)
		{
			this->latency--;
			return;
		}
		// Neo Geo CDZ does not like the status changing to TOC here.
		//this->status = this->loaded ? CD_STAT_TOC : CD_STAT_NO_DISC;
		if (!this->loaded)
		{
			this->status = CD_STAT_NO_DISC;
		}
	}
	else if (this->status == CD_STAT_SEEK)
	{
		if (this->latency > 0)
		{
			this->latency--;
			return;
		}
		this->status = CD_STAT_PAUSE;
	}
	else if (this->status == CD_STAT_PLAY)
	{
		if (this->latency > 0)
		{
			this->latency--;
			return;
		}

		if (this->index >= this->toc.last)
		{
			this->status = CD_STAT_END;
			return;
		}

		if (CanSendData && !CanSendData(this->toc.tracks[this->index].type))
		{
			// Not ready yet to receive sector
			return;
		}

		if (this->toc.tracks[this->index].type)
		{
			// CD-ROM (Mode 1)
			uint8_t header[4];
			msf_t msf;
			LBAToMSF(this->lba + 150, &msf);
			header[0] = BCD(msf.m);
			header[1] = BCD(msf.s);
			header[2] = BCD(msf.f);
			header[3] = 0x01;

			SectorSend(header);
		}
		else
		{
			if (this->lba >= this->toc.tracks[this->index].start)
			{
				this->isData = 0x00;
			}
			SectorSend(0);
		}

		this->lba++;
		this->chd_audio_read_lba++;

		if (this->lba >= this->toc.tracks[this->index].end)
		{
			this->index++;

			this->isData = 0x01;

			if (!this->physical && this->toc.tracks[this->index].f.opened())
			{
				FileSeek(&this->toc.tracks[this->index].f, (this->toc.tracks[this->index].start * 2352) - this->toc.tracks[this->index].offset, SEEK_SET);
			}
		}
	}
	else if (cdd.status == CD_STAT_SCAN)
	{
		this->lba += this->scanOffset;

		if (this->lba >= this->toc.tracks[this->index].end)
		{
			this->index++;
			if (this->index < this->toc.last)
			{
				this->lba = this->toc.tracks[this->index].start;
			}
			else
			{
				this->lba = this->toc.end;
				this->chd_audio_read_lba = this->lba;
				this->status = CD_STAT_END;
				this->isData = 0x01;
				return;
			}
		}
		else if (this->lba < this->toc.tracks[this->index].start)
		{
			if (this->index > 0)
			{
				this->index--;
				this->lba = this->toc.tracks[this->index].end;
			}
			else
			{
				this->lba = 0;
			}
		}

		this->chd_audio_read_lba = this->lba;

		this->isData = this->toc.tracks[this->index].type;

		if (!this->physical && this->toc.sub.opened()) FileSeek(&this->toc.sub, this->lba * 96, SEEK_SET);

		if (this->physical)
		{
			// Physical reads are positioned by LBA.
		}
		else if (this->toc.tracks[this->index].type)
		{
			// DATA track
			FileSeek(&this->toc.tracks[0].f, this->lba * this->sectorSize, SEEK_SET);
		}
		else if (this->toc.tracks[this->index].f.opened())
		{
			// AUDIO track
			FileSeek(&this->toc.tracks[this->index].f, (this->lba * 2352) - this->toc.tracks[this->index].offset, SEEK_SET);
		}
	}
}

void cdd_t::CommandExec() {
	msf_t msf;

	switch (comm[0]) {
	case CD_COMM_IDLE:
		if (this->latency <= 3)
		{
			stat[0] = this->status;
			if (stat[1] == 0x0f)
			{
				int lba = this->lba + 150;
				LBAToMSF(lba, &msf);
				stat[1] = 0x0;
	                        stat[2] = BCD(msf.m) >> 4;
				stat[3] = BCD(msf.m) & 0xF;
				stat[4] = BCD(msf.s) >> 4;
				stat[5] = BCD(msf.s) & 0xF;
				stat[6] = BCD(msf.f) >> 4;
				stat[7] = BCD(msf.f) & 0xF;
				stat[8] = this->toc.tracks[this->index].type ? 0x04 : 0x00;
			} else if (stat[1] == 0x00) {
				int lba = this->lba + 150;
				LBAToMSF(lba, &msf);
                                stat[2] = BCD(msf.m) >> 4;
                                stat[3] = BCD(msf.m) & 0xF;
                                stat[4] = BCD(msf.s) >> 4;
                                stat[5] = BCD(msf.s) & 0xF;
                                stat[6] = BCD(msf.f) >> 4;
                                stat[7] = BCD(msf.f) & 0xF;
                                stat[8] = this->toc.tracks[this->index].type ? 0x04 : 0x00;
			} else if (stat[1] == 0x01) {
				int lba = abs(this->lba - this->toc.tracks[this->index].start);
				LBAToMSF(lba,&msf);
                                stat[2] = BCD(msf.m) >> 4;
                                stat[3] = BCD(msf.m) & 0xF;
                                stat[4] = BCD(msf.s) >> 4;
                                stat[5] = BCD(msf.s) & 0xF;
                                stat[6] = BCD(msf.f) >> 4;
                                stat[7] = BCD(msf.f) & 0xF;
                                stat[8] = this->toc.tracks[this->index].type ? 0x04 : 0x00;
			} else if (stat[1] == 0x02) {
                               stat[2] = (cdd.index < this->toc.last) ? BCD(this->index + 1) >> 4 : 0xA;
                               stat[3] = (cdd.index < this->toc.last) ? BCD(this->index + 1) & 0xF : 0xA;
			}
		}

		//printf("MCD: Command IDLE, status = %u\n\x1b[0m", this->status);
		break;

	case CD_COMM_STOP:
		this->status = CD_STAT_STOP;
		this->isData = 1;

		stat[0] = this->status;
		stat[1] = 0;
		stat[2] = 0;
		stat[3] = 0;
		stat[4] = 0;
		stat[5] = 0;
		stat[6] = 0;
		stat[7] = 0;
		stat[8] = 0;

		//printf("\x1b[32mMCD: Command STOP, status = %u, frame = %u\n\x1b[0m", this->status, frame);
		break;

	case CD_COMM_TOC:
		if (this->status == CD_STAT_STOP) {
			this->status = CD_STAT_TOC;
		}
		switch (comm[3]) {
		case 0: {
			int lba_ = this->lba + 150;
			LBAToMSF(lba_, &msf);

			stat[0] = this->status;
			stat[1] = 0x0;
			stat[2] = BCD(msf.m) >> 4;
			stat[3] = BCD(msf.m) & 0xF;
			stat[4] = BCD(msf.s) >> 4;
			stat[5] = BCD(msf.s) & 0xF;
			stat[6] = BCD(msf.f) >> 4;
			stat[7] = BCD(msf.f) & 0xF;
			stat[8] = this->toc.tracks[this->index].type << 2;

			//printf("\x1b[32mMCD: Command TOC 0, lba = %i, command = %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X, status = %02X%08X, frame = %u\n\x1b[0m", lba, comm[9], comm[8], comm[7], comm[6], comm[5], comm[4], comm[3], comm[2], comm[1], comm[0], (uint32_t)(GetStatus() >> 32), (uint32_t)GetStatus(), frame);
		}
			break;

		case 1: {
			int lba_ = abs(this->lba - this->toc.tracks[this->index].start);
			LBAToMSF(lba_, &msf);

			stat[0] = this->status;
			stat[1] = 0x1;
			stat[2] = BCD(msf.m) >> 4;
			stat[3] = BCD(msf.m) & 0xF;
			stat[4] = BCD(msf.s) >> 4;
			stat[5] = BCD(msf.s) & 0xF;
			stat[6] = BCD(msf.f) >> 4;
			stat[7] = BCD(msf.f) & 0xF;
			stat[8] = this->toc.tracks[this->index].type << 2;

			//printf("\x1b[32mMCD: Command TOC 1, lba = %i, command = %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X, status = %02X%08X, frame = %u\n\x1b[0m", lba, comm[9], comm[8], comm[7], comm[6], comm[5], comm[4], comm[3], comm[2], comm[1], comm[0], (uint32_t)(GetStatus() >> 32), (uint32_t)GetStatus(), frame);
		}
			break;

		case 2: {
			stat[0] = this->status;
			stat[1] = 0x2;
			stat[2] = ((this->index < this->toc.last) ?  BCD(this->index + 1) >> 4 : 0xA);
			stat[3] = ((this->index < this->toc.last) ? BCD(this->index + 1) & 0xF : 0xA);
			stat[4] = 0;
			stat[5] = 0;
			stat[6] = 0;
			stat[7] = 0;
			stat[8] = 0;

			//printf("\x1b[32mMCD: Command TOC 2, index = %i, command = %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X, status = %02X%08X, frame = %u\n\x1b[0m", this->index, comm[9], comm[8], comm[7], comm[6], comm[5], comm[4], comm[3], comm[2], comm[1], comm[0], (uint32_t)(GetStatus() >> 32), (uint32_t)GetStatus(), frame);

		}
			break;

		case 3: {
			int lba_ = this->toc.end + 150;
			LBAToMSF(lba_, &msf);

			stat[0] = this->status;
			stat[1] = 0x3;
			stat[2] = BCD(msf.m) >> 4;
			stat[3] = BCD(msf.m) & 0xF;
			stat[4] = BCD(msf.s) >> 4;
			stat[5] = BCD(msf.s) & 0xF;
			stat[6] = BCD(msf.f) >> 4;
			stat[7] = BCD(msf.f) & 0xF;
			stat[8] = 0;

			//printf("\x1b[32mMCD: Command TOC 3, lba = %i, command = %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X, frame = %u\n\x1b[0m", lba, comm[9], comm[8], comm[7], comm[6], comm[5], comm[4], comm[3], comm[2], comm[1], comm[0], frame);
		}
			break;

		case 4: {
			stat[0] = this->status;
			stat[1] = 0x4;
			stat[2] = 0;
			stat[3] = 1;
			stat[4] = BCD(this->toc.last) >> 4;
			stat[5] = BCD(this->toc.last) & 0xF;
			stat[6] = 0;
			stat[7] = 0;
			stat[8] = 0;

			//printf("\x1b[32mMCD: Command TOC 4, last = %i, command = %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X, frame = %u\n\x1b[0m", this->toc.last, comm[9], comm[8], comm[7], comm[6], comm[5], comm[4], comm[3], comm[2], comm[1], comm[0], frame);
		}
			break;

		case 5: {
			int track = comm[4] * 10 + comm[5];
			int lba_ = this->toc.tracks[track - 1].start + 150;
			LBAToMSF(lba_, &msf);

			stat[0] = this->status;
			stat[1] = 0x5;
			stat[2] = BCD(msf.m) >> 4;
			stat[3] = BCD(msf.m) & 0xF;
			stat[4] = BCD(msf.s) >> 4;
			stat[5] = BCD(msf.s) & 0xF;
			stat[6] = (BCD(msf.f) >> 4) | (this->toc.tracks[track - 1].type << 3);
			stat[7] = BCD(msf.f) & 0xF;
			stat[8] = BCD(track) & 0xF;

			//printf("\x1b[32mMCD: Command TOC 5, lba = %i, track = %i, command = %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X, frame = %u\n\x1b[0m", lba_, track, comm[9], comm[8], comm[7], comm[6], comm[5], comm[4], comm[3], comm[2], comm[1], comm[0], frame);
		}
			break;

		case 6:
			stat[0] = this->status;
			stat[1] = 0x6;
			stat[2] = 0;
			stat[3] = 0;
			stat[4] = 0;
			stat[5] = 0;
			stat[6] = 0;
			stat[7] = 0;
			stat[8] = 0;
			break;

		default:

			break;
		}
		break;

	case CD_COMM_PLAY: {
		int lba_;
		MSFToLBA(&lba_, comm[2] * 10 + comm[3], comm[4] * 10 + comm[5],	comm[6] * 10 + comm[7]);
		lba_ -= 150;

		SeekToLBA(lba_, 1);

		this->isData = 1;

		this->status = CD_STAT_PLAY;

		stat[0] = CD_STAT_SEEK;
		stat[1] = 0xf;
		stat[2] = 0;
		stat[3] = 0;
		stat[4] = 0;
		stat[5] = 0;
		stat[6] = 0;
		stat[7] = 0;
		stat[8] = 0;

		//printf("\x1b[32mMCD: Command PLAY, lba = %i, index = %i, command = %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X, frame = %u\n\x1b[0m", lba_, this->index, comm[9], comm[8], comm[7], comm[6], comm[5], comm[4], comm[3], comm[2], comm[1], comm[0], frame);
	}
		break;

	case CD_COMM_SEEK: {
		int lba_;
		MSFToLBA(&lba_, comm[2] * 10 + comm[3], comm[4] * 10 + comm[5], comm[6] * 10 + comm[7]);
		lba_ -= 150;

		SeekToLBA(lba_, 0);

		this->isData = 1;

		this->status = CD_STAT_SEEK;

		stat[0] = this->status;
		stat[1] = 0xf;
		stat[2] = 0;
		stat[3] = 0;
		stat[4] = 0;
		stat[5] = 0;
		stat[6] = 0;
		stat[7] = 0;
		stat[8] = 0;

		//printf("\x1b[32mMCD: Command PLAY, lba = %i, index = %i, command = %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X, frame = %u\n\x1b[0m", lba_, this->index, comm[9], comm[8], comm[7], comm[6], comm[5], comm[4], comm[3], comm[2], comm[1], comm[0], frame);
	}
		break;

	case CD_COMM_PAUSE:
		this->isData = 0x01;

		this->status = CD_STAT_PAUSE;

		stat[0] = this->status;
		//printf("\x1b[32mMCD: Command PAUSE, status = %X, frame = %u\n\x1b[0m", this->status, frame);
		break;

	case CD_COMM_RESUME:
		this->status = CD_STAT_PLAY;
		stat[0] = this->status;
		this->audioOffset = 0;
		//printf("\x1b[32mMCD: Command RESUME, status = %X\n\x1b[0m", this->status);
		break;

	case CD_COMM_FW_SCAN:
		this->scanOffset = CD_SCAN_SPEED;
		this->status = CD_STAT_SCAN;
		stat[0] = this->status;
		break;

	case CD_COMM_RW_SCAN:
		this->scanOffset = -CD_SCAN_SPEED;
		this->status = CD_STAT_SCAN;
		stat[0] = this->status;
		break;

	case CD_COMM_TRACK_MOVE:
		this->isData = 1;
		this->status = CD_STAT_PAUSE;
		stat[0] = this->status;
		break;

	case CD_COMM_TRACK_PLAY: {
		int index = comm[2] * 10 + comm[3];
		if (index > 0)
		{
			index -= 1;
		}
		int lba = this->toc.tracks[index].start;

		SeekToLBA(lba, 1);

		this->isData = 1;

		this->status = CD_STAT_PLAY;

		stat[0] = CD_STAT_SEEK;
		stat[1] = 0xf;
		stat[2] = 0;
		stat[3] = 0;
		stat[4] = 0;
		stat[5] = 0;
		stat[6] = 0;
		stat[7] = 0;
		stat[8] = 0;

		//printf("\x1b[32mMCD: Command CD_COMM_TRACK_PLAY, index: %u, status = %u \n\x1b[0m", index, this->status);
	}
		break;

	case CD_COMM_TRAY_CLOSE:
		this->isData = 1;
		this->status = this->loaded ? CD_STAT_TOC : CD_STAT_NO_DISC;
		stat[0] = CD_STAT_STOP;

		//printf("\x1b[32mMCD: Command TRAY_CLOSE, status = %u, frame = %u\n\x1b[0m", this->status, frame);
		break;

	case CD_COMM_TRAY_OPEN:
		this->isData = 1;
		this->status = CD_STAT_OPEN;
		stat[0] = CD_STAT_OPEN;

		//printf("\x1b[32mMCD: Command TRAY_OPEN, status = %u, frame = %u\n\x1b[0m", this->status, frame);
		break;

	default:
		stat[0] = this->status;

		//printf("\x1b[32mMCD: Command undefined, status = %u, frame = %u\n\x1b[0m", this->status, frame);
		break;
	}
}

uint64_t cdd_t::GetStatus(uint8_t crc_start) {
	uint8_t n9 = ~(crc_start + stat[0] + stat[1] + stat[2] + stat[3] + stat[4] + stat[5] + stat[6] + stat[7] + stat[8]);
	return ((uint64_t)(n9 & 0xF) << 36) |
		((uint64_t)(stat[8] & 0xF) << 32) |
		((uint64_t)(stat[7] & 0xF) << 28) |
		((uint64_t)(stat[6] & 0xF) << 24) |
		((uint64_t)(stat[5] & 0xF) << 20) |
		((uint64_t)(stat[4] & 0xF) << 16) |
		((uint64_t)(stat[3] & 0xF) << 12) |
		((uint64_t)(stat[2] & 0xF) << 8) |
		((uint64_t)(stat[1] & 0xF) << 4) |
		((uint64_t)(stat[0] & 0xF) << 0);
}

int cdd_t::SetCommand(uint64_t c, uint8_t crc_start) {
	comm[0] = (c >> 0) & 0xF;
	comm[1] = (c >> 4) & 0xF;
	comm[2] = (c >> 8) & 0xF;
	comm[3] = (c >> 12) & 0xF;
	comm[4] = (c >> 16) & 0xF;
	comm[5] = (c >> 20) & 0xF;
	comm[6] = (c >> 24) & 0xF;
	comm[7] = (c >> 28) & 0xF;
	comm[8] = (c >> 32) & 0xF;
	comm[9] = (c >> 36) & 0xF;

	uint8_t crc = (~(crc_start + comm[0] + comm[1] + comm[2] + comm[3] + comm[4] + comm[5] + comm[6] + comm[7] + comm[8])) & 0xF;
	if (comm[9] != crc)
		return -1;

	return 0;
}

void cdd_t::LBAToMSF(int lba, msf_t* msf) {
	msf->m = (lba / 75) / 60;
	msf->s = (lba / 75) % 60;
	msf->f = (lba % 75);
}

void cdd_t::MSFToLBA(int* lba, uint8_t m, uint8_t s, uint8_t f) {
	*lba = f + s * 75 + m * 60 * 75;
}

void cdd_t::MSFToLBA(int* lba, msf_t* msf) {
	*lba = msf->f + msf->s * 75 + msf->m * 60 * 75;
}

void cdd_t::SeekToLBA(int lba, int play) {
	int index = 0;

	this->latency = 0;
	if (play)
	{
		this->latency = 11;
	}

	this->latency += (abs(lba - this->lba) * 120) / 270000;

	this->lba = lba;

	while ((this->toc.tracks[index].end <= lba) && (index < this->toc.last)) index++;
	this->index = index;

	if (lba < this->toc.tracks[index].start)
	{
		lba = this->toc.tracks[index].start;
	}

	if (this->physical)
	{
		// Physical reads are positioned by LBA and need no file seek.
	}
	else if (this->toc.tracks[index].type)
	{
		/* DATA track */
		FileSeek(&this->toc.tracks[0].f, lba * this->sectorSize, SEEK_SET);
	}
	else if (this->toc.tracks[index].f.opened())
	{
		/* PCM AUDIO track */
		FileSeek(&this->toc.tracks[index].f, (lba * 2352) - this->toc.tracks[index].offset, SEEK_SET);
	}

	if (play)
	{
		this->chd_audio_read_lba = this->lba;
		this->audioOffset = 0;
	}

	if (this->toc.sub.opened()) FileSeek(&this->toc.sub, lba * 96, SEEK_SET);

}

void cdd_t::ReadData(uint8_t *buf)
{
	if (this->toc.tracks[this->index].type && (this->lba >= 0))
	{
		if (this->physical)
		{
			uint8_t raw[MCD_PHYSICAL_SECTOR_SIZE];
			mcd_physical_read(raw, this->lba, 1);
			memcpy(buf, raw + 16, 2048);
			return;
		}

		if (this->toc.chd_f)
		{
			int read_offset = 0;
			if (this->sectorSize != 2048)
			{
				read_offset += 16;
			}

			mister_chd_read_sector(this->toc.chd_f, this->lba + this->toc.tracks[0].offset, 0, read_offset, 2048, buf, this->chd_hunkbuf, &this->chd_hunknum);
		} else {
			if (this->sectorSize == 2048)
			{
				FileSeek(&this->toc.tracks[0].f, this->lba * 2048, SEEK_SET);
			} else {
			FileSeek(&this->toc.tracks[0].f, this->lba * 2352 + 16, SEEK_SET);
			}
			FileReadAdv(&this->toc.tracks[0].f, buf, 2048);
		}
	}
}

int cdd_t::ReadCDDA(uint8_t *buf)
{
	this->audioLength = 2352 + 2352 - this->audioOffset;
	this->audioOffset = 2352;

	//printf("\x1b[32mMCD: AUDIO LENGTH %d LBA: %d INDEX: %d START: %d END %d\n\x1b[0m", this->audioLength, this->lba, this->index, this->toc.tracks[this->index].start, this->toc.tracks[this->index].end);
	//

	if (this->isData)
	{
		return this->audioLength;
	}

	if (this->physical)
	{
		mcd_physical_read(buf, this->lba, this->audioLength / MCD_PHYSICAL_SECTOR_SIZE);
		return this->audioLength;
	}

	if (this->toc.chd_f)
	{
		for(int i = 0; i < this->audioLength / 2352; i++)
		{
			mister_chd_read_sector(this->toc.chd_f, this->chd_audio_read_lba + this->toc.tracks[this->index].offset, 2352*i, 0, 2352, buf, this->chd_hunkbuf, &this->chd_hunknum);
		}

		//CHD audio requires byteswap. There's probably a better way to do this...

		for (int swapidx = 0; swapidx < this->audioLength; swapidx += 2)
		{
			uint8_t temp = buf[swapidx];
			buf[swapidx] = buf[swapidx+1];
			buf[swapidx+1] = temp;
		}

		if ((this->audioLength / 2352) > 1)
		{
			this->chd_audio_read_lba++;
		}

	} else if (this->toc.tracks[this->index].f.opened()) {
		FileReadAdv(&this->toc.tracks[this->index].f, buf, this->audioLength);
	}

	return this->audioLength;
}

void InterleaveSubcode(uint8_t *subc_data, uint16_t *buf)
{
	for(int i = 0, n=0; i < 96; i+=2,n++)
	{
		int code = 0;
		for (int j = 0; j < 8; j++)
		{
			int bits = (subc_data[(j * 12) + (i / 8)] >> (6 - (i&6))) & 3;
			code |= ((bits & 1) << (15 - j));
			code |= ((bits >> 1) << (7 - j));
		}
		buf[n] = code;
	}
}

int cdd_t::ReadSubcode(uint16_t* buf)
{
	int err = 0;
	uint8_t subc[96];
	if (this->physical)
	{
		// MMC returns the 96 representable P-W symbols; Mega CD's CDC consumes
		// a 98-byte frame whose first two positions are the non-byte EFM sync
		// symbols. Keep those placeholders clear and append the drive payload.
		memset(buf, 0, 98);
		mcd_physical_read_subcode((uint8_t *)buf + 2, this->lba);
	}
	else if (this->toc.chd_f)
	{
		//Just use the read sector call with an offset, since we previously read that sector, it is already in the hunk cache
		if (this->toc.tracks[this->index].sbc_type == SUBCODE_RW_RAW) {
			mister_chd_read_sector(this->toc.chd_f, this->chd_audio_read_lba + this->toc.tracks[this->index].offset, 0, CD_MAX_SECTOR_DATA, 96, (uint8_t *)buf, this->chd_hunkbuf, &this->chd_hunknum);
		} else if (this->toc.tracks[this->index].sbc_type == SUBCODE_RW) {
			mister_chd_read_sector(this->toc.chd_f, this->chd_audio_read_lba + this->toc.tracks[this->index].offset, 0, CD_MAX_SECTOR_DATA, 96, subc, this->chd_hunkbuf, &this->chd_hunknum);
			InterleaveSubcode(subc, buf);
		} else {
			err = -1;
		}
	} else if (this->toc.sub.opened()) {
		FileReadAdv(&this->toc.sub, subc, 96);
		InterleaveSubcode(subc, buf);
	} else {
		err = -1;
	}

	return err;
}


int cdd_t::SectorSend(uint8_t* header)
{
	uint8_t buf[2352 + 2352];
	int len = 2352;
	uint8_t index = MCD_DATA_IO_INDEX;

	if (header) {
		memcpy(buf + 12, header, 4);
		ReadData(buf + 16);
	}
	else {
		index = MCD_CDDA_IO_INDEX;
		len = ReadCDDA(buf);
	}

	SubcodeSend();
	if (SendData)
		return SendData(buf, len, index);

	return 0;
}


int cdd_t::SubcodeSend()
{
	// Diagnostic isolation: keep requesting and caching raw P-W data from the
	// physical drive, but do not inject it into Mega CD's CDC. This separates
	// optical throughput from subcode framing/interrupt behavior.
	if (this->physical) return 0;

	uint16_t buf[98 / 2] = {};

	int err = ReadSubcode(buf);

	if (!err && SendData)
		return SendData((uint8_t*)buf, 98, MCD_SUB_IO_INDEX);

	return 0;
}
