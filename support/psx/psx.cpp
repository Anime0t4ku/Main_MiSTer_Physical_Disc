
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <errno.h>
#include <climits>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <linux/cdrom.h>
#include <scsi/sg.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>

#ifndef IOPRIO_CLASS_SHIFT
#define IOPRIO_CLASS_SHIFT 13
#endif
#ifndef IOPRIO_CLASS_IDLE
#define IOPRIO_CLASS_IDLE 3
#endif
#ifndef IOPRIO_WHO_PROCESS
#define IOPRIO_WHO_PROCESS 1
#endif
#define PSX_IOPRIO_VALUE(class_, data_) (((class_) << IOPRIO_CLASS_SHIFT) | (data_))

#include "../../file_io.h"
#include "../../user_io.h"
#include "../../spi.h"
#include "../../hardware.h"
#include "../../menu.h"
#include "psx.h"
#include "mcdheader.h"
#include "../../cd.h"
#include "../chd/mister_chd.h"
#include <libchdr/chd.h>

static char buf[1024];
static uint8_t *chd_hunkbuf = NULL;
static int chd_hunknum;
static int noreset = 0;
static int physical_cd_fd = -1;
static int physical_cd_enabled = 0;
static int physical_disc_present = 0;
static uint32_t physical_poll_timer = 0;

#define PHYSICAL_CD_DEVICE "/dev/sr0"
#define PHYSICAL_CACHE_SLOTS 64
#define PHYSICAL_CACHE_SLOTS_PER_STREAM (PHYSICAL_CACHE_SLOTS / 2)
#define PHYSICAL_CACHE_SECTORS 64
#define PHYSICAL_PACKET_SECTORS 16
#define PHYSICAL_PREFETCH_PACKET_SECTORS 16
#define PHYSICAL_PREFETCH_HIGH_SECTORS 1792
#define PHYSICAL_PREFETCH_LOW_SECTORS 128
#define PHYSICAL_CD_SPEED 0
#define PHYSICAL_STREAM_DATA 0
#define PHYSICAL_STREAM_CDDA 1

struct physical_cache_slot_t
{
	int start_lba;
	int valid_sectors;
	int valid;
	int stream;
	uint8_t data[PHYSICAL_CACHE_SECTORS * 2352];
};

static physical_cache_slot_t physical_cache[PHYSICAL_CACHE_SLOTS] = {};
static pthread_mutex_t physical_cache_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t physical_cache_cond = PTHREAD_COND_INITIALIZER;
static pthread_t physical_cache_thread;
static int physical_cache_thread_started = 0;
static int physical_cache_busy = 0;
static int physical_cache_request_slot = 0;
static int physical_cache_request_lba = 0;
static int physical_cache_request_sectors = PHYSICAL_CACHE_SECTORS;
static int physical_cache_request_prefetch = 0;
static int physical_cache_request_packet_sectors = PHYSICAL_PACKET_SECTORS;
static int physical_stream_lba[2] = {-1, -1};
static int physical_stream_coasting[2] = {0, 0};
static int physical_last_read_end[2] = {-1, -1};
static int physical_cache_demand_waiting = 0;
static int physical_cache_failed_lba = -1;
static int physical_cache_failed_end = -1;
static int physical_cache_schedule_prefetch_locked(int stream);

static pthread_mutex_t psx_stats_mutex = PTHREAD_MUTEX_INITIALIZER;
static uint64_t psx_stats_packet_calls;
static uint64_t psx_stats_packet_sectors;
static uint64_t psx_stats_packet_ms;
static uint32_t psx_stats_packet_max_ms;
static uint64_t psx_stats_packet_failures;
static uint64_t psx_stats_fallback_sectors;
static uint64_t psx_stats_cache_hits;
static uint64_t psx_stats_cache_misses;
static uint64_t psx_stats_cache_wait_ms;
static uint32_t psx_stats_cache_wait_max_ms;
static uint64_t psx_stats_contention_misses;
static uint64_t psx_stats_contention_wait_ms;
static uint32_t psx_stats_contention_wait_max_ms;
static uint64_t psx_stats_cold_misses;
static uint64_t psx_stats_cold_wait_ms;
static uint32_t psx_stats_cold_wait_max_ms;
static uint64_t psx_stats_read_calls;
static uint64_t psx_stats_read_sectors;
static uint64_t psx_stats_sequential_reads;
static uint64_t psx_stats_random_reads;
static uint64_t psx_stats_validated_sectors;
static uint64_t psx_stats_bad_sync;
static uint64_t psx_stats_bad_msf;
static uint64_t psx_stats_bad_mode;
static uint64_t psx_stats_bad_subheader;
static uint64_t psx_stats_bad_edc;
static uint64_t psx_stats_mode1_sectors;
static uint64_t psx_stats_mode2_form1_sectors;
static uint64_t psx_stats_mode2_form2_sectors;
static int psx_stats_last_read_end[2] = {-1, -1};
static uint32_t psx_stats_timer;

#define PSX_DELIVERY_TRACE_MAX 2048
struct psx_delivery_trace_t {
	uint32_t t;
	uint32_t lba;
	int blks;
	int buffer_hit;
	uint32_t gap_us;
	uint32_t read_us;
	uint32_t spi_us;
	uint32_t postfill_us;
	uint32_t total_us;
};
static psx_delivery_trace_t psx_delivery_trace[PSX_DELIVERY_TRACE_MAX];
static int psx_delivery_trace_count;
static int psx_delivery_trace_dumped;
static uint64_t psx_delivery_last_request_us;

#define PSX_INTEGRITY_TRACE_MAX 256
struct psx_integrity_trace_t {
	uint32_t t;
	int lba;
	uint8_t flags;
	uint8_t expected_msf[3];
	uint8_t actual_msf[3];
	uint8_t mode;
	uint8_t subheader_a[4];
	uint8_t subheader_b[4];
	uint32_t expected_edc;
	uint32_t actual_edc;
};
static psx_integrity_trace_t psx_integrity_trace[PSX_INTEGRITY_TRACE_MAX];
static int psx_integrity_trace_count;
static int psx_integrity_trace_dumped;

#define PSX_MISS_TRACE_MAX 256
struct psx_miss_trace_t {
	uint32_t t;
	int lba, cnt, sequential, busy, worker_lba, worker_sectors, worker_prefetch, stream_lba;
	int valid[PHYSICAL_CACHE_SLOTS];
	int start[PHYSICAL_CACHE_SLOTS];
	int end[PHYSICAL_CACHE_SLOTS];
	uint32_t wait_ms;
};
static psx_miss_trace_t psx_miss_trace[PSX_MISS_TRACE_MAX];
static int psx_miss_trace_count;
static int psx_miss_trace_dumped;

static uint32_t psx_stats_now_ms()
{
	struct timespec tp;
	clock_gettime(CLOCK_MONOTONIC, &tp);
	return (uint32_t)(tp.tv_sec * 1000ULL + tp.tv_nsec / 1000000);
}

uint64_t psx_trace_now_us()
{
	struct timespec tp;
	clock_gettime(CLOCK_MONOTONIC, &tp);
	return tp.tv_sec * 1000000ULL + tp.tv_nsec / 1000;
}

void psx_trace_delivery(uint32_t lba, int blks, int buffer_hit, uint64_t request_us,
	uint64_t read_us, uint64_t spi_us, uint64_t postfill_us)
{
	if (!physical_cd_enabled || psx_delivery_trace_count >= PSX_DELIVERY_TRACE_MAX) return;
	psx_delivery_trace_t *e = &psx_delivery_trace[psx_delivery_trace_count++];
	e->t = GetTimer(0);
	e->lba = lba;
	e->blks = blks;
	e->buffer_hit = buffer_hit;
	uint64_t gap = psx_delivery_last_request_us ? request_us - psx_delivery_last_request_us : 0;
	e->gap_us = gap > UINT32_MAX ? UINT32_MAX : (uint32_t)gap;
	e->read_us = read_us > UINT32_MAX ? UINT32_MAX : (uint32_t)read_us;
	e->spi_us = spi_us > UINT32_MAX ? UINT32_MAX : (uint32_t)spi_us;
	e->postfill_us = postfill_us > UINT32_MAX ? UINT32_MAX : (uint32_t)postfill_us;
	uint64_t total = psx_trace_now_us() - request_us;
	e->total_us = total > UINT32_MAX ? UINT32_MAX : (uint32_t)total;
	psx_delivery_last_request_us = request_us;
}

static void psx_physical_stats_reset()
{
	pthread_mutex_lock(&psx_stats_mutex);
	psx_stats_packet_calls = psx_stats_packet_sectors = psx_stats_packet_ms = 0;
	psx_stats_packet_max_ms = 0;
	psx_stats_packet_failures = psx_stats_fallback_sectors = 0;
	psx_stats_cache_hits = psx_stats_cache_misses = psx_stats_cache_wait_ms = 0;
	psx_stats_cache_wait_max_ms = 0;
	psx_stats_contention_misses = psx_stats_contention_wait_ms = 0;
	psx_stats_contention_wait_max_ms = 0;
	psx_stats_cold_misses = psx_stats_cold_wait_ms = 0;
	psx_stats_cold_wait_max_ms = 0;
	psx_stats_read_calls = psx_stats_read_sectors = 0;
	psx_stats_sequential_reads = psx_stats_random_reads = 0;
	psx_stats_validated_sectors = psx_stats_bad_sync = psx_stats_bad_msf = 0;
	psx_stats_bad_mode = psx_stats_bad_subheader = psx_stats_bad_edc = 0;
	psx_stats_mode1_sectors = psx_stats_mode2_form1_sectors = psx_stats_mode2_form2_sectors = 0;
	psx_stats_last_read_end[PHYSICAL_STREAM_DATA] = -1;
	psx_stats_last_read_end[PHYSICAL_STREAM_CDDA] = -1;
	psx_stats_timer = GetTimer(5000);
	pthread_mutex_unlock(&psx_stats_mutex);
	FILE *f = fopen("/tmp/psx_physical_stats.log", "w");
	if (f) {
		fprintf(f, "PSX physical-disc telemetry started (v0.46)\n");
		fclose(f);
	}
	psx_miss_trace_count = psx_miss_trace_dumped = 0;
	psx_delivery_trace_count = psx_delivery_trace_dumped = 0;
	psx_integrity_trace_count = psx_integrity_trace_dumped = 0;
	psx_delivery_last_request_us = 0;
	f = fopen("/tmp/psx_physical_misses.log", "w");
	if (f) {
		fprintf(f, "PSX cache miss trace: t lba cnt seq busy worker_lba worker_sectors prefetch stream slots(start-end) wait\n");
		fclose(f);
	}
	f = fopen("/tmp/psx_physical_delivery.log", "w");
	if (f) {
		fprintf(f, "PSX delivery trace v0.46: t lba blks source gap_us read_us spi_us postfill_us total_us\n");
		fclose(f);
	}
	f = fopen("/tmp/psx_physical_integrity.log", "w");
	if (f) {
		fprintf(f, "PSX raw-sector integrity trace v0.46: flags S=sync A=address M=mode H=subheader E=EDC\n");
		fclose(f);
	}
}

static void psx_physical_stats_poll()
{
	if (!psx_stats_timer || !CheckTimer(psx_stats_timer)) return;
	psx_stats_timer = GetTimer(5000);
	pthread_mutex_lock(&psx_stats_mutex);
	uint64_t pc = psx_stats_packet_calls, ps = psx_stats_packet_sectors, pm = psx_stats_packet_ms;
	uint32_t pmax = psx_stats_packet_max_ms;
	uint64_t pf = psx_stats_packet_failures, fb = psx_stats_fallback_sectors;
	uint64_t hit = psx_stats_cache_hits, miss = psx_stats_cache_misses, wm = psx_stats_cache_wait_ms;
	uint32_t wmax = psx_stats_cache_wait_max_ms;
	uint64_t cm = psx_stats_contention_misses, cwm = psx_stats_contention_wait_ms;
	uint32_t cwmax = psx_stats_contention_wait_max_ms;
	uint64_t cold = psx_stats_cold_misses, coldwm = psx_stats_cold_wait_ms;
	uint32_t coldmax = psx_stats_cold_wait_max_ms;
	uint64_t rc = psx_stats_read_calls, rs = psx_stats_read_sectors;
	uint64_t seq = psx_stats_sequential_reads, rnd = psx_stats_random_reads;
	uint64_t validated = psx_stats_validated_sectors, bad_sync = psx_stats_bad_sync;
	uint64_t bad_msf = psx_stats_bad_msf, bad_mode = psx_stats_bad_mode;
	uint64_t bad_sub = psx_stats_bad_subheader;
	uint64_t bad_edc = psx_stats_bad_edc, mode1 = psx_stats_mode1_sectors;
	uint64_t mode2f1 = psx_stats_mode2_form1_sectors, mode2f2 = psx_stats_mode2_form2_sectors;
	pthread_mutex_unlock(&psx_stats_mutex);
	pthread_mutex_lock(&physical_cache_mutex);
	int coast_data = physical_stream_coasting[PHYSICAL_STREAM_DATA];
	int coast_cdda = physical_stream_coasting[PHYSICAL_STREAM_CDDA];
	pthread_mutex_unlock(&physical_cache_mutex);
	FILE *f = fopen("/tmp/psx_physical_stats.log", "a");
	if (f) {
		fprintf(f, "t=%lu reads=%llu sectors=%llu sequential=%llu random=%llu packets=%llu packet_sectors=%llu packet_avg=%.1fms packet_max=%ums failures=%llu fallback_sectors=%llu cache_hit=%llu cache_miss=%llu wait_avg=%.1fms wait_max=%ums contention=%llu contention_avg=%.1fms contention_max=%ums cold=%llu cold_avg=%.1fms cold_max=%ums validated=%llu mode1=%llu mode2_form1=%llu mode2_form2=%llu bad_sync=%llu bad_msf=%llu bad_mode=%llu bad_subheader=%llu bad_edc=%llu coast_data=%d coast_cdda=%d\n",
			GetTimer(0) / 1000, (unsigned long long)rc, (unsigned long long)rs,
			(unsigned long long)seq, (unsigned long long)rnd, (unsigned long long)pc,
			(unsigned long long)ps, pc ? (double)pm / pc : 0.0, pmax,
			(unsigned long long)pf, (unsigned long long)fb, (unsigned long long)hit,
			(unsigned long long)miss, miss ? (double)wm / miss : 0.0, wmax,
			(unsigned long long)cm, cm ? (double)cwm / cm : 0.0, cwmax,
			(unsigned long long)cold, cold ? (double)coldwm / cold : 0.0, coldmax,
			(unsigned long long)validated, (unsigned long long)mode1,
			(unsigned long long)mode2f1, (unsigned long long)mode2f2,
			(unsigned long long)bad_sync,
			(unsigned long long)bad_msf, (unsigned long long)bad_mode,
			(unsigned long long)bad_sub, (unsigned long long)bad_edc, coast_data, coast_cdda);
		fclose(f);
	}
	f = fopen("/tmp/psx_physical_misses.log", "a");
	if (f) {
		while (psx_miss_trace_dumped < psx_miss_trace_count) {
			psx_miss_trace_t *e = &psx_miss_trace[psx_miss_trace_dumped++];
			fprintf(f, "t=%u lba=%d cnt=%d seq=%d busy=%d worker=%d+%d prefetch=%d stream=%d slots=",
				e->t, e->lba, e->cnt, e->sequential, e->busy, e->worker_lba,
				e->worker_sectors, e->worker_prefetch, e->stream_lba);
			for (int i = 0; i < PHYSICAL_CACHE_SLOTS; i++)
				fprintf(f, "%s%s%d-%d", i ? "," : "", e->valid[i] ? "" : "!", e->start[i], e->end[i]);
			fprintf(f, " wait=%ums\n", e->wait_ms);
		}
		fclose(f);
	}
	f = fopen("/tmp/psx_physical_delivery.log", "a");
	if (f) {
		while (psx_delivery_trace_dumped < psx_delivery_trace_count) {
			psx_delivery_trace_t *e = &psx_delivery_trace[psx_delivery_trace_dumped++];
			fprintf(f, "t=%u lba=%u blks=%d source=%s gap=%uus read=%uus spi=%uus postfill=%uus total=%uus\n",
				e->t, e->lba, e->blks, e->buffer_hit ? "buffer" : "refill",
				e->gap_us, e->read_us, e->spi_us, e->postfill_us, e->total_us);
		}
		fclose(f);
	}
	f = fopen("/tmp/psx_physical_integrity.log", "a");
	if (f) {
		while (psx_integrity_trace_dumped < psx_integrity_trace_count) {
			psx_integrity_trace_t *e = &psx_integrity_trace[psx_integrity_trace_dumped++];
			fprintf(f, "t=%u lba=%d flags=%s%s%s%s%s expected=%02X:%02X:%02X actual=%02X:%02X:%02X mode=%02X sub=%02X%02X%02X%02X/%02X%02X%02X%02X edc=%08X/%08X\n",
				e->t, e->lba, e->flags & 1 ? "S" : "", e->flags & 2 ? "A" : "",
				e->flags & 4 ? "M" : "", e->flags & 8 ? "H" : "", e->flags & 16 ? "E" : "",
				e->expected_msf[0], e->expected_msf[1], e->expected_msf[2],
				e->actual_msf[0], e->actual_msf[1], e->actual_msf[2], e->mode,
				e->subheader_a[0], e->subheader_a[1], e->subheader_a[2], e->subheader_a[3],
				e->subheader_b[0], e->subheader_b[1], e->subheader_b[2], e->subheader_b[3],
				e->expected_edc, e->actual_edc);
		}
		fclose(f);
	}
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

		if (*instr == 10) instr++;
		*in = instr;
	} while (!*out && **in);

	return *out;
}

static uint32_t libCryptSectors[16] =
{
	14105,
	14231,
	14485,
	14579,
	14649,
	14899,
	15056,
	15130,
	15242,
	15312,
	15378,
	15628,
	15919,
	16031,
	16101,
	16167,
};

static uint32_t msfToLba(uint32_t m, uint32_t s, uint32_t f)
{
	return (m * 60 + s) * 75 + f;
}

static uint8_t bcdToDec(uint8_t bcd)
{
	return (bcd >> 4) * 10 + (bcd & 0x0F);
}

#define SBI_HEADER_SIZE 4
#define SBI_BLOCK_SIZE  14

static uint16_t libCryptMask(fileTYPE* sbi_file)
{
	int sz;
	uint16_t mask = 0;
	if ((sz = FileReadAdv(sbi_file, buf, sizeof(buf))))
	{
		for (int i = 0;; i++)
		{
			int pos = SBI_HEADER_SIZE + i * SBI_BLOCK_SIZE;
			if (pos >= sz) break;
			uint32_t lba = msfToLba(bcdToDec(buf[pos]), bcdToDec(buf[pos + 1]), bcdToDec(buf[pos + 2]));
			for (int m = 0; m < 16; m++) if (libCryptSectors[m] == lba) mask |= (1 << (15 - m));
		}
	}

	return mask;
}

static void unload_chd(toc_t *table)
{
	if (table->chd_f)
	{
		chd_close(table->chd_f);
	}
	if (chd_hunkbuf) free(chd_hunkbuf);
	memset(table, 0, sizeof(toc_t));
	chd_hunknum = -1;

}

static void unload_cue(toc_t *table)
{
	for (int i = 0; i < table->last; i++)
	{
		FileClose(&table->tracks[i].f);
	}
	memset(table, 0, sizeof(toc_t));
}

static int load_chd(const char *filename, toc_t *table)
{

	unload_chd(table);
	chd_error err = mister_load_chd(filename, table);
	if (err != CHDERR_NONE)
	{
		return 0;
	}

	/* PSX core expects the TOC values for track start/end to not take into account
	* pregap, unlike some other cores. Adjust the CHD toc to reflect this
	*/

	for (int i = 0; i < table->last; i++)
	{
		if (i == 0) //First track fakes a pregap even if it doesn't exist
		{
			table->tracks[i].indexes[1] = 150;
			table->tracks[i].start = 150;
			table->tracks[i].end += 150-1;
		} else {
			int frame_cnt = table->tracks[i].end - table->tracks[i].start;
			frame_cnt += table->tracks[i].indexes[1];
			table->tracks[i].start = table->tracks[i-1].end + 1;
			table->tracks[i].end = table->tracks[i].start + frame_cnt - 1;
		}
	}

	table->end = table->tracks[table->last - 1].end + 1;

	chd_hunkbuf = (uint8_t *)malloc(table->chd_hunksize);
	chd_hunknum = -1;

	return 1;
}

static int load_cue(const char* filename, toc_t *table)
{
	static char fname[1024 + 10];
	static char line[128];
	char *ptr, *lptr;
	static char toc[100 * 1024];

	unload_cue(table);
	printf("\x1b[32mPSX: Open CUE: %s\n\x1b[0m", fname);

	strcpy(fname, filename);

	memset(toc, 0, sizeof(toc));
	if (!FileLoad(fname, toc, sizeof(toc) - 1))
	{
		printf("\x1b[32mPSX: cannot load file: %s\n\x1b[0m", fname);
		return 0;
	}

	int mm, ss, bb;
	int pregap = 0;

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

			if (!FileOpen(&table->tracks[table->last].f, fname)) return 0;

			printf("\x1b[32mPSX: Open track file: %s\n\x1b[0m", fname);

			table->tracks[table->last].offset = 0;

			if (!strstr(lptr, "BINARY"))
			{
				FileClose(&table->tracks[table->last].f);
				printf("\x1b[32mPSX: unsupported file: %s\n\x1b[0m", fname);
				return 0;
			}
		}

		/* decode PREGAP commands */
		else if (sscanf(lptr, "PREGAP %02d:%02d:%02d", &mm, &ss, &bb) == 3)
		{
			// Single bin specific, add pregab but subtract inherent pregap
			pregap += bb + ss * 75 + mm * 60 * 75;
      table->tracks[table->last].pregap = 1;

		}
		/* decode TRACK commands */
		else if ((sscanf(lptr, "TRACK %02d %*s", &bb)) || (sscanf(lptr, "TRACK %d %*s", &bb)))
		{
      pregap = 0;
			if (bb != (table->last + 1))
			{
				FileClose(&table->tracks[table->last].f);
				printf("\x1b[32mPSX: missing tracks: %s\n\x1b[0m", fname);
				return 0;
			}

			if (strstr(lptr, "MODE1/2352") || strstr(lptr, "MODE2/2352"))
			{
				table->tracks[table->last].sector_size = 2352;
				table->tracks[table->last].type = TT_MODE1;
				if (!table->last) table->end = 150; // implicit 2 seconds pregap for track 1
			}
			else if (strstr(lptr, "AUDIO"))
			{
				table->tracks[table->last].sector_size = 2352;
				table->tracks[table->last].type = TT_CDDA;
			}
			else
			{
				FileClose(&table->tracks[table->last].f);
				printf("\x1b[32mPSX: unsupported track type: %s\n\x1b[0m", lptr);
				return 0;
			}
		}

		/* decode INDEX commands */
		else if ((sscanf(lptr, "INDEX 00 %02d:%02d:%02d", &mm, &ss, &bb) == 3) ||
			(sscanf(lptr, "INDEX 0 %02d:%02d:%02d", &mm, &ss, &bb) == 3))
		{
			// Single bin specific
			if (!table->tracks[table->last].f.opened())
			{


        pregap = bb+ss*75+mm*60*75;
			}
		}
		else if ((sscanf(lptr, "INDEX 01 %02d:%02d:%02d", &mm, &ss, &bb) == 3) ||
			(sscanf(lptr, "INDEX 1 %02d:%02d:%02d", &mm, &ss, &bb) == 3))
		{
			if (!table->tracks[table->last].f.opened())
			{
        table->tracks[table->last].start = bb+ss*75+mm*60*75; 
        if (table->tracks[table->last].pregap)
          table->tracks[table->last].start += pregap;
        //Subtract the fake 150 sector pregap used for the first data track
        table->tracks[table->last].offset = table->tracks[table->last].start*table->tracks[table->last].sector_size;
        if (table->last)
        {
          table->tracks[table->last-1].end = table->tracks[table->last].start-1;
          if (pregap)
          {
            table->tracks[table->last].indexes[1] = table->tracks[table->last].start - pregap;
            if (!table->tracks[table->last].pregap)
            {
              table->tracks[table->last].offset -= 2352*table->tracks[table->last].indexes[1];
              table->tracks[table->last].indexes[1] = table->tracks[table->last].start - pregap;
            } else {
              table->tracks[table->last].indexes[1] = pregap;
            }
          }
        } else if (table->tracks[table->last].type) {
          table->tracks[table->last].indexes[1] = 150;
        }
			}
			else
			{
				table->tracks[table->last].indexes[1] = bb + ss * 75 + mm * 60 * 75;
				if (table->tracks[table->last].type && !table->last) table->tracks[table->last].indexes[1] = 150;
				table->tracks[table->last].start = table->end;
				table->end += (table->tracks[table->last].f.size / table->tracks[table->last].sector_size);
				table->tracks[table->last].offset = 0;
			}
			table->tracks[table->last].end = table->end - 1;
			table->last++;
			if (table->last >= 99) break;
		}
	}

	/*
	for (int i = 0; i < table->last; i++)
	{
		printf("\x1b[32mPSX: Track = %u, start = %u, end = %u, offset = %d, sector_size=%d, type = %u\n\x1b[0m", i, table->tracks[i].start, table->tracks[i].end, table->tracks[i].offset, table->tracks[i].sector_size, table->tracks[i].type);
		if (table->tracks[i].indexes[1])
			printf("\x1b[32mPSX: Track = %u,Index1 = %u seconds\n\x1b[0m", i, table->tracks[i].indexes[1] / 75);

	}*/

	return 1;
}

static int load_cd_image(const char *filename, toc_t *table)
{

	const char *ext = strrchr(filename, '.');
	if (!ext) return 0;

	if (!strncasecmp(".chd", ext, 4))
	{
		return load_chd(filename, table);
	}
	else if (!strncasecmp(".cue", ext, 4))
	{
		return load_cue(filename, table);
	}

	return 0;
}


struct track_t
{
	uint32_t start_lba;
	uint32_t end_lba;
	uint32_t bcd;
	uint32_t reserved;
};

struct disk_t
{
	uint32_t track_count;
	uint32_t total_lba;
	uint32_t total_bcd;
	uint16_t libcrypt_mask;
	uint16_t metadata; // lower 2 bits encode the region, 3rd bit is reset request, the other bits are reseved
	track_t  track[99];
};

enum region_t
{
	UNKNOWN = 0,
	JP,
	US,
	EU
};

static const char* region_string(region_t region)
{
	switch (region)
	{
		case region_t::JP: return "Japan";
		case region_t::US: return "USA";
		case region_t::EU: return "Europe";
		default: return "Unknown";
	}
}

#define BCD(v) ((uint8_t)((((v)/10) << 4) | ((v)%10)))

static void send_cue_and_metadata(toc_t *table, uint16_t libcrypt_mask, enum region_t region, int reset)
{
	disk_t *disk = new disk_t;
	if (disk)
	{
		for (int i = 0; i < table->last; i++)
		{
			printf("\x1b[32mPSX: Track = %u, start = %u, end = %u, offset = %d, sector_size=%d, type = %u\n\x1b[0m", i, table->tracks[i].start, table->tracks[i].end, table->tracks[i].offset, table->tracks[i].sector_size, table->tracks[i].type);
			if (table->tracks[i].indexes[1]) printf("\x1b[32mPSX: Track = %u,Index1 = %u seconds\n\x1b[0m", i, table->tracks[i].indexes[1] / 75);
		}

		memset(disk, 0, sizeof(disk_t));
		disk->libcrypt_mask = libcrypt_mask;
		disk->metadata = region; // the lower 2 bits of metadata contain the region
		if (reset) disk->metadata |= 4; // 3rd bit is reset request
		disk->track_count = (BCD(table->last) << 8) | table->last;
		disk->total_lba = table->end;
		int m = (disk->total_lba / 75) / 60;
		int s = (disk->total_lba / 75) % 60;
		disk->total_bcd = (BCD(m) << 8) | BCD(s);

		for (int i = 0; i < table->last; i++)
		{
			disk->track[i].start_lba = i ? table->tracks[i].start : 0;
			disk->track[i].end_lba = table->tracks[i].end;
			m = ((disk->track[i].start_lba + table->tracks[i].indexes[1]) / 75) / 60;
			s = ((disk->track[i].start_lba + table->tracks[i].indexes[1]) / 75) % 60;
			disk->track[i].bcd = ((BCD(m) << 8) | BCD(s)) | ((table->tracks[i].type ? 0 : 1) << 16);
		}

		user_io_set_index(251);
		user_io_set_download(1);
		user_io_file_tx_data((uint8_t *)disk, sizeof(disk_t));
		user_io_set_download(0);
		delete(disk);
	}
}

#define MCD_SIZE (128*1024)

static void psx_mount_save(const char *filename)
{
	user_io_set_index(2);
	if (strlen(filename))
	{
		FileGenerateSavePath(filename, buf, 0);
		user_io_file_mount(buf, 2, 1, MCD_SIZE);
		StoreIdx_S(2, buf);
	}
	else
	{
		user_io_file_mount("", 2);
		StoreIdx_S(2, "");
	}
}

void psx_fill_blanksave(uint8_t *buffer, uint32_t lba, int cnt)
{
	uint32_t offset = lba * 1024;
	uint32_t size = cnt * 1024;

	if ((offset + size) <= sizeof(mcdheader))
	{
		memcpy(buffer, mcdheader + offset, size);
	}
	else
	{
		memset(buffer, 0, size);
	}
}

static toc_t toc = {};
#define CD_SECTOR_LEN 2352

static void physical_cd_close()
{
	if (physical_cache_thread_started)
	{
		pthread_mutex_lock(&physical_cache_mutex);
		while (physical_cache_busy) pthread_cond_wait(&physical_cache_cond, &physical_cache_mutex);
		for (int i = 0; i < PHYSICAL_CACHE_SLOTS; i++) physical_cache[i].valid = 0;
		pthread_mutex_unlock(&physical_cache_mutex);
	}
	if (physical_cd_fd >= 0) close(physical_cd_fd);
	physical_cd_fd = -1;
	physical_disc_present = 0;
}

static int physical_cd_open()
{
	if (physical_cd_fd >= 0) return 1;
	physical_cd_fd = open(PHYSICAL_CD_DEVICE, O_RDONLY | O_NONBLOCK);
	if (physical_cd_fd < 0)
	{
		printf("PSX: cannot open %s: %s\n", PHYSICAL_CD_DEVICE, strerror(errno));
		return 0;
	}
	return 1;
}

static int physical_cd_has_disc()
{
	if (!physical_cd_open()) return 0;
	return ioctl(physical_cd_fd, CDROM_DRIVE_STATUS, CDSL_CURRENT) == CDS_DISC_OK;
}

static void physical_cd_clear_toc()
{
	unload_cue(&toc);
	unload_chd(&toc);
	memset(&toc, 0, sizeof(toc));
}

static int physical_cd_load_toc(toc_t *table)
{
	struct cdrom_tochdr header = {};
	struct cdrom_tocentry entries[100] = {};

	if (!physical_cd_has_disc()) return 0;
	if (ioctl(physical_cd_fd, CDROMREADTOCHDR, &header) < 0)
	{
		printf("PSX: CDROMREADTOCHDR failed: %s\n", strerror(errno));
		return 0;
	}

	int count = header.cdth_trk1 - header.cdth_trk0 + 1;
	if (count < 1 || count > 99) return 0;

	for (int i = 0; i < count; i++)
	{
		entries[i].cdte_track = header.cdth_trk0 + i;
		entries[i].cdte_format = CDROM_LBA;
		if (ioctl(physical_cd_fd, CDROMREADTOCENTRY, &entries[i]) < 0)
		{
			printf("PSX: CDROMREADTOCENTRY track %d failed: %s\n", i + 1, strerror(errno));
			return 0;
		}
	}

	entries[count].cdte_track = CDROM_LEADOUT;
	entries[count].cdte_format = CDROM_LBA;
	if (ioctl(physical_cd_fd, CDROMREADTOCENTRY, &entries[count]) < 0)
	{
		printf("PSX: CDROMREADTOCENTRY lead-out failed: %s\n", strerror(errno));
		return 0;
	}

	physical_cd_clear_toc();
	for (int i = 0; i < count; i++)
	{
		cd_track_t *track = &table->tracks[i];
		track->start = entries[i].cdte_addr.lba + 150;
		track->end = entries[i + 1].cdte_addr.lba + 149;
		track->sector_size = CD_SECTOR_LEN;
		track->type = (entries[i].cdte_ctrl & CDROM_DATA_TRACK) ? TT_MODE1 : TT_CDDA;
		track->indexes[1] = 0;
		printf("PSX: Physical track %d, start=%u, end=%u, type=%s\n",
			i + 1, track->start, track->end, track->type ? "data" : "audio");
	}

	table->last = count;
	table->end = entries[count].cdte_addr.lba + 150;
	return 1;
}

static int physical_cd_read_sector(uint8_t *buffer, int lba)
{
	if (!physical_disc_present || physical_cd_fd < 0 || lba < 150)
	{
		memset(buffer, 0, CD_SECTOR_LEN);
		return 0;
	}

	// CDROMREADRAW takes an MSF address and overwrites the same argument with
	// one complete 2352-byte raw sector. MiSTer LBAs include the standard
	// 150-frame lead-in, so they already map directly to absolute MSF here.
	union
	{
		struct cdrom_msf msf;
		uint8_t raw[CD_SECTOR_LEN];
	} request = {};

	int absolute_frame = lba;
	request.msf.cdmsf_min0 = absolute_frame / (60 * 75);
	request.msf.cdmsf_sec0 = (absolute_frame / 75) % 60;
	request.msf.cdmsf_frame0 = absolute_frame % 75;
	absolute_frame++;
	request.msf.cdmsf_min1 = absolute_frame / (60 * 75);
	request.msf.cdmsf_sec1 = (absolute_frame / 75) % 60;
	request.msf.cdmsf_frame1 = absolute_frame % 75;

	if (ioctl(physical_cd_fd, CDROMREADRAW, &request) < 0)
	{
		printf("PSX: physical CD read failed at LBA %d: %s\n", lba - 150, strerror(errno));
		memset(buffer, 0, CD_SECTOR_LEN);
		return 0;
	}
	memcpy(buffer, request.raw, CD_SECTOR_LEN);
	return 1;
}

static int physical_cd_track_for_lba(int lba)
{
	for (int i = 0; i < toc.last; i++)
	{
		if (lba >= (int)toc.tracks[i].start && lba <= (int)toc.tracks[i].end) return i;
	}
	return -1;
}

static uint8_t physical_cd_to_bcd(int value)
{
	return (uint8_t)(((value / 10) << 4) | (value % 10));
}

static uint32_t physical_cd_sector_edc(const uint8_t *data, int length)
{
	static uint32_t table[256];
	static int initialized;
	if (!initialized) {
		for (int i = 0; i < 256; i++) {
			uint32_t c = i;
			for (int bit = 0; bit < 8; bit++)
				c = (c >> 1) ^ ((c & 1) ? 0xD8018001U : 0);
			table[i] = c;
		}
		initialized = 1;
	}
	uint32_t edc = 0;
	for (int i = 0; i < length; i++)
		edc = (edc >> 8) ^ table[(edc ^ data[i]) & 0xFF];
	return edc;
}

static uint32_t physical_cd_read_le32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
		((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void physical_cd_validate_data_sector(const uint8_t *sector, int lba)
{
	static const uint8_t sync_pattern[12] =
		{0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
	int minute = lba / (60 * 75);
	int second = (lba / 75) % 60;
	int frame = lba % 75;
	uint8_t expected[3] = {physical_cd_to_bcd(minute), physical_cd_to_bcd(second), physical_cd_to_bcd(frame)};
	uint8_t flags = 0;
	if (memcmp(sector, sync_pattern, sizeof(sync_pattern))) flags |= 1;
	if (memcmp(sector + 12, expected, sizeof(expected))) flags |= 2;
	if (sector[15] != 1 && sector[15] != 2) flags |= 4;
	if (sector[15] == 2 && memcmp(sector + 16, sector + 20, 4)) flags |= 8;
	uint32_t calculated_edc = 0;
	uint32_t stored_edc = 0;
	int sector_kind = 0;
	if (sector[15] == 1) {
		sector_kind = 1;
		calculated_edc = physical_cd_sector_edc(sector, 2064);
		stored_edc = physical_cd_read_le32(sector + 2064);
	} else if (sector[15] == 2 && (sector[18] & 0x20)) {
		sector_kind = 3;
		calculated_edc = physical_cd_sector_edc(sector + 16, 2332);
		stored_edc = physical_cd_read_le32(sector + 2348);
	} else if (sector[15] == 2) {
		sector_kind = 2;
		calculated_edc = physical_cd_sector_edc(sector + 16, 2056);
		stored_edc = physical_cd_read_le32(sector + 2072);
	}
	if (sector_kind && calculated_edc != stored_edc) flags |= 16;

	pthread_mutex_lock(&psx_stats_mutex);
	psx_stats_validated_sectors++;
	if (flags & 1) psx_stats_bad_sync++;
	if (flags & 2) psx_stats_bad_msf++;
	if (flags & 4) psx_stats_bad_mode++;
	if (flags & 8) psx_stats_bad_subheader++;
	if (flags & 16) psx_stats_bad_edc++;
	if (sector_kind == 1) psx_stats_mode1_sectors++;
	else if (sector_kind == 2) psx_stats_mode2_form1_sectors++;
	else if (sector_kind == 3) psx_stats_mode2_form2_sectors++;
	if (flags && psx_integrity_trace_count < PSX_INTEGRITY_TRACE_MAX) {
		psx_integrity_trace_t *e = &psx_integrity_trace[psx_integrity_trace_count++];
		e->t = GetTimer(0);
		e->lba = lba;
		e->flags = flags;
		memcpy(e->expected_msf, expected, 3);
		memcpy(e->actual_msf, sector + 12, 3);
		e->mode = sector[15];
		memcpy(e->subheader_a, sector + 16, 4);
		memcpy(e->subheader_b, sector + 20, 4);
		e->expected_edc = calculated_edc;
		e->actual_edc = stored_edc;
	}
	pthread_mutex_unlock(&psx_stats_mutex);
}

static int physical_cd_read_packet(uint8_t *buffer, int lba, int cnt, int track)
{
	if (physical_cd_fd < 0 || cnt < 1 || track < 0) return 0;

	uint8_t cdb[12] = {};
	uint8_t sense[32] = {};
	struct sg_io_hdr io = {};
	int drive_lba = lba - 150;

	cdb[0] = GPCMD_READ_CD;
	cdb[2] = (drive_lba >> 24) & 0xFF;
	cdb[3] = (drive_lba >> 16) & 0xFF;
	cdb[4] = (drive_lba >> 8) & 0xFF;
	cdb[5] = drive_lba & 0xFF;
	cdb[6] = (cnt >> 16) & 0xFF;
	cdb[7] = (cnt >> 8) & 0xFF;
	cdb[8] = cnt & 0xFF;

	// For CD-DA, the user-data field is the complete 2352-byte audio frame.
	// Data tracks need sync, headers, user data and EDC/ECC for a raw frame.
	cdb[9] = toc.tracks[track].type == TT_CDDA ? 0x10 : 0xF8;

	io.interface_id = 'S';
	io.cmd_len = sizeof(cdb);
	io.cmdp = cdb;
	io.dxfer_direction = SG_DXFER_FROM_DEV;
	io.dxfer_len = cnt * CD_SECTOR_LEN;
	io.dxferp = buffer;
	io.sbp = sense;
	io.mx_sb_len = sizeof(sense);
	io.timeout = 3000;

	int ioctl_result = ioctl(physical_cd_fd, SG_IO, &io);
	int ok = ioctl_result >= 0 && !io.status && !io.host_status && !io.driver_status;
	if (!ok)
	{
		printf("PSX-CD: SG_IO READ CD failed at LBA %d (%d sectors): ioctl=%d errno=%d status=%u host=%u driver=%u sense=%02X/%02X/%02X\n",
			drive_lba, cnt, ioctl_result, errno, io.status, io.host_status, io.driver_status,
			io.sb_len_wr > 2 ? sense[2] & 0x0F : 0,
			io.sb_len_wr > 12 ? sense[12] : 0,
			io.sb_len_wr > 13 ? sense[13] : 0);
		return 0;
	}
	return 1;
}

static int physical_cd_read_sectors_uncached(uint8_t *buffer, int lba, int cnt, int packet_limit)
{
	int success = 1;
	while (cnt > 0)
	{
		int track = physical_cd_track_for_lba(lba);
		if (track < 0 || lba < 150)
		{
			memset(buffer, 0, CD_SECTOR_LEN);
			buffer += CD_SECTOR_LEN;
			lba++;
			cnt--;
			continue;
		}

		int chunk = cnt;
		int remaining_in_track = toc.tracks[track].end - lba + 1;
		if (chunk > remaining_in_track) chunk = remaining_in_track;
		if (chunk > packet_limit) chunk = packet_limit;

		if (!physical_cd_read_packet(buffer, lba, chunk, track))
		{
			// Some USB optical bridges do not expose MMC packet commands through
			// this ioctl. Preserve compatibility with the older one-sector path.
			for (int i = 0; i < chunk; i++)
				if (!physical_cd_read_sector(buffer + i * CD_SECTOR_LEN, lba + i)) success = 0;
		}

		buffer += chunk * CD_SECTOR_LEN;
		lba += chunk;
		cnt -= chunk;
	}
	return success;
}

static void physical_cache_invalidate()
{
	pthread_mutex_lock(&physical_cache_mutex);
	for (int i = 0; i < PHYSICAL_CACHE_SLOTS; i++) physical_cache[i].valid = 0;
	physical_stream_lba[PHYSICAL_STREAM_DATA] = -1;
	physical_stream_lba[PHYSICAL_STREAM_CDDA] = -1;
	physical_stream_coasting[PHYSICAL_STREAM_DATA] = 0;
	physical_stream_coasting[PHYSICAL_STREAM_CDDA] = 0;
	physical_last_read_end[PHYSICAL_STREAM_DATA] = -1;
	physical_last_read_end[PHYSICAL_STREAM_CDDA] = -1;
	physical_cache_failed_lba = -1;
	physical_cache_failed_end = -1;
	pthread_mutex_unlock(&physical_cache_mutex);
}

static void *physical_cache_worker(void *)
{
	// Keep speculative optical work below MiSTer's time-sensitive main loop.
	// Linux applies nice levels per thread, while IOPRIO_CLASS_IDLE allows any
	// normal-priority storage activity to take precedence over SG_IO refills.
	int nice_result = setpriority(PRIO_PROCESS, 0, 19);
	int ioprio_result = syscall(SYS_ioprio_set, IOPRIO_WHO_PROCESS, 0,
		PSX_IOPRIO_VALUE(IOPRIO_CLASS_IDLE, 0));
	printf("PSX-CD: optical worker priority nice=%s ioprio=%s\n",
		nice_result == 0 ? "idle" : "unsupported",
		ioprio_result == 0 ? "idle" : "unsupported");

	// main() pins the process to CPU1 before creating this thread. Give the
	// blocking optical worker access to both ARM cores so it doesn't compete
	// exclusively with MiSTer's time-sensitive main loop.
	cpu_set_t cpus;
	CPU_ZERO(&cpus);
	CPU_SET(0, &cpus);
	CPU_SET(1, &cpus);
	pthread_setaffinity_np(pthread_self(), sizeof(cpus), &cpus);

	while (1)
	{
		pthread_mutex_lock(&physical_cache_mutex);
		while (!physical_cache_busy) pthread_cond_wait(&physical_cache_cond, &physical_cache_mutex);
		int slot = physical_cache_request_slot;
		int lba = physical_cache_request_lba;
		int sectors = physical_cache_request_sectors;
		int was_prefetch = physical_cache_request_prefetch;
		int request_packet_sectors = physical_cache_request_packet_sectors;
		pthread_mutex_unlock(&physical_cache_mutex);

		int completed = 0;
		int cancelled = 0;
		int success = 1;
		while (completed < sectors)
		{
			int chunk = sectors - completed;
			int packet_limit = was_prefetch ? request_packet_sectors : PHYSICAL_PACKET_SECTORS;
			if (chunk > packet_limit) chunk = packet_limit;
			if (!physical_cd_read_sectors_uncached(physical_cache[slot].data + completed * CD_SECTOR_LEN,
				lba + completed, chunk, packet_limit)) success = 0;
			completed += chunk;

			// Speculative refills use shorter MMC transactions. If the
			// core seeks while one is running, stop after the current transaction
			// so the demand read isn't stuck behind the remaining packets.
			if (was_prefetch && completed < sectors)
			{
				pthread_mutex_lock(&physical_cache_mutex);
				cancelled = physical_cache_demand_waiting;
				pthread_mutex_unlock(&physical_cache_mutex);
				if (cancelled) break;
			}
		}

		pthread_mutex_lock(&physical_cache_mutex);
		if (success && completed > 0)
		{
			physical_cache[slot].start_lba = lba;
			// An interrupted speculative fill may already contain the demand
			// sector. Publish the completed packets instead of throwing them away.
			physical_cache[slot].valid_sectors = completed;
			physical_cache[slot].valid = physical_cd_enabled && physical_disc_present;
		}
		else if (!cancelled)
		{
			physical_cache[slot].valid = 0;
			physical_cache_failed_lba = lba;
			physical_cache_failed_end = lba + sectors;
		}
		physical_cache_busy = 0;
		pthread_cond_broadcast(&physical_cache_cond);
		// Do not recursively schedule another speculative window here. Demand
		// traffic advances the bounded look-ahead from the foreground read path.
		pthread_mutex_unlock(&physical_cache_mutex);
	}
	return NULL;
}

static void physical_cache_start()
{
	if (physical_cache_thread_started) return;
	if (!pthread_create(&physical_cache_thread, NULL, physical_cache_worker, NULL))
	{
		physical_cache_thread_started = 1;
		printf("PSX-CD: split data/CDDA read-ahead enabled (%d x %d sectors, %.1f MiB)\n",
			PHYSICAL_CACHE_SLOTS, PHYSICAL_CACHE_SECTORS,
			(double)(PHYSICAL_CACHE_SLOTS * PHYSICAL_CACHE_SECTORS * CD_SECTOR_LEN) / (1024.0 * 1024.0));
	}
	else
	{
		printf("PSX-CD: unable to start read-ahead worker\n");
	}
}

static int physical_cache_find(int lba, int cnt, int stream)
{
	for (int i = 0; i < PHYSICAL_CACHE_SLOTS; i++)
	{
		if (physical_cache[i].valid && physical_cache[i].stream == stream && lba >= physical_cache[i].start_lba &&
			(lba + cnt) <= (physical_cache[i].start_lba + physical_cache[i].valid_sectors)) return i;
	}
	return -1;
}

static int physical_cache_choose_demand_victim(int lba, int stream)
{
	int first = stream * PHYSICAL_CACHE_SLOTS_PER_STREAM;
	int last = first + PHYSICAL_CACHE_SLOTS_PER_STREAM;
	for (int i = first; i < last; i++)
		if (!physical_cache[i].valid) return i;

	int victim = first;
	int victim_distance = -1;
	for (int i = first; i < last; i++)
	{
		int start = physical_cache[i].start_lba;
		int end = start + physical_cache[i].valid_sectors;
		int distance = lba < start ? start - lba : (lba >= end ? lba - end + 1 : 0);
		if (distance > victim_distance) {
			victim = i;
			victim_distance = distance;
		}
	}
	return victim;
}

static void physical_cache_queue(int slot, int lba, int sectors, int prefetch, int stream,
	int packet_sectors = PHYSICAL_PACKET_SECTORS)
{
	physical_cache[slot].valid = 0;
	physical_cache[slot].stream = stream;
	physical_cache_failed_lba = -1;
	physical_cache_failed_end = -1;
	physical_cache_request_slot = slot;
	physical_cache_request_lba = lba;
	physical_cache_request_sectors = sectors;
	physical_cache_request_prefetch = prefetch;
	physical_cache_request_packet_sectors = packet_sectors;
	physical_cache_busy = 1;
	pthread_cond_signal(&physical_cache_cond);
}

static int physical_cache_schedule_prefetch_locked(int stream)
{
	if (physical_cache_busy || physical_stream_lba[stream] < 0) return 0;
	int slot = physical_cache_find(physical_stream_lba[stream], 1, stream);
	if (slot < 0) return 0;
	int connected[PHYSICAL_CACHE_SLOTS] = {};
	connected[slot] = 1;
	int next_lba = physical_cache[slot].start_lba + physical_cache[slot].valid_sectors;
	int next_slot;
	while ((next_slot = physical_cache_find(next_lba, 1, stream)) >= 0)
	{
		connected[next_slot] = 1;
		int end = physical_cache[next_slot].start_lba + physical_cache[next_slot].valid_sectors;
		if (end <= next_lba) break;
		next_lba = end;
	}
	int ahead = next_lba - physical_stream_lba[stream];
	if (physical_stream_coasting[stream]) {
		if (ahead > PHYSICAL_PREFETCH_LOW_SECTORS) return 0;
		physical_stream_coasting[stream] = 0;
	}
	if (ahead >= PHYSICAL_PREFETCH_HIGH_SECTORS) {
		physical_stream_coasting[stream] = 1;
		return 0;
	}
	if (next_lba >= toc.end) return 0;
	int target = -1;
	int first = stream * PHYSICAL_CACHE_SLOTS_PER_STREAM;
	int last = first + PHYSICAL_CACHE_SLOTS_PER_STREAM;
	for (int i = first; i < last; i++)
		if (!physical_cache[i].valid) { target = i; break; }
	if (target < 0) {
		for (int i = first; i < last; i++) {
			if (connected[i]) continue;
			if (target < 0 || physical_cache[i].start_lba < physical_cache[target].start_lba) target = i;
		}
	}
	if (target < 0) return 0;
	physical_cache_queue(target, next_lba, PHYSICAL_CACHE_SECTORS, 1, stream,
		PHYSICAL_PREFETCH_PACKET_SECTORS);
	return 1;
}

static void physical_cd_read_sectors(uint8_t *buffer, int lba, int cnt)
{
	int request_track = physical_cd_track_for_lba(lba);
	int request_stream = (request_track >= 0 && toc.tracks[request_track].type == TT_CDDA) ?
		PHYSICAL_STREAM_CDDA : PHYSICAL_STREAM_DATA;
	int sequential_request = physical_last_read_end[request_stream] == lba;
	physical_last_read_end[request_stream] = lba + cnt;

	if (!physical_cache_thread_started)
	{
		physical_cd_read_sectors_uncached(buffer, lba, cnt, PHYSICAL_PACKET_SECTORS);
		return;
	}

	// A PSX request can straddle two adjacent cache slots. The old path required
	// the entire request to fit in one slot, turning every boundary crossing into
	// a blocking 64-sector refill. Copy each cached portion separately instead.
	while (cnt > 0)
	{
		int track = physical_cd_track_for_lba(lba);
		int stream = (track >= 0 && toc.tracks[track].type == TT_CDDA) ? PHYSICAL_STREAM_CDDA : PHYSICAL_STREAM_DATA;
		pthread_mutex_lock(&physical_cache_mutex);
		if (sequential_request) physical_stream_lba[stream] = lba;
		int slot = physical_cache_find(lba, 1, stream);
		while (slot < 0)
		{
			slot = physical_cache_find(lba, 1, stream);
			if (slot >= 0) break;
			if (!physical_cache_busy && lba >= physical_cache_failed_lba && lba < physical_cache_failed_end)
			{
				// Do not publish a partially read cache window. Return an explicit
				// zero-filled error response and let a later request retry the drive.
				physical_cache_failed_lba = -1;
				physical_cache_failed_end = -1;
				physical_cache_demand_waiting = 0;
				pthread_mutex_unlock(&physical_cache_mutex);
				memset(buffer, 0, cnt * CD_SECTOR_LEN);
				return;
			}
			if (physical_cache_busy) {
				physical_cache_demand_waiting = 1;
				pthread_cond_wait(&physical_cache_cond, &physical_cache_mutex);
				continue;
			}

			int target = physical_cache_choose_demand_victim(lba, stream);
			int fill = sequential_request ? PHYSICAL_CACHE_SECTORS : PHYSICAL_PACKET_SECTORS;
			if (fill < cnt) fill = cnt < PHYSICAL_CACHE_SECTORS ? cnt : PHYSICAL_CACHE_SECTORS;
			physical_cache_queue(target, lba, fill, 0, stream);
		}
		physical_cache_demand_waiting = 0;

		int available = physical_cache[slot].start_lba + physical_cache[slot].valid_sectors - lba;
		int chunk = cnt < available ? cnt : available;
		memcpy(buffer, physical_cache[slot].data +
			(lba - physical_cache[slot].start_lba) * CD_SECTOR_LEN,
			chunk * CD_SECTOR_LEN);

		if (sequential_request) physical_cache_schedule_prefetch_locked(stream);
		pthread_mutex_unlock(&physical_cache_mutex);

		buffer += chunk * CD_SECTOR_LEN;
		lba += chunk;
		cnt -= chunk;
	}
}

int psx_chd_hunksize()
{
	if (toc.chd_f)
		return toc.chd_hunksize;

	return 0;
}


void psx_read_cd(uint8_t *buffer, int lba, int cnt)
{
	//printf("req lba=%d, cnt=%d\n", lba, cnt);
	if (physical_cd_enabled)
	{
		physical_cd_read_sectors(buffer, lba, cnt);
		return;
	}

	while (cnt > 0)
	{
		if (lba < toc.tracks[0].start || !toc.last)
		{
			memset(buffer, 0, CD_SECTOR_LEN);
		}
		else
		{
			memset(buffer, 0xAA, CD_SECTOR_LEN);
			for (int i = 0; i < toc.last; i++)
			{
				if (lba >= toc.tracks[i].start && lba <= toc.tracks[i].end)
				{
					if (!toc.chd_f)
					{
						if (toc.tracks[i].offset)
            {
							FileSeek(&toc.tracks[0].f, toc.tracks[i].offset+((lba-toc.tracks[i].start)*CD_SECTOR_LEN), SEEK_SET);

            }else {
							FileSeek(&toc.tracks[i].f, (lba - toc.tracks[i].start) * CD_SECTOR_LEN, SEEK_SET);
            }
					}
					while (cnt)
					{
            if (toc.tracks[i+1].pregap && lba > (toc.tracks[i+1].start-toc.tracks[i+1].indexes[1]))
            {
              //The TOC is setup so that pregap sectors are actually part of the
              //PREVIOUS track. If the pregap field is set the file doesn't contain
              //this data, so we have to fake it. 
              //Check the next track's pregap and indexes[1] values to determine
              //if we're reading pregap sectors
              

              memset(buffer, 0x0, CD_SECTOR_LEN);
            }
            else if (toc.chd_f)
						{

							// The "fake" 150 sector pregap moves all the LBAs up by 150, so adjust here to read where the core actually wants data from
							int read_lba = lba - toc.tracks[0].indexes[1];
							if (mister_chd_read_sector(toc.chd_f, (read_lba + toc.tracks[i].offset), 0, 0, CD_SECTOR_LEN, buffer, chd_hunkbuf, &chd_hunknum) == CHDERR_NONE)
							{
								if (!toc.tracks[i].type) //CHD requires byteswap of audio data
								{
									for (int swapidx = 0; swapidx < CD_SECTOR_LEN; swapidx += 2)
									{
										uint8_t temp = buffer[swapidx];
										buffer[swapidx] = buffer[swapidx + 1];
										buffer[swapidx + 1] = temp;
									}
								}
							}
							else {
								printf("\x1b[32mPSX: CHD read error: %d\n\x1b[0m", lba);
							}
						}
						else {
							if (toc.tracks[i].offset)
								FileReadAdv(&toc.tracks[0].f, buffer, CD_SECTOR_LEN);
							else
								FileReadAdv(&toc.tracks[i].f, buffer, CD_SECTOR_LEN);
						}
						if ((lba + 1) > toc.tracks[i].end) break;
						buffer += CD_SECTOR_LEN;
						cnt--;
						lba++;
					}
					break;
				}
			}
		}

		buffer += CD_SECTOR_LEN;
		cnt--;
		lba++;
	}
}

#define ROOT_FOLDER_LBA 150 + 22

struct region_info_t
{
	const char* game_id_prefix;
	enum region_t region;
};

const region_info_t region_info_table[]
{
	{ "SCES", region_t::EU },
	{ "SLES", region_t::EU },
	{ "SCUS", region_t::US },
	{ "SLUS", region_t::US },
	{ "SCPM", region_t::JP },
	{ "SLPM", region_t::JP },
	{ "SCPS", region_t::JP },
	{ "SLPS", region_t::JP },
	{ "SIPS", region_t::JP },
	// for demo disks
	{ "PUPX", region_t::US },
	{ "PEPX", region_t::EU },
	{ "PAPX", region_t::JP },
	{ "PCPX", region_t::JP },
	{ "SCZS", region_t::JP },
	{ "SCED", region_t::EU },
	{ "SLED", region_t::EU },
};

struct game_info_t
{
	const char* game_id;
	region_t region;
};

static region_t psx_get_region()
{
	uint8_t buffer[CD_SECTOR_LEN];
	int license_sector = 154;
	psx_read_cd(buffer, license_sector, 1);
	uint8_t* license_start = (uint8_t*)memmem(buffer, CD_SECTOR_LEN, "          Licensed  by          Sony Computer Entertainment ", 60);
	if (license_start) {
		const uint8_t* region_start = license_start + 60;
		if (memcmp(region_start, "Amer  ica ", 10) == 0)
			return region_t::US;
		if (memcmp(region_start, "Inc.", 4) == 0)
			return region_t::JP;
		if (memcmp(region_start, "Euro pe", 7) == 0)
			return region_t::EU;
	}

	return region_t::UNKNOWN;
}

static game_info_t psx_get_game_info()
{
	uint8_t buffer[CD_SECTOR_LEN];

	static char game_id[11];
	memset(game_id, 0, sizeof(game_id));
	enum region_t game_region = UNKNOWN;

	for (int sector = ROOT_FOLDER_LBA; sector < ROOT_FOLDER_LBA + 25; ++sector)
	{
		psx_read_cd(buffer, sector, 1);
		//hexdump(buffer, CD_SECTOR_LEN);
		char* start = nullptr;

		for (const auto& region_info : region_info_table)
		{
			game_region = region_info.region;
			start = (char*)memmem(buffer, CD_SECTOR_LEN, region_info.game_id_prefix, 4);
			if (start) break;
		}

		if (!start) continue;

		const size_t start_pos = start - (char*)buffer;
		char* end = (char*)memmem(start, CD_SECTOR_LEN - start_pos, ";1", 2);

		if (!end) continue;

		size_t size = end - start;

		// file is usually in CCCC_DDD.DD format, normalize to CCCC-DDDDD
		if (size == 11)
		{
			if (start[4] == '_') start[4] = '-';
			if (start[8] == '.')
			{
				start[8] = start[9];
				start[9] = start[10];
				--size;
			}
		}

		const size_t max_length = sizeof(game_id) - 1;
		if (size > max_length) size = max_length;

		return { (const char*)memcpy(game_id, start, size), game_region };
	}

	return { game_id, region_t::UNKNOWN };
}

const char* psx_get_game_id()
{
	return psx_get_game_info().game_id;
}

static void mount_cd(int size, int index)
{
	spi_uio_cmd_cont(UIO_SET_SDINFO);
	spi32_w(size);
	spi32_w(0);
	DisableIO();
	spi_uio_cmd8(UIO_SET_SDSTAT, (1 << index) | 0x80);
	user_io_bufferinvalidate(1);
}

static int load_bios(const char* filename)
{
	int sz = FileLoad(filename, 0, 0);
	if (sz != 512 * 1024) return 0;
	return user_io_file_tx(filename, 0xC0);
}

void psx_mount_cd(int f_index, int s_index, const char *filename)
{
	static char last_dir[1024] = {};
	physical_cd_enabled = 0;
	physical_cd_close();

	int loaded = 0;

	if (strlen(filename))
	{
		if (load_cd_image(filename, &toc) && toc.last)
		{
			int reset = 0;
			game_info_t game_info = psx_get_game_info();
			const char* game_id = game_info.game_id;
			region_t region = psx_get_region();
			if (region == region_t::UNKNOWN)
				region = game_info.region;
			printf("Game ID: %s, region: %s\n", game_id, region_string(region));

			// Write game ID if it's not empty (BIOS check is handled in user_io_write_gameid)
			if (game_id && game_id[0] != '\0')
			{
				user_io_write_gameid(filename, 0, game_id);
			}

			int name_len = strlen(filename);

			if (toc.tracks[0].type) // is first track a data?
			{
				const char *p = strrchr(filename, '/');
				int cur_len = p ? p - filename : 0;
				int old_len = strlen(last_dir);

				int same_game = old_len && (cur_len == old_len) && !strncmp(last_dir, filename, old_len);

				if (!same_game)
				{
					if (!noreset && old_len)
					{
						strcat(last_dir, "/noreset.txt");
						noreset = FileExists(last_dir);
					}
					reset = !noreset;

					strcpy(last_dir, filename);
					char *p = strrchr(last_dir, '/');
					if (p) *p = 0;
					else *last_dir = 0;

					if (reset)
					{
						int bios_loaded = 0;

						// load cd_bios.rom from game directory
						sprintf(buf, "%s/", last_dir);
						p = strrchr(buf, '/');
						if (p)
						{
							strcpy(p + 1, "cd_bios.rom");
							bios_loaded = load_bios(buf);
						}

						// load cd_bios.rom from parent directory
						if (!bios_loaded) {
							strcpy(buf, last_dir);
							p = strrchr(buf, '/');
							if (p)
							{
								strcpy(p + 1, "cd_bios.rom");
								bios_loaded = load_bios(buf);
							}
						}

					}

					if (!(user_io_status_get("[63]"))) psx_mount_save(last_dir);
				}
			}

			uint16_t mask = 0;

			fileTYPE sbi_file = {};
			bool has_sbi_file = false;

			// search for .sbi file in PSX/sbi.zip
			sprintf(buf, "%s/sbi.zip/%s.sbi", HomeDir(), game_id);
			has_sbi_file = (FileOpen(&sbi_file, buf, 1));

			if (!has_sbi_file)
			{
				// search for .sbi file base on image name
				strcpy(buf, filename);
				strcpy((name_len > 4) ? buf + name_len - 4 : buf + name_len, ".sbi");
				has_sbi_file = (FileOpen(&sbi_file, buf, 1));
			}

			if (has_sbi_file)
			{
				printf("Found SBI file: %s\n", buf);
				mask = libCryptMask(&sbi_file);
			}

			process_ss(filename, name_len != 0);
			send_cue_and_metadata(&toc, mask, region, reset);

			user_io_set_index(f_index);

			mount_cd(toc.end*CD_SECTOR_LEN, s_index);
			loaded = 1;
		}
	}

	if (!loaded)
	{
		printf("Unmount CD\n");
		unload_cue(&toc);
		unload_chd(&toc);
		mount_cd(0, s_index);
	}
}

static int physical_cd_mount_inserted_disc()
{
	printf("PSX-CD: reading physical TOC\n");
	fflush(stdout);
	if (!physical_cd_load_toc(&toc)) return 0;

	physical_disc_present = 1;
	physical_cache_invalidate();
	game_info_t game_info = psx_get_game_info();
	region_t region = psx_get_region();
	if (region == region_t::UNKNOWN) region = game_info.region;
	printf("PSX-CD: Physical disc TOC loaded; game ID: %s, detected region: %s\n",
		game_info.game_id && game_info.game_id[0] ? game_info.game_id : "unknown",
		region_string(region));
	fflush(stdout);

	// Original discs carry their protection data physically. The current PSX
	// FPGA interface represents LibCrypt through a mask, so unprotected discs
	// use zero here. Protected-disc subchannel extraction is a follow-up backend
	// extension and does not affect normal data or CD-audio operation.
	printf("PSX-CD: sending TOC to core\n");
	fflush(stdout);
	send_cue_and_metadata(&toc, 0, region, 1);
	printf("PSX-CD: mounting physical CD interface\n");
	fflush(stdout);
	user_io_set_index(1);
	mount_cd(toc.end * CD_SECTOR_LEN, 1);
	printf("PSX-CD: closing virtual lid\n");
	fflush(stdout);
	user_io_status_set("[42]", 0);
	printf("PSX-CD: physical mount complete\n");
	fflush(stdout);
	return 1;
}

void psx_use_physical_cd()
{
	printf("PSX-CD: Use Physical Disc selected\n");
	fflush(stdout);
	physical_cd_enabled = 1;
	physical_poll_timer = 0;
	physical_disc_present = 0;
	physical_cache_start();
	physical_cache_invalidate();
	printf("PSX-CD: v0.50 performance build\n");

	if (!physical_cd_open())
	{
		mount_cd(0, 1);
		user_io_status_set("[42]", 1);
		Info("No physical CD drive found at /dev/sr0", 3000);
		return;
	}

	printf("PSX-CD: opened %s\n", PHYSICAL_CD_DEVICE);
	fflush(stdout);
	if (ioctl(physical_cd_fd, CDROM_SELECT_SPEED, PHYSICAL_CD_SPEED) < 0)
		printf("PSX-CD: drive speed selection not supported: %s\n", strerror(errno));
	else
		printf("PSX-CD: requested maximum drive speed\n");
	fflush(stdout);
	if (!physical_cd_mount_inserted_disc())
	{
		mount_cd(0, 1);
		user_io_status_set("[42]", 1);
		Info("Physical CD mode enabled - insert a disc", 3000);
		return;
	}

	Info("Physical PlayStation disc mounted", 2000);
}

void psx_poll()
{
	if (!physical_cd_enabled && !strcasecmp(user_io_get_core_name(), "CD-PSX"))
		psx_use_physical_cd();

	spi_uio_cmd(UIO_CD_GET);

	if (physical_cd_enabled && (!physical_poll_timer || CheckTimer(physical_poll_timer)))
	{
		physical_poll_timer = GetTimer(500);
		int present = physical_cd_has_disc();
		if (present != physical_disc_present)
		{
			if (!present)
			{
				physical_disc_present = 0;
				physical_cache_invalidate();
				physical_cd_clear_toc();
				mount_cd(0, 1);
				user_io_status_set("[42]", 1);
				printf("PSX: physical disc removed\n");
			}
			else
			{
				printf("PSX: physical disc inserted\n");
				physical_cd_mount_inserted_disc();
			}
		}
	}
}

void psx_reset()
{
	noreset = 0;
}
