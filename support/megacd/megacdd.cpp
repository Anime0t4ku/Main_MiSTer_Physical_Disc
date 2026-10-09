
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>
#include <sys/stat.h>
#include <stdarg.h>
#include <pthread.h>
#include <unistd.h>

#include "megacd.h"
#include "../../cfg.h"
#include "../chd/mister_chd.h"
#include "../physical_disc/physical_disc.h"

cdd_t cdd;

// Physical disc: distance between the requested sector and the one whose subcode the drive returned
#define PHYS_SUB_MAX_SHIFT 16
static int phys_sub_shift = 0;

// Subcode Q from raw P-W (bit 6 of each of the 96 bytes). Returns 1 when its CRC is valid.
static int SubcodeQ(const uint8_t *sub, uint8_t *q)
{
	for (int b = 0; b < 12; b++)
	{
		uint8_t v = 0;
		for (int bit = 0; bit < 8; bit++) v = (uint8_t)((v << 1) | ((sub[b * 8 + bit] >> 6) & 1));
		q[b] = v;
	}

	uint16_t crc = 0;
	for (int i = 0; i < 10; i++)
	{
		crc ^= (uint16_t)q[i] << 8;
		for (int b = 0; b < 8; b++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
	}
	return (uint16_t)~crc == (((uint16_t)q[10] << 8) | q[11]);
}

// LBA of the sector this subcode belongs to, from the absolute time in a valid mode 1 Q, or -1.
static int SubcodeLBA(const uint8_t *sub)
{
	uint8_t q[12];
	if (!SubcodeQ(sub, q) || (q[0] & 0x0F) != 1) return -1;

	int m = (q[7] >> 4) * 10 + (q[7] & 0x0F);
	int s = (q[8] >> 4) * 10 + (q[8] & 0x0F);
	int f = (q[9] >> 4) * 10 + (q[9] & 0x0F);
	return m * 60 * 75 + s * 75 + f - 150;
}

// MEGACD_SUBCODE_LOG=1 in MiSTer.ini: once per second of playback, write a summary of the
// subcode sent to the core, to check CD+G discs. The core gets raw P-W, one byte per symbol
// (bit 7 = P, bit 6 = Q, bits 5..0 = R..W), 4 packs of 24 symbols per sector.
// The main loop only fills a memory buffer: a separate thread writes it to the SD card, so a
// slow SD write never delays the sectors sent to the core.
#define SUBCODE_LOG_DIR    "/media/fat/logs"
#define SUBCODE_LOG_PATH   SUBCODE_LOG_DIR "/megacd_subcode.log"
#define SUBCODE_LOG_WINDOW 75
#define DRIVE_STATS_PATH   "/tmp/physical_disc_stats.log"
#define SUBCODE_RAW_PATH   SUBCODE_LOG_DIR "/megacd_subcode_raw.bin"
#define SUBCODE_RAW_COUNT  22500

#define SUBCODE_LOG_BUFSIZE 32768

static FILE *sublog_f = NULL;
static int sublog_phys = 0;

static pthread_t sublog_thread;
static pthread_mutex_t sublog_lock = PTHREAD_MUTEX_INITIALIZER;
static int sublog_stop = 0;
static char sublog_buf[SUBCODE_LOG_BUFSIZE];
static int sublog_len = 0;
static int sublog_dropped = 0;

// Raw copy of 22500 consecutive sectors (5 minutes) of subcode from the first CD+G pack, 96 bytes each,
// to check the R-W data offline. Filled by the main loop, written once by the writer thread.
static uint8_t sublog_raw[SUBCODE_RAW_COUNT][96];
static int sublog_raw_count = 0;
static int sublog_raw_written = 0;

static void SubcodeLogPrintf(const char *fmt, ...)
{
	char line[512];
	va_list args;
	va_start(args, fmt);
	int n = vsnprintf(line, sizeof(line), fmt, args);
	va_end(args);
	if (n <= 0) return;
	if (n >= (int)sizeof(line)) n = sizeof(line) - 1;

	pthread_mutex_lock(&sublog_lock);
	if (sublog_len + n <= SUBCODE_LOG_BUFSIZE)
	{
		memcpy(sublog_buf + sublog_len, line, n);
		sublog_len += n;
	}
	else
	{
		sublog_dropped += n;
	}
	pthread_mutex_unlock(&sublog_lock);
}

// Writes the buffered text to the SD card. Only called from the writer thread, or after it has stopped.
static void SubcodeLogWrite()
{
	static char out[SUBCODE_LOG_BUFSIZE];

	pthread_mutex_lock(&sublog_lock);
	int len = sublog_len;
	int dropped = sublog_dropped;
	memcpy(out, sublog_buf, len);
	sublog_len = 0;
	sublog_dropped = 0;
	pthread_mutex_unlock(&sublog_lock);

	if (len) fwrite(out, 1, len, sublog_f);
	if (dropped) fprintf(sublog_f, "(log buffer full, %d bytes lost)\n", dropped);
	if (len || dropped) fflush(sublog_f);
}

// Writes the raw subcode copy once it is complete. Only called from the writer thread, or after it has stopped.
static void SubcodeLogWriteRaw()
{
	if (sublog_raw_written || __atomic_load_n(&sublog_raw_count, __ATOMIC_ACQUIRE) < SUBCODE_RAW_COUNT) return;

	FILE *raw = fopen(SUBCODE_RAW_PATH, "wb");
	if (raw)
	{
		fwrite(sublog_raw, 96, SUBCODE_RAW_COUNT, raw);
		fclose(raw);
		SubcodeLogPrintf("raw subcode of %d sectors written to %s\n", SUBCODE_RAW_COUNT, SUBCODE_RAW_PATH);
	}
	sublog_raw_written = 1;
}

static void *SubcodeLogThread(void *)
{
	int ticks = 0;
	while (!__atomic_load_n(&sublog_stop, __ATOMIC_ACQUIRE))
	{
		usleep(200 * 1000);
		if (++ticks % 5) continue;

		// Physical disc: every 5 seconds also copy the drive read statistics (cache misses,
		// sectors read as zeros), which the drive thread rewrites every 5 seconds
		FILE *stats = (sublog_phys && !(ticks % 25)) ? fopen(DRIVE_STATS_PATH, "r") : NULL;
		if (stats)
		{
			char line[256];
			while (fgets(line, sizeof(line), stats)) SubcodeLogPrintf("  drive: %s", line);
			fclose(stats);
		}

		SubcodeLogWriteRaw();
		SubcodeLogWrite();
	}
	return NULL;
}

static struct
{
	int first_lba, last_lba;
	int sent, missing, q_ok;
	int prev_lba, jumps, jump_from, jump_to;
	int first_missing, first_bad_q;
	int cdg_packs, empty_packs, other_packs;
	int have_raw, have_q;
	uint8_t raw[96];
	uint8_t q[12];
} sublog;

static void SubcodeLogFlush()
{
	if (!sublog_f || !(sublog.sent + sublog.missing)) return;

	SubcodeLogPrintf("lba %d-%d: sent %d, missing %d, Q crc ok %d, packs: graphics %d, empty %d, other %d\n",
		sublog.first_lba, sublog.last_lba, sublog.sent, sublog.missing, sublog.q_ok,
		sublog.cdg_packs, sublog.empty_packs, sublog.other_packs);

	if (sublog.missing || sublog.q_ok < sublog.sent)
	{
		SubcodeLogPrintf("  first missing lba %d, first Q crc error lba %d\n", sublog.missing ? sublog.first_missing : -1, (sublog.q_ok < sublog.sent) ? sublog.first_bad_q : -1);
	}

	if (sublog.jumps)
	{
		SubcodeLogPrintf("  not in sequence: %d time(s), first %d -> %d\n", sublog.jumps, sublog.jump_from, sublog.jump_to);
	}

	if (sublog.have_q)
	{
		const uint8_t *q = sublog.q;
		SubcodeLogPrintf("  last Q: ctrl/adr %02X track %02X index %02X rel %02X:%02X:%02X abs %02X:%02X:%02X\n",
			q[0], q[1], q[2], q[3], q[4], q[5], q[7], q[8], q[9]);
	}

	if (sublog.have_raw)
	{
		char hex[96 * 2 + 4 + 1];
		int pos = 0;
		for (int i = 0; i < 96; i++) pos += sprintf(hex + pos, "%s%02X", (i % 24) ? "" : " ", sublog.raw[i]);
		SubcodeLogPrintf("  raw lba %d:%s\n", sublog.first_lba, hex);
	}

	int prev_lba = sublog.prev_lba;
	memset(&sublog, 0, sizeof(sublog));
	sublog.prev_lba = prev_lba;
}

static void SubcodeLogOpen(const char *source, int phys)
{
	if (sublog_f || !cfg.megacd_subcode_log) return;

	sublog_phys = phys;

	mkdir(SUBCODE_LOG_DIR, 0777);
	sublog_f = fopen(SUBCODE_LOG_PATH, "w");
	if (!sublog_f)
	{
		printf("\x1b[32mMCD: cannot write %s\n\x1b[0m", SUBCODE_LOG_PATH);
		return;
	}

	memset(&sublog, 0, sizeof(sublog));
	sublog.prev_lba = -1;
	fprintf(sublog_f, "MegaCD subcode log, source: %s\n", source);
	fprintf(sublog_f, "graphics = CD+G/CD+EG packs (mode/item 09 or 0A), empty = 00, Q crc ok = position data read correctly\n");
	fprintf(sublog_f, "not in sequence = subcode lba not following the previous one (seek, or a sector skipped or repeated)\n");
	fflush(sublog_f);

	sublog_len = 0;
	sublog_dropped = 0;
	sublog_raw_count = 0;
	sublog_raw_written = 0;
	__atomic_store_n(&sublog_stop, 0, __ATOMIC_RELEASE);
	if (pthread_create(&sublog_thread, NULL, SubcodeLogThread, NULL))
	{
		printf("\x1b[32mMCD: cannot start the subcode log thread\n\x1b[0m");
		fclose(sublog_f);
		sublog_f = NULL;
		return;
	}
	printf("\x1b[32mMCD: subcode log in %s\n\x1b[0m", SUBCODE_LOG_PATH);
}

static void SubcodeLogClose()
{
	if (!sublog_f) return;

	SubcodeLogFlush();
	__atomic_store_n(&sublog_stop, 1, __ATOMIC_RELEASE);
	pthread_join(sublog_thread, NULL);
	SubcodeLogWriteRaw();
	SubcodeLogWrite();
	fclose(sublog_f);
	sublog_f = NULL;
}

static void SubcodeLogSector(int lba, const uint8_t *sub)
{
	if (!sublog_f) return;

	if (!(sublog.sent + sublog.missing)) sublog.first_lba = lba;
	sublog.last_lba = lba;

	if (sublog.prev_lba >= 0 && lba != sublog.prev_lba + 1)
	{
		if (!sublog.jumps)
		{
			sublog.jump_from = sublog.prev_lba;
			sublog.jump_to = lba;
		}
		sublog.jumps++;
	}
	sublog.prev_lba = lba;

	if (!sub)
	{
		if (!sublog.missing) sublog.first_missing = lba;
		sublog.missing++;
	}
	else
	{
		sublog.sent++;

		int raw_count = sublog_raw_count;
		if (raw_count < SUBCODE_RAW_COUNT &&
			(raw_count || (sub[0] & 0x3F) == 0x09 || (sub[24] & 0x3F) == 0x09 || (sub[48] & 0x3F) == 0x09 || (sub[72] & 0x3F) == 0x09))
		{
			memcpy(sublog_raw[raw_count], sub, 96);
			__atomic_store_n(&sublog_raw_count, raw_count + 1, __ATOMIC_RELEASE);
		}
		if (!sublog.have_raw)
		{
			memcpy(sublog.raw, sub, 96);
			sublog.have_raw = 1;
		}

		uint8_t q[12];
		if (SubcodeQ(sub, q))
		{
			sublog.q_ok++;
			memcpy(sublog.q, q, 12);
			sublog.have_q = 1;
		}
		else if (sublog.q_ok + 1 == sublog.sent)
		{
			// first Q error of this window: all sectors before it were good
			sublog.first_bad_q = lba;
		}

		// Symbol 0 of a pack (mode/item) is not moved by the R-W interleave, so it can be read in place.
		for (int p = 0; p < 4; p++)
		{
			uint8_t mode_item = sub[p * 24] & 0x3F;
			if (mode_item == 0x09 || mode_item == 0x0A) sublog.cdg_packs++;
			else if (!mode_item) sublog.empty_packs++;
			else sublog.other_packs++;
		}
	}

	if (sublog.sent + sublog.missing >= SUBCODE_LOG_WINDOW) SubcodeLogFlush();
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
	phys_sub_shift = 0;

	const char *ext = filename+strlen(filename)-4;
	if (!strcmp(filename, PHYSICAL_DISC_SENTINEL))
	{
		if (physical_disc_open(NULL) || physical_disc_load_toc(&this->toc))
		{



			physical_disc_close();
			printf("\x1b[32mMCD: no readable physical disc\n\x1b[0m");
			return (-1);
		}
	} else if (!strncasecmp(".cue", ext, 4))
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

	if (this->toc.phys)
	{


	}
	else if (this->toc.chd_f)
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

		SubcodeLogOpen(this->toc.phys ? "physical disc" : this->toc.chd_f ? "CHD" : this->toc.sub.opened() ? "CUE + .sub" : "CUE, no subcode", this->toc.phys);

		return 1;
	}

	return 0;
}







int cdd_t::SwapPhys()
{
	toc_t nt = {};
	if (physical_disc_current_toc(&nt) || !nt.last) return 0;   
	phys_sub_shift = 0;
	this->toc = nt;                                       
	this->sectorSize = this->toc.tracks[0].sector_size ? this->toc.tracks[0].sector_size : 2352;
	this->toc.tracks[this->toc.last].start = this->toc.end;   
	this->loaded = 1;



	this->index = 0;
	this->lba = 0;
	this->scanOffset = 0;
	this->audioLength = 0;
	this->audioOffset = 0;
	this->chd_audio_read_lba = 0;
	return 1;
}

void cdd_t::Unload()
{
	SubcodeLogClose();

	if (this->loaded)
	{
		if (this->toc.phys)
		{
			physical_disc_close();
		}

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

			if (this->toc.tracks[this->index].f.opened())
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

		if (this->toc.sub.opened()) FileSeek(&this->toc.sub, this->lba * 96, SEEK_SET);

		if (this->toc.phys)
		{
			physical_disc_seek_hint(this->lba);
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

	if (this->toc.phys)
	{


		physical_disc_seek_hint(lba);
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

		if (this->toc.phys)
		{
			physical_disc_read_data2048(this->lba, buf);
		}
		else if (this->toc.chd_f)
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

	if (this->toc.phys)
	{


		for (int i = 0; i < this->audioLength / 2352; i++)
		{
			physical_disc_read_sector(this->chd_audio_read_lba + i, buf + 2352 * i, NULL);
		}

		if ((this->audioLength / 2352) > 1)
		{
			this->chd_audio_read_lba++;
		}
	}
	else if (this->toc.chd_f)
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
	if (this->toc.phys)
	{
		uint8_t rawsec[2352];

		// The drive can return the subcode of a sector a few frames away, and the distance changes
		// between reads: sent as is, some subcode would be repeated or skipped (broken CD+G graphics).
		// The absolute time in Q tells which sector the subcode belongs to: read the one that matches.
		int want = this->chd_audio_read_lba;
		int at = want + phys_sub_shift;
		err = -1;
		for (int tries = 0; tries < 3; tries++)
		{
			uint8_t cand[96];
			if (!physical_disc_read_sector_sub(at, rawsec, cand)) break;   // keep the previous read, if any
			memcpy(subc, cand, 96);
			err = 0;

			int sub_lba = SubcodeLBA(subc);
			if (sub_lba < 0) break;              // no position in this Q: keep the current shift
			if (sub_lba == want)
			{
				phys_sub_shift = at - want;
				break;
			}

			int next = at + want - sub_lba;
			if (next - want > PHYS_SUB_MAX_SHIFT || want - next > PHYS_SUB_MAX_SHIFT) break;
			at = next;
		}

		//The drive returns raw P-W subchannel (READ CD sub-channel 001b), already in the layout the core expects, same as CHD RW_RAW
		if (!err) memcpy(buf, subc, 96);
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
	uint16_t buf[98 / 2];

	int err = ReadSubcode(buf);
	SubcodeLogSector(this->chd_audio_read_lba, err ? NULL : (const uint8_t*)buf);

	if (!err && SendData)
		return SendData((uint8_t*)buf, 98, MCD_SUB_IO_INDEX);

	return 0;
}





