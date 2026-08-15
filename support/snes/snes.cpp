
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <limits.h>
#include <glob.h>
#include <pthread.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <stdarg.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/mount.h>
#include <sys/ioctl.h>
#include <linux/cdrom.h>

#include "../../file_io.h"
#include "../../user_io.h"
#include "../../spi.h"
#include "../../hardware.h"
#include "../../menu.h"

#define SNESMSU_DIAG_LOG "/media/fat/snesmsu1_diagnostic.log"

static void snesmsu_diag(const char *fmt, ...)
{
	FILE *f = fopen(SNESMSU_DIAG_LOG, "a");
	if (!f) return;

	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	struct tm tmv;
	localtime_r(&ts.tv_sec, &tmv);
	fprintf(f, "%04d-%02d-%02d %02d:%02d:%02d.%03ld pid=%d ",
		tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
		tmv.tm_hour, tmv.tm_min, tmv.tm_sec, ts.tv_nsec / 1000000,
		(int)getpid());

	va_list ap;
	va_start(ap, fmt);
	vfprintf(f, fmt, ap);
	va_end(ap);
	fputc('\n', f);
	fclose(f);
}


static uint8_t hdr[512];

enum HeaderField {
	CartName = 0x00,
	Mapper = 0x15,
	RomType = 0x16,
	RomSize = 0x17,
	RamSize = 0x18,
	CartRegion = 0x19,
	Company = 0x1a,
	Version = 0x1b,
	Complement = 0x1c,  //inverse checksum
	Checksum = 0x1e,
	ResetVector = 0x3c,
};

static uint32_t score_header(const uint8_t *data, uint32_t size, uint32_t addr)
{
	if (size < addr + 64) return 0;  //image too small to contain header at this location?
	int score = 0;

	uint16_t resetvector = data[addr + ResetVector] | (data[addr + ResetVector + 1] << 8);
	uint16_t checksum = data[addr + Checksum] | (data[addr + Checksum + 1] << 8);
	uint16_t complement = data[addr + Complement] | (data[addr + Complement + 1] << 8);

	uint8_t resetop = data[(addr & ~0x7fff) | (resetvector & 0x7fff)];  //first opcode executed upon reset
	uint8_t mapper = data[addr + Mapper] & ~0x10;                      //mask off irrelevent FastROM-capable bit

																	   //$00:[0000-7fff] contains uninitialized RAM and MMIO.
																	   //reset vector must point to ROM at $00:[8000-ffff] to be considered valid.
	if (resetvector < 0x8000) return 0;

	//some images duplicate the header in multiple locations, and others have completely
	//invalid header information that cannot be relied upon.
	//below code will analyze the first opcode executed at the specified reset vector to
	//determine the probability that this is the correct header.

	//most likely opcodes
	if (resetop == 0x78  //sei
		|| resetop == 0x18  //clc (clc; xce)
		|| resetop == 0x38  //sec (sec; xce)
		|| resetop == 0x9c  //stz $nnnn (stz $4200)
		|| resetop == 0x4c  //jmp $nnnn
		|| resetop == 0x5c  //jml $nnnnnn
		) score += 8;

	//plausible opcodes
	if (resetop == 0xc2  //rep #$nn
		|| resetop == 0xe2  //sep #$nn
		|| resetop == 0xad  //lda $nnnn
		|| resetop == 0xae  //ldx $nnnn
		|| resetop == 0xac  //ldy $nnnn
		|| resetop == 0xaf  //lda $nnnnnn
		|| resetop == 0xa9  //lda #$nn
		|| resetop == 0xa2  //ldx #$nn
		|| resetop == 0xa0  //ldy #$nn
		|| resetop == 0x20  //jsr $nnnn
		|| resetop == 0x22  //jsl $nnnnnn
		) score += 4;

	//implausible opcodes
	if (resetop == 0x40  //rti
		|| resetop == 0x60  //rts
		|| resetop == 0x6b  //rtl
		|| resetop == 0xcd  //cmp $nnnn
		|| resetop == 0xec  //cpx $nnnn
		|| resetop == 0xcc  //cpy $nnnn
		) score -= 4;

	//least likely opcodes
	if (resetop == 0x00  //brk #$nn
		|| resetop == 0x02  //cop #$nn
		|| resetop == 0xdb  //stp
		|| resetop == 0x42  //wdm
		|| resetop == 0xff  //sbc $nnnnnn,x
		) score -= 8;

	//at times, both the header and reset vector's first opcode will match ...
	//fallback and rely on info validity in these cases to determine more likely header.

	//a valid checksum is the biggest indicator of a valid header.
	if ((checksum + complement) == 0xffff && (checksum != 0) && (complement != 0)) score += 4;

	if (addr == 0x007fc0 && mapper == 0x20) score += 2;  //0x20 is usually LoROM
	if (addr == 0x00ffc0 && mapper == 0x21) score += 2;  //0x21 is usually HiROM
	if (addr == 0x007fc0 && mapper == 0x22) score += 2;  //0x22 is usually SDD1
	if (addr == 0x40ffc0 && mapper == 0x25) score += 2;  //0x25 is usually ExHiROM

	if (data[addr + Company] == 0x33) score += 2;        //0x33 indicates extended header
	if (data[addr + RomType] < 0x08) score++;
	if (data[addr + RomSize] < 0x10) score++;
	if (data[addr + RamSize] < 0x08) score++;
	if (data[addr + CartRegion] < 14) score++;

	if (score < 0) score = 0;
	return score;
}

static uint32_t find_header(const uint8_t *data, uint32_t size)
{
	uint32_t score_lo = score_header(data, size, 0x007fc0);
	uint32_t score_hi = score_header(data, size, 0x00ffc0);
	uint32_t score_ex = score_header(data, size, 0x40ffc0);
	if (score_ex) score_ex += 4;  //favor ExHiROM on images > 32mbits

	if (score_lo >= score_hi && score_lo >= score_ex)
	{
		return score_lo ? 0x007fc0 : 0;
	}
	else if (score_hi >= score_ex)
	{
		return score_hi ? 0x00ffc0 : 0;
	}

	return score_ex ? 0x40ffc0 : 0;
}

const char snes_cc92_header[] = {
	0x00, 0x08, 0x22, 0x02, 0x1C, 0x00, 0x10, 0x00, 0x08, 0x65, 0x80, 0x84, 0x20, 0x00, 0x22, 0x25, 
	0x00, 0x83, 0x0C, 0x80, 0x10, 0x00, 0x00, 0xA0, 0x80, 0x01, 0x80, 0x80, 0x00, 0x01, 0x02, 0x2D
};
const char snes_pf94_10k_header[] = {
	0xC9, 0x80, 0x80, 0x44, 0x15, 0x00, 0x62, 0x09, 0x29, 0xA0, 0x52, 0x70, 0x50, 0x12, 0x05, 0x35,
	0x31, 0x63, 0xC0, 0x22, 0x01, 0x80, 0xC2, 0x3A, 0x6C, 0xB0, 0xE8, 0x4A, 0x11, 0x20, 0xC0, 0xF8
};
const char snes_pf94_1m_header[] = {
	0x50, 0x52, 0x45, 0x48, 0x49, 0x53, 0x54, 0x4F, 0x52, 0x49, 0x4B, 0x20, 0x4D, 0x41, 0x4E, 0x20,
	0x20, 0x20, 0x20, 0x20, 0x20, 0x30, 0x00, 0x0A, 0x00, 0x01, 0x33, 0x00, 0xFF, 0xFF, 0x00, 0x00,
	0xFF, 0xFF, 0xFF, 0xFF, 0x2B, 0x80, 0x2B, 0x80, 0x2B, 0x80, 0xFE, 0x91, 0x2B, 0x80, 0xA4, 0xF7,
	0xFF, 0xFF, 0xFF, 0xFF, 0x2B, 0x80, 0x2B, 0x80, 0x2B, 0x80, 0x75, 0xF7, 0x00, 0x80, 0xA4, 0xF7
};

uint8_t* snes_get_header(fileTYPE *f)
{
	memset(hdr, 0, sizeof(hdr));
	uint32_t size = f->size;
	uint8_t *prebuf = (uint8_t*)malloc(size);
	if (prebuf)
	{
		FileSeekLBA(f, 0);
		if (FileReadAdv(f, prebuf, size))
		{
			uint8_t *buf = prebuf;

			if (size & 512)
			{
				buf += 512;
				size -= 512;
			}

			*(uint32_t*)(&hdr[8]) = size;

			uint32_t addr = find_header(buf, size);

			bool is_bsx_bios = false;
			if (!memcmp(buf+0x7FC0, "Satellaview BS-X     ", 21)) {
				is_bsx_bios = true;
			}

			bool is_sufami_bios = false, is_sufami_base = false, is_sufami_turbo = false;
			if (!memcmp(buf, "BANDAI SFC-ADX", 14)) {
				uint32_t offs = 0;
				while (offs < size) {
					if (!memcmp(buf + offs, "BANDAI SFC-ADX", 14)) {
						if (!memcmp(buf + offs + 0x10, "SFC-ADX BACKUP", 14)) {
							is_sufami_bios = true;
						}
						else {
							if (!is_sufami_base) addr = offs;
							is_sufami_turbo = is_sufami_base;
							is_sufami_base = true;
						}
						offs += 1024 * 1024;
					}
				}
			}

			bool is_cc92 = false, is_pf94 = false;
			if (!memcmp(buf + 0x7FC0, snes_cc92_header, 32)) {
				is_cc92 = true;
			}
			if (!memcmp(buf + 0x7FC0, snes_pf94_10k_header, 32) ||
				!memcmp(buf + 0x7FC0, snes_pf94_1m_header, 64)) {
				is_pf94 = true;
			}

			if (addr)
			{
				uint8_t ramsz = buf[addr + RamSize];
				if (ramsz >= 0x09) ramsz = 0;

				//re-calc rom size
				uint8_t romsz = 15;
				size--;
				if (!(size & 0xFF000000))
				{
					while (!(size & 0x1000000))
					{
						romsz--;
						size <<= 1;
					}
				}

				bool has_bsx_slot = false;
				if (buf[addr - 14] == 'Z' && buf[addr - 11] == 'J' &&
					((buf[addr - 13] >= 'A' && buf[addr - 13] <= 'Z') || (buf[addr - 13] >= '0' && buf[addr - 13] <= '9')) &&
					(buf[addr + Company] == 0x33 || (buf[addr - 10] == 0x00 && buf[addr - 4] == 0x00)) ) {
					has_bsx_slot = true;
				}

				//Rom type: 0-Low, 1-High, 2-ExHigh, 3-SpecialLoRom
				hdr[1] = (addr == 0x00ffc0) ? 1 :
						 (addr == 0x40ffc0) ? 2 :
						 has_bsx_slot ? 3 :
						 0;

				//BSX 3
				if (is_bsx_bios) {
					hdr[1] = 0x30;
				}
				//Sufami Turbo type 2
				else if (is_sufami_base) {
					hdr[1] = 0x20 | (is_sufami_turbo ? 8 : 0) | (is_sufami_bios ? 4 : 0);
					const uint8_t rom_sz_tbl[9] = { 0,7,8,9,9,10,10,10,10 };
					const uint8_t ram_sz_tbl[5] = { 0,1,2,3,3 };
					romsz = buf[addr + 0x36] >= 8 ? rom_sz_tbl[8] : rom_sz_tbl[buf[addr + 0x36] & 0x0F];
					ramsz = buf[addr + 0x37] >= 4 ? ram_sz_tbl[4] : ram_sz_tbl[buf[addr + 0x37] & 0x07];
				}
				//Campus Challenge '92, type E (DSP1A)
				else if (is_cc92) {
					hdr[1] = 0xE4;
					ramsz = 3;
				}
				//PowerFest '94, type F (DSP1A)
				else if (is_pf94) {
					hdr[1] = 0xF4;
					ramsz = 3;
				}
				else {

					//DSPn types 8..B, OBC1 type C
					if (buf[addr + Mapper] == 0x20 && buf[addr + RomType] == 0x03)
					{	//DSP1
						hdr[1] |= 0x84;
					}
					else if (buf[addr + Mapper] == 0x21 && buf[addr + RomType] == 0x03)
					{	//DSP1B
						hdr[1] |= 0x80;
					}
					else if (buf[addr + Mapper] == 0x30 && buf[addr + RomType] == 0x05 && buf[addr + Company] != 0xb2)
					{	//DSP1B
						hdr[1] |= 0x80;
					}
					else if (buf[addr + Mapper] == 0x31 && (buf[addr + RomType] == 0x03 || buf[addr + RomType] == 0x05))
					{	//DSP1B
						hdr[1] |= 0x80;
					}
					else if (buf[addr + Mapper] == 0x20 && buf[addr + RomType] == 0x05)
					{	//DSP2
						hdr[1] |= 0x90;
					}
					else if (buf[addr + Mapper] == 0x30 && buf[addr + RomType] == 0x05 && buf[addr + Company] == 0xb2)
					{	//DSP3
						hdr[1] |= 0xA0;
					}
					else if (buf[addr + Mapper] == 0x30 && buf[addr + RomType] == 0x03)
					{	//DSP4
						hdr[1] |= 0xB0;
					}
					else if (buf[addr + Mapper] == 0x30 && buf[addr + RomType] == 0xf6)
					{	//ST010
						hdr[1] |= 0x88;
						ramsz = 1;
						if (buf[addr + RomSize] < 10) hdr[1] |= 0x20; // ST011
					}
					else if (buf[addr + Mapper] == 0x30 && buf[addr + RomType] == 0x25)
					{	//OBC1
						hdr[1] |= 0xC0;
					}

					if (buf[addr + Mapper] == 0x3a && (buf[addr + RomType] == 0xf5 || buf[addr + RomType] == 0xf9)) {
						//SPC7110
						hdr[1] |= 0xD0;
						if (buf[addr + RomType] == 0xf9) hdr[1] |= 0x08; // with RTC
					}

					if (buf[addr + Mapper] == 0x35 && buf[addr + RomType] == 0x55)
					{
						//S-RTC (+ExHigh)
						hdr[1] |= 0x08;
					}

					//CX4 4
					if (buf[addr + Mapper] == 0x20 && buf[addr + RomType] == 0xf3)
					{
						hdr[1] |= 0x40;
					}

					//SDD1 5
					if (buf[addr + Mapper] == 0x32 && (buf[addr + RomType] == 0x43 || buf[addr + RomType] == 0x45))
					{
						if (romsz < 14) hdr[1] |= 0x50; // except Star Ocean un-SDD1
					}

					//SA1 6
					if (buf[addr + Mapper] == 0x23 && (buf[addr + RomType] == 0x32 || buf[addr + RomType] == 0x34 || buf[addr + RomType] == 0x35))
					{
						hdr[1] |= 0x60;
					}

					//GSU 7
					if (buf[addr + Mapper] == 0x20 && (buf[addr + RomType] == 0x13 || buf[addr + RomType] == 0x14 || buf[addr + RomType] == 0x15 || buf[addr + RomType] == 0x1a))
					{
						ramsz = buf[addr - 3];
						if (ramsz == 0xFF) ramsz = 5; //StarFox
						if (ramsz > 6) ramsz = 6;
						hdr[1] |= 0x70;
					}

					//1 - reserved for other mappers.
				}

				hdr[2] = 0;

				//PAL Regions
				if (((buf[addr + CartRegion] >= 0x02 && buf[addr + CartRegion] <= 0x0C) || buf[addr + CartRegion] == 0x11) && !is_sufami_base && !is_cc92 && !is_pf94)
				{
					hdr[3] |= 1;
				}

				hdr[0] = (ramsz << 4) | romsz;
				printf("Size from header: 0x%X, calculated size: 0x%X\n", buf[addr + RomSize], romsz);
			}
			*(uint32_t*)(&hdr[4]) = addr;
		}
		FileSeekLBA(f, 0);
		free(prebuf);
	}
	return hdr;
}

void snes_patch_bs_header(fileTYPE *f, uint8_t *buf)
{
	if ((f->offset == 0x008000 && (buf[0xFD8] == 0x20 || buf[0xFD8] == 0x30)) ||
		(f->offset == 0x010000 && (buf[0xFD8] == 0x21 || buf[0xFD8] == 0x31)))
	{
		if (buf[0xFD0] == 0xF0 || (buf[0xFD1] == 0xFF && buf[0xFD2] == 0xFF && buf[0xFD3] == 0xFF))
		{
			printf("SNES: Patch bad BS header: offset %04X, bad value %02X %02X %02X %02X\n", 0x7FD0 | (f->offset == 0x008000 ? 0x0000 : 0x8000), buf[0xFD0], buf[0xFD1], buf[0xFD2], buf[0xFD3]);
			buf[0xFD3] = 0x00;
			buf[0xFD2] = 0x00;
			buf[0xFD1] = 0x00;
			buf[0xFD0] = f->size <= 256 * 1024 ? 0x03 :
						 f->size <= 512 * 1024 ? 0x0F :
						 0xFF;
		}

		if (buf[0xFD5] >= 0x80)
		{
			printf("SNES: Patch bad BS header: offset %04X, bad value %02X %02X\n", 0x7FD4 | (f->offset == 0x008000 ? 0x0000 : 0x8000), buf[0xFD4], buf[0xFD5]);
			buf[0xFD5] = 0xFF;
			buf[0xFD4] = 0xFF;
		}

		if (buf[0xFDA] != 0x33)
		{
			printf("SNES: Patch bad BS header: offset %04X, bad value %02X\n", 0x7FDA | (f->offset == 0x008000 ? 0x0000 : 0x8000), buf[0xFDA]);
			buf[0xFDA] = 0x33;
		}
	}
}

static uint32_t snes_mirror(uint32_t addr, uint32_t size)
{
	if (!size) return 0;
	uint32_t base = 0;
	// Start mask at the highest power-of-2 >= size so it covers
	// the full address range regardless of ROM size.
	uint32_t mask = 1u;
	while (mask < size) mask <<= 1;
	while (addr >= size)
	{
		while (mask && !(addr & mask)) mask >>= 1;
		if (!mask) return addr % size; // fallback: should not occur
		addr -= mask;
		if (size > mask)
		{
			size -= mask;
			base += mask;
		}
	mask >>= 1;
	}
	return base + addr;
}

static uint32_t next_pow2(uint32_t v)
{
	if (!v) return 1;
	v--;
	v |= v >> 1;
	v |= v >> 2;
	v |= v >> 4;
	v |= v >> 8;
	v |= v >> 16;
	return v + 1;
}

uint8_t* snes_get_mirrored_rom(fileTYPE *f, uint32_t *out_size)
{
	uint32_t size = f->size;
	uint8_t *rom = (uint8_t*)malloc(size);
	if (!rom) { printf("SNES: malloc(%u) failed\n", size); return NULL; }

	FileSeekLBA(f, 0);
	if (!FileReadAdv(f, rom, size)) {
		printf("SNES: read failed\n");
		free(rom);
		return NULL;
	}

	// Detect and strip 512-byte copier header (same heuristic as snes_get_header)
	bool has_header = (size & 512) != 0;
	if (has_header) size -= 512;

	uint32_t padded = next_pow2(size);
	if (padded == size) {
		printf("SNES: ROM size is a power of 2 (%u bytes), no mirroring\n", size);
		if (has_header) memmove(rom, rom + 512, size);
		if (out_size) *out_size = size;
		return rom;
	}

	printf("SNES: mirroring %u (0x%X) -> %u (0x%X) bytes\n", size, size, padded, padded);
	uint8_t *data = has_header ? rom + 512 : rom;
	uint8_t *mirrored = (uint8_t*)malloc(padded);
	if (!mirrored) {
		printf("SNES: malloc(%u) for mirrored buffer failed\n", padded);
		free(rom);
		return NULL;
	}

	memcpy(mirrored, data, size);
	uint32_t pos = size;
	while (pos < padded) {
		uint32_t src = snes_mirror(pos, size);
		uint32_t run = size - src;
		if (run > padded - pos) run = padded - pos;
		memcpy(mirrored + pos, data + src, run);
		pos += run;
	}

	free(rom);
	if (out_size) *out_size = padded;
	return mirrored;
}

////////////// MSU /////////////

#define MSU_CD_SET               1
#define MSU_AUDIO_TRACK_MOUNTED  2
#define MSU_DATA_BASE            3

static char snes_romFileName[1024] = {};
static char SelectedPath[1024] = {};
static uint8_t buf[1024];
static char has_cd = 0;
static fileTYPE f_audio = {};


#define CD_SNES_BUFFER_SIZE (2 * 1024 * 1024)
#define CD_SNES_PREFILL_SIZE (512 * 1024)

struct cd_snes_stream_t
{
	pthread_t thread;
	pthread_mutex_t lock;
	pthread_cond_t cond;
	uint8_t *buffer;
	int fd;
	uint64_t size;
	uint64_t read_pos;
	uint64_t write_pos;
	uint32_t generation;
	int running;
	int stop;
	int eof;
};

static cd_snes_stream_t cd_snes_stream = {};
static int cd_snes_state = 0;
static uint8_t snes_last_req = 255;
static unsigned long cd_snes_osd_suppress_until = 0;
static int cd_snes_drive = -1;
static const char cd_snes_mount_path[] = "/tmp/cd-snes";

static int cd_snes_active(void)
{
	const char *name = user_io_get_core_name();
	return name && !strcmp(name, "A0CD-SNES");
}

static int cd_snes_streaming_active(void)
{
	return cd_snes_state == 2 && !strncmp(SelectedPath, cd_snes_mount_path, sizeof(cd_snes_mount_path) - 1);
}

static void *cd_snes_stream_worker(void *)
{
	uint8_t temp[64 * 1024];
	while (1)
	{
		pthread_mutex_lock(&cd_snes_stream.lock);
		while (!cd_snes_stream.stop && (cd_snes_stream.eof || cd_snes_stream.write_pos - cd_snes_stream.read_pos >= CD_SNES_BUFFER_SIZE - sizeof(temp)))
			pthread_cond_wait(&cd_snes_stream.cond, &cd_snes_stream.lock);
		if (cd_snes_stream.stop)
		{
			pthread_mutex_unlock(&cd_snes_stream.lock);
			break;
		}
		uint64_t offset = cd_snes_stream.write_pos;
		uint32_t generation = cd_snes_stream.generation;
		uint64_t remaining = cd_snes_stream.size > offset ? cd_snes_stream.size - offset : 0;
		size_t length = remaining > sizeof(temp) ? sizeof(temp) : remaining;
		pthread_mutex_unlock(&cd_snes_stream.lock);
		if (!length)
		{
			pthread_mutex_lock(&cd_snes_stream.lock);
			if (generation == cd_snes_stream.generation) cd_snes_stream.eof = 1;
			pthread_cond_broadcast(&cd_snes_stream.cond);
			pthread_mutex_unlock(&cd_snes_stream.lock);
			continue;
		}
		ssize_t count = pread(cd_snes_stream.fd, temp, length, offset);
		pthread_mutex_lock(&cd_snes_stream.lock);
		if (generation == cd_snes_stream.generation && offset == cd_snes_stream.write_pos)
		{
			if (count > 0)
			{
				size_t pos = offset % CD_SNES_BUFFER_SIZE;
				size_t first = count;
				if (first > CD_SNES_BUFFER_SIZE - pos) first = CD_SNES_BUFFER_SIZE - pos;
				memcpy(cd_snes_stream.buffer + pos, temp, first);
				if ((size_t)count > first) memcpy(cd_snes_stream.buffer, temp + first, count - first);
				cd_snes_stream.write_pos += count;
			}
			else if (!count) cd_snes_stream.eof = 1;
		}
		pthread_cond_broadcast(&cd_snes_stream.cond);
		pthread_mutex_unlock(&cd_snes_stream.lock);
		if (count < 0) usleep(2000);
	}
	return NULL;
}

static void cd_snes_stream_stop(void)
{
	if (!cd_snes_stream.running) return;
	pthread_mutex_lock(&cd_snes_stream.lock);
	cd_snes_stream.stop = 1;
	pthread_cond_broadcast(&cd_snes_stream.cond);
	pthread_mutex_unlock(&cd_snes_stream.lock);
	pthread_join(cd_snes_stream.thread, NULL);
	if (cd_snes_stream.fd >= 0) close(cd_snes_stream.fd);
	free(cd_snes_stream.buffer);
	pthread_cond_destroy(&cd_snes_stream.cond);
	pthread_mutex_destroy(&cd_snes_stream.lock);
	memset(&cd_snes_stream, 0, sizeof(cd_snes_stream));
	cd_snes_stream.fd = -1;
}

static int cd_snes_stream_wait(size_t amount, int timeout_ms)
{
	struct timespec deadline;
	clock_gettime(CLOCK_REALTIME, &deadline);
	deadline.tv_sec += timeout_ms / 1000;
	deadline.tv_nsec += (timeout_ms % 1000) * 1000000;
	if (deadline.tv_nsec >= 1000000000)
	{
		deadline.tv_sec++;
		deadline.tv_nsec -= 1000000000;
	}
	pthread_mutex_lock(&cd_snes_stream.lock);
	while (!cd_snes_stream.stop && !cd_snes_stream.eof && cd_snes_stream.write_pos - cd_snes_stream.read_pos < amount)
	{
		if (pthread_cond_timedwait(&cd_snes_stream.cond, &cd_snes_stream.lock, &deadline) == ETIMEDOUT) break;
	}
	int ready = cd_snes_stream.write_pos > cd_snes_stream.read_pos;
	pthread_mutex_unlock(&cd_snes_stream.lock);
	return ready;
}

static int cd_snes_stream_start(const char *path, uint64_t offset)
{
	cd_snes_stream_stop();
	int fd = open(path, O_RDONLY | O_CLOEXEC);
	if (fd < 0) return 0;
	struct stat st = {};
	if (fstat(fd, &st) || offset > (uint64_t)st.st_size)
	{
		close(fd);
		return 0;
	}
	uint8_t *buffer = (uint8_t*)malloc(CD_SNES_BUFFER_SIZE);
	if (!buffer)
	{
		close(fd);
		return 0;
	}
	memset(&cd_snes_stream, 0, sizeof(cd_snes_stream));
	cd_snes_stream.buffer = buffer;
	cd_snes_stream.fd = fd;
	cd_snes_stream.size = st.st_size;
	cd_snes_stream.read_pos = offset;
	cd_snes_stream.write_pos = offset;
	cd_snes_stream.generation = 1;
	pthread_mutex_init(&cd_snes_stream.lock, NULL);
	pthread_cond_init(&cd_snes_stream.cond, NULL);
	if (pthread_create(&cd_snes_stream.thread, NULL, cd_snes_stream_worker, NULL))
	{
		close(fd);
		free(buffer);
		pthread_cond_destroy(&cd_snes_stream.cond);
		pthread_mutex_destroy(&cd_snes_stream.lock);
		memset(&cd_snes_stream, 0, sizeof(cd_snes_stream));
		cd_snes_stream.fd = -1;
		return 0;
	}
	cd_snes_stream.running = 1;
	return cd_snes_stream_wait(CD_SNES_PREFILL_SIZE, 5000);
}

static int cd_snes_stream_seek(uint64_t offset)
{
	if (!cd_snes_stream.running || offset > cd_snes_stream.size) return 0;
	pthread_mutex_lock(&cd_snes_stream.lock);
	cd_snes_stream.read_pos = offset;
	cd_snes_stream.write_pos = offset;
	cd_snes_stream.generation++;
	cd_snes_stream.eof = 0;
	pthread_cond_broadcast(&cd_snes_stream.cond);
	pthread_mutex_unlock(&cd_snes_stream.lock);
	return cd_snes_stream_wait(CD_SNES_PREFILL_SIZE, 5000);
}

static int cd_snes_stream_read(void *data, size_t length)
{
	if (!cd_snes_stream.running) return 0;
	memset(data, 0, length);
	struct timespec deadline;
	clock_gettime(CLOCK_REALTIME, &deadline);
	deadline.tv_nsec += 100000000;
	if (deadline.tv_nsec >= 1000000000)
	{
		deadline.tv_sec++;
		deadline.tv_nsec -= 1000000000;
	}
	pthread_mutex_lock(&cd_snes_stream.lock);
	while (!cd_snes_stream.stop && !cd_snes_stream.eof && cd_snes_stream.write_pos - cd_snes_stream.read_pos < length)
	{
		if (pthread_cond_timedwait(&cd_snes_stream.cond, &cd_snes_stream.lock, &deadline) == ETIMEDOUT) break;
	}
	size_t available = cd_snes_stream.write_pos - cd_snes_stream.read_pos;
	if (available > length) available = length;
	if (available)
	{
		size_t pos = cd_snes_stream.read_pos % CD_SNES_BUFFER_SIZE;
		size_t first = available;
		if (first > CD_SNES_BUFFER_SIZE - pos) first = CD_SNES_BUFFER_SIZE - pos;
		memcpy(data, cd_snes_stream.buffer + pos, first);
		if (available > first) memcpy((uint8_t*)data + first, cd_snes_stream.buffer, available - first);
		cd_snes_stream.read_pos += available;
		pthread_cond_broadcast(&cd_snes_stream.cond);
	}
	pthread_mutex_unlock(&cd_snes_stream.lock);
	return available;
}

static int cd_snes_name_compare(const void *a, const void *b)
{
	return strcasecmp((const char*)a, (const char*)b);
}

static int cd_snes_find_rom(char *path, size_t size)
{
	DIR *dir = opendir(cd_snes_mount_path);
	if (!dir) return 0;
	char names[64][NAME_MAX + 1];
	int count = 0;
	struct dirent *entry;
	while ((entry = readdir(dir)) && count < 64)
	{
		const char *ext = strrchr(entry->d_name, '.');
		if (!ext || (strcasecmp(ext, ".sfc") && strcasecmp(ext, ".smc"))) continue;
		strncpy(names[count], entry->d_name, NAME_MAX);
		names[count][NAME_MAX] = 0;
		count++;
	}
	closedir(dir);
	if (!count) return 0;
	qsort(names, count, sizeof(names[0]), cd_snes_name_compare);
	snprintf(path, size, "%s/%s", cd_snes_mount_path, names[0]);
	return 1;
}

static int cd_snes_open_disc(char *rom, size_t size)
{
	snesmsu_diag("DISC open begin core=%s state=%d", user_io_get_core_name(1), cd_snes_state);
	mkdir(cd_snes_mount_path, 0755);
	umount2(cd_snes_mount_path, MNT_DETACH);
	char device[32] = {};
	for (int i = 0; i < 8; i++)
	{
		snprintf(device, sizeof(device), "/dev/sr%d", i);
		if (!access(device, R_OK)) break;
		device[0] = 0;
	}
	if (!device[0])
	{
		snesmsu_diag("DISC no readable /dev/srN device found");
		return 0;
	}
	snesmsu_diag("DISC device=%s mount=%s", device, cd_snes_mount_path);
	if (mount(device, cd_snes_mount_path, "iso9660", MS_RDONLY | MS_NOSUID | MS_NODEV, "iocharset=utf8") &&
		mount(device, cd_snes_mount_path, "iso9660", MS_RDONLY | MS_NOSUID | MS_NODEV, NULL))
	{
		snesmsu_diag("DISC mount FAILED errno=%d (%s)", errno, strerror(errno));
		return 0;
	}
	snesmsu_diag("DISC mount OK");
	cd_snes_drive = open(device, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
	if (cd_snes_drive >= 0) ioctl(cd_snes_drive, CDROM_SELECT_SPEED, 0);
	int found = cd_snes_find_rom(rom, size);
	snesmsu_diag("DISC ROM discovery result=%d path=%s", found, found ? rom : "(none)");
	return found;
}

static void cd_snes_close_disc(void)
{
	snesmsu_diag("DISC close begin state=%d mount=%s drive_fd=%d has_cd=%d audio_size=%u",
		cd_snes_state, cd_snes_mount_path, cd_snes_drive, has_cd, f_audio.size);
	cd_snes_stream_stop();
	FileClose(&f_audio);

	// Match MD+ teardown: release the ISO9660 mount first and explicitly
	// clear Linux's optical-drive tray lock before the final close.
	if (umount(cd_snes_mount_path)) umount2(cd_snes_mount_path, MNT_DETACH);
	if (cd_snes_drive >= 0)
	{
		ioctl(cd_snes_drive, CDROM_LOCKDOOR, 0);
		ioctl(cd_snes_drive, CDROM_SELECT_SPEED, 4);
		close(cd_snes_drive);
		cd_snes_drive = -1;
	}

	cd_snes_state = 0;
	has_cd = 0;
	snes_last_req = 255;
	cd_snes_osd_suppress_until = 0;
	snesmsu_diag("DISC close complete");
}

void snes_cd_session_reset(void)
{
	snesmsu_diag("SESSION reset requested core=%s", user_io_get_core_name(1));
	// Like MD+, A0CD-SNES can be reloaded without an intermediate core name.
	// Force the host-side MSU/disc session back to a cold state on every new
	// core initialization so it cannot leak into a following MD+ session.
	cd_snes_close_disc();
	memset(snes_romFileName, 0, sizeof(snes_romFileName));
}

static void msu_send_command(uint64_t cmd)
{
	spi_uio_cmd_cont(UIO_CD_SET);
	spi_w((cmd >> 00) & 0xFFFF);
	spi_w((cmd >> 16) & 0xFFFF);
	spi_w((cmd >> 32) & 0xFFFF);
	DisableIO();
}

static int msu_send_data(fileTYPE *f, int idx)
{
	int chunk = sizeof(buf);

	memset(buf, 0, chunk);
	if (cd_snes_streaming_active() && cd_snes_stream.running) cd_snes_stream_read(buf, chunk);
	else if (f->size) FileReadAdv(f, buf, chunk);

	user_io_set_index(idx);
	user_io_set_download(1);
	user_io_file_tx_data(buf, chunk);
	user_io_set_download(0);

	return 1;
}

void snes_msu_init(const char* name)
{
	snesmsu_diag("INIT begin rom=%s core=%s cd_active=%d", name ? name : "(null)", user_io_get_core_name(1), cd_snes_active());
	static fileTYPE f = {};
	snes_last_req = 255;
	cd_snes_stream_stop();
	FileClose(&f_audio);

	memset(snes_romFileName, 0, 1024);
	int extSize = strlen(strrchr(name, '.'));
	strncpy(snes_romFileName, name, strlen(name) - extSize);
	printf("MSU: Rom named '%s' initialised\n", name);

	snprintf(SelectedPath, sizeof(SelectedPath), "%s.msu", snes_romFileName);
	has_cd = FileOpen(&f, SelectedPath) ? 1 : 0;
	uint32_t size = f.size;
	snesmsu_diag("INIT MSU file path=%s open=%d size=%u", SelectedPath, has_cd, size);
	FileClose(&f);

	printf("MSU: enable cd: %d\n", has_cd);

	if (size && size < 0x1F200000)
	{
		msu_send_command((0x20600000ULL << 16) | MSU_DATA_BASE);
		user_io_file_tx(SelectedPath, 3, 0, 0, 0, 0x20600000);
	}

	msu_send_command((has_cd << 15) | MSU_CD_SET);
	snesmsu_diag("INIT complete has_cd=%d rombase=%s", has_cd, snes_romFileName);
}

void snes_cd_session_poll(void)
{
	static int diag_announced = 0;
	if (cd_snes_active() && !diag_announced)
	{
		snesmsu_diag("SESSION active core detected state=%d", cd_snes_state);
		diag_announced = 1;
	}
	if (!cd_snes_active()) diag_announced = 0;
	if (!cd_snes_active())
	{
		if (cd_snes_state) cd_snes_close_disc();
		return;
	}
	if (cd_snes_state) return;
	cd_snes_state = 1;
	snesmsu_diag("SESSION startup attempt");
	char rom[PATH_MAX] = {};
	if (!cd_snes_open_disc(rom, sizeof(rom)))
	{
		snesmsu_diag("SESSION disc open failed");		cd_snes_close_disc();
		cd_snes_state = -1;
		return;
	}
	cd_snes_state = 2;
	snesmsu_diag("ROM TX begin path=%s index=0 opensave=1", rom);
	int snes_tx_ok = user_io_file_tx(rom, 0, 1, 0, 0, 0);
	snesmsu_diag("ROM TX end result=%d", snes_tx_ok);
	if (!snes_tx_ok) cd_snes_close_disc();
	else
	{
		cd_snes_osd_suppress_until = GetTimer(2000);
		MenuHide();
	}
}

int snes_cd_suppress_osd(void)
{
	if (!cd_snes_active() || !cd_snes_osd_suppress_until) return 0;
	if (CheckTimer(cd_snes_osd_suppress_until))
	{
		cd_snes_osd_suppress_until = 0;
		return 0;
	}
	return 1;
}

void snes_poll(void)
{
	if (!has_cd) { snes_last_req = 255; return; }

	// Detect incoming command via CD_GET (which we are repurposing for MSU1)
	uint8_t req = spi_uio_cmd_cont(UIO_CD_GET);
	if (req != snes_last_req)
	{
		snes_last_req = req;

		uint16_t command = spi_w(0);
		uint32_t data = spi_w(0);
		data = (spi_w(0) << 16) | data;
		DisableIO();

		switch(command)
		{
		case 0xFF:
			printf("MSU: request to reset\n");
			break;

		case 0x35:
			snesmsu_diag("FPGA AUDIO track request track=%u req=%u", data, req);
			snprintf(SelectedPath, sizeof(SelectedPath), "%s-%d.pcm", snes_romFileName, data);
			printf("MSU: New track selected: %s\n", SelectedPath);
			FileOpen(&f_audio, SelectedPath);
			snesmsu_diag("AUDIO PCM open path=%s size=%u", SelectedPath, f_audio.size);			if (cd_snes_streaming_active() && f_audio.size) cd_snes_stream_start(SelectedPath, 0);
			printf(f_audio.size ? "MSU: Track mounted\n" : "MSU: Track not found!\n");
			msu_send_command((f_audio.size << 16) | MSU_AUDIO_TRACK_MOUNTED);
			break;

		case 0x36:
			printf("MSU: Jump to offset: 0x%X\n", data * 1024);
			if (cd_snes_streaming_active() && cd_snes_stream.running) cd_snes_stream_seek((uint64_t)data * 1024);
			else FileSeek(&f_audio, data * 1024, SEEK_SET);
			msu_send_data(&f_audio, 2);
			break;

		case 0x34:
			// Next sector requested
			msu_send_data(&f_audio, 2);
			break;
		}
	}
	else
	{
		DisableIO();
	}
}
