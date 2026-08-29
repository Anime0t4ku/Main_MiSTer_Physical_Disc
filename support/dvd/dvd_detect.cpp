// dvd_detect.cpp — see dvd_detect.h.

#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <scsi/sg.h>

#include "dvd_detect.h"

// Read one 2048-byte data sector with READ(10) — works on DVDs (the CD layer's
// READ CD / 2352 path does not). Returns 0 on success.
static int dvd_read10(int fd, uint32_t lba, uint8_t *buf)
{
	uint8_t cdb[10] = { 0x28, 0,
		(uint8_t)(lba >> 24), (uint8_t)(lba >> 16), (uint8_t)(lba >> 8), (uint8_t)lba,
		0, 0, 1, 0 };
	uint8_t sense[32];
	struct sg_io_hdr io;
	memset(&io, 0, sizeof(io));
	io.interface_id = 'S';
	io.dxfer_direction = SG_DXFER_FROM_DEV;
	io.cmd_len = sizeof(cdb);
	io.cmdp = cdb;
	io.dxfer_len = 2048;
	io.dxferp = buf;
	io.sbp = sense;
	io.mx_sb_len = sizeof(sense);
	io.timeout = 5000;
	if (ioctl(fd, SG_IO, &io) < 0) return -1;
	if (io.status || io.host_status) return -1;
	return 0;
}

int dvd_video_probe(int fd)
{
	uint8_t sec[2048];
	if (dvd_read10(fd, 16, sec)) return 0;              // primary volume descriptor
	if (memcmp(sec + 1, "CD001", 5)) return 0;

	uint32_t root_lba = sec[158] | (sec[159] << 8) | (sec[160] << 16) | ((uint32_t)sec[161] << 24);
	uint32_t root_len = sec[166] | (sec[167] << 8) | (sec[168] << 16) | ((uint32_t)sec[169] << 24);
	uint32_t nsec = (root_len + 2047) / 2048;
	if (nsec > 8) nsec = 8;
	for (uint32_t s = 0; s < nsec; s++)
	{
		if (dvd_read10(fd, root_lba + s, sec)) break;
		uint32_t off = 0;
		while (off + 33 < 2048)
		{
			uint8_t rlen = sec[off];
			if (!rlen) break;
			uint8_t flags = sec[off + 25];
			uint8_t nlen = sec[off + 32];
			if ((flags & 0x02) && nlen == 8 && !memcmp(sec + off + 33, "VIDEO_TS", 8)) return 1;
			off += rlen;
		}
	}
	return 0;
}
