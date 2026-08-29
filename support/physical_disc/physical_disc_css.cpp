// physical_disc_css.cpp — see physical_disc_css.h.
//
// libdvdcss is dlopen'd at runtime (never linked). The only libdvdcss surface we
// use is its published API, declared here so the build needs no libdvdcss headers.

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <limits.h>
#include <fcntl.h>
#include <unistd.h>
#include <dlfcn.h>
#include <sys/ioctl.h>
#include <linux/cdrom.h>
#include <linux/fs.h>

#include "physical_disc_css.h"

// Log to stdout AND to a file, so the reason for a failed mount is visible over
// SSH regardless of which (possibly supervised) MiSTer instance handled it.
#define CSS_LOG_PATH "/tmp/dvdcss.log"
static void css_log(const char *fmt, ...)
{
	char buf[256];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	printf("CSS: %s\n", buf);
	FILE *f = fopen(CSS_LOG_PATH, "a");
	if (f) { fprintf(f, "%s\n", buf); fclose(f); }
}

// --- libdvdcss API (from dvdcss.h; reproduced so we need no external headers) ---
typedef struct dvdcss_s *dvdcss_t;
#define DVDCSS_READ_DECRYPT (1 << 0)
#define DVDCSS_SEEK_KEY     (1 << 1)

typedef dvdcss_t(*fn_open_t)(const char *);
typedef int (*fn_close_t)(dvdcss_t);
typedef int (*fn_seek_t)(dvdcss_t, int, int);
typedef int (*fn_read_t)(dvdcss_t, void *, int, int);
typedef char *(*fn_error_t)(dvdcss_t);

static void *css_lib = NULL;
static fn_open_t  p_open  = NULL;
static fn_close_t p_close = NULL;
static fn_seek_t  p_seek  = NULL;
static fn_read_t  p_read  = NULL;
static fn_error_t p_error = NULL;

static dvdcss_t css = NULL;
static uint64_t css_size = 0;   // bytes
static int css_pos = -1;        // last block position, to avoid redundant seeks

// Candidate locations for the user-supplied library. The install script drops it
// at the first path; the sonames cover a lib already on the default search path.
static const char *css_lib_names[] =
{
	"/media/fat/dvdcss/libdvdcss.so.2",
	"/media/fat/linux/libdvdcss.so.2",
	"libdvdcss.so.2",
	"libdvdcss.so",
	NULL
};

static int load_library(void)
{
	if (css_lib) return 1;

	for (int i = 0; css_lib_names[i]; i++)
	{
		css_lib = dlopen(css_lib_names[i], RTLD_NOW | RTLD_LOCAL);
		if (css_lib)
		{
			css_log("loaded %s", css_lib_names[i]);
			break;
		}
		css_log("dlopen(%s): %s", css_lib_names[i], dlerror());
	}
	if (!css_lib)
	{
		css_log("libdvdcss not found — run Scripts/install_dvdcss to play encrypted discs");
		return 0;
	}

	p_open  = (fn_open_t)  dlsym(css_lib, "dvdcss_open");
	p_close = (fn_close_t) dlsym(css_lib, "dvdcss_close");
	p_seek  = (fn_seek_t)  dlsym(css_lib, "dvdcss_seek");
	p_read  = (fn_read_t)  dlsym(css_lib, "dvdcss_read");
	p_error = (fn_error_t) dlsym(css_lib, "dvdcss_error");

	if (!p_open || !p_close || !p_seek || !p_read)
	{
		css_log("libdvdcss is missing required symbols");
		dlclose(css_lib);
		css_lib = NULL;
		return 0;
	}
	return 1;
}

// Find the first /dev/srN that currently holds a disc. Returns 1 and fills `out`
// on success. Mirrors the drive scan in physical_disc.cpp, but opens nothing that
// would collide with libdvdcss (it needs its own handle for the CSS ioctls).
// The disc may report CDS_DRIVE_NOT_READY while it spins up, so a not-ready drive
// with media is accepted as a fallback and left for dvdcss_open to spin up.
static int find_dvd_device(char *out, int outsz)
{
	char fallback[32] = "";
	uint64_t fb_size = 0;

	for (int i = 0; i < 8; i++)
	{
		char path[32];
		snprintf(path, sizeof(path), "/dev/sr%d", i);
		int fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
		if (fd < 0) continue;

		int status = ioctl(fd, CDROM_DRIVE_STATUS, CDSL_CURRENT);
		uint64_t bytes = 0;
		if (ioctl(fd, BLKGETSIZE64, &bytes) < 0) bytes = 0;
		close(fd);

		css_log("scan %s: drive_status=%d size=%lluMB", path, status,
		        (unsigned long long)(bytes >> 20));

		if (status == CDS_DISC_OK)
		{
			css_size = bytes;
			snprintf(out, outsz, "%s", path);
			return 1;
		}
		if (status == CDS_DRIVE_NOT_READY && fallback[0] == '\0')
		{
			snprintf(fallback, sizeof(fallback), "%s", path);
			fb_size = bytes;
		}
	}

	if (fallback[0])
	{
		css_log("no ready disc; trying not-ready %s (spinning up)", fallback);
		css_size = fb_size;
		snprintf(out, outsz, "%s", fallback);
		return 1;
	}
	return 0;
}

int physical_disc_css_open(void)
{
	if (css) return 1;
	css_log("open: begin");
	if (!load_library()) return 0;

	char dev[32];
	if (!find_dvd_device(dev, sizeof(dev)))
	{
		css_log("no readable disc in an optical drive");
		return 0;
	}

	css = p_open(dev);
	if (!css)
	{
		css_log("dvdcss_open(%s) failed", dev);
		return 0;
	}

	// If the size was not available at scan time (drive was still spinning up),
	// read it now that the device is open and the disc is ready.
	if (css_size == 0)
	{
		int fd = open(dev, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
		if (fd >= 0)
		{
			uint64_t bytes = 0;
			if (ioctl(fd, BLKGETSIZE64, &bytes) == 0) css_size = bytes;
			close(fd);
		}
	}

	css_pos = -1;
	css_log("opened %s (%llu MB)%s", dev, (unsigned long long)(css_size >> 20),
	        css_size ? "" : "  <-- WARNING size 0, core will reject");
	return 1;
}

int physical_disc_css_active(void)
{
	return css != NULL;
}

uint64_t physical_disc_css_size(void)
{
	return css ? css_size : 0;
}

int physical_disc_css_read(void *buf, uint32_t lba, uint32_t count)
{
	if (!css) return -1;

	// Seek with DVDCSS_SEEK_KEY on a discontinuity so libdvdcss fetches (and
	// caches) the title key covering this position; sequential reads within a
	// cached region skip the seek. DVDCSS_READ_DECRYPT descrambles each block.
	if ((int)lba != css_pos)
	{
		if (p_seek(css, (int)lba, DVDCSS_SEEK_KEY) < 0)
		{
			css_log("seek to %u failed: %s", lba, p_error ? p_error(css) : "?");
			css_pos = -1;
			return -1;
		}
	}

	int n = p_read(css, buf, (int)count, DVDCSS_READ_DECRYPT);
	if (n < 0)
	{
		css_log("read %u@%u failed: %s", count, lba, p_error ? p_error(css) : "?");
		css_pos = -1;
		return -1;
	}

	css_pos = (int)(lba + n);

	// --- diagnostics: confirm the core is reading, and that the data looks like a
	// filesystem. The ISO9660 primary volume descriptor lives at sector 16 with
	// "CD001" at byte offset 1; UDF anchor is at sector 256. ---
	static unsigned long nreads = 0;
	nreads++;
	if (nreads == 1) css_log("first read ok: lba=%u count=%u n=%d", lba, count, n);
	if (lba <= 16 && (uint32_t)(lba + n) > 16)
	{
		const unsigned char *p = (const unsigned char *)buf + (16 - lba) * 2048;
		css_log("LBA16 type=%02x sig=%c%c%c%c%c", p[0], p[1], p[2], p[3], p[4], p[5]);
	}
	if ((nreads % 4096) == 0) css_log("progress: reads=%lu last_lba=%u", nreads, lba);
	return n;
}

void physical_disc_css_close(void)
{
	if (css && p_close) p_close(css);
	css = NULL;
	css_pos = -1;
	css_size = 0;
}
