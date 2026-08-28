// physical_disc_css.cpp — see physical_disc_css.h.
//
// libdvdcss is dlopen'd at runtime (never linked). The only libdvdcss surface we
// use is its published API, declared here so the build needs no libdvdcss headers.

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <dlfcn.h>
#include <sys/ioctl.h>
#include <linux/cdrom.h>
#include <linux/fs.h>

#include "physical_disc_css.h"

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
			printf("CSS: loaded %s\n", css_lib_names[i]);
			break;
		}
	}
	if (!css_lib)
	{
		printf("CSS: libdvdcss not found — run Scripts/install_dvdcss to play encrypted discs\n");
		return 0;
	}

	p_open  = (fn_open_t)  dlsym(css_lib, "dvdcss_open");
	p_close = (fn_close_t) dlsym(css_lib, "dvdcss_close");
	p_seek  = (fn_seek_t)  dlsym(css_lib, "dvdcss_seek");
	p_read  = (fn_read_t)  dlsym(css_lib, "dvdcss_read");
	p_error = (fn_error_t) dlsym(css_lib, "dvdcss_error");

	if (!p_open || !p_close || !p_seek || !p_read)
	{
		printf("CSS: libdvdcss is missing required symbols\n");
		dlclose(css_lib);
		css_lib = NULL;
		return 0;
	}
	return 1;
}

// Find the first /dev/srN that currently holds a disc. Returns 1 and fills `out`
// on success. Mirrors the drive scan in physical_disc.cpp, but opens nothing that
// would collide with libdvdcss (it needs its own handle for the CSS ioctls).
static int find_dvd_device(char *out, int outsz)
{
	for (int i = 0; i < 8; i++)
	{
		char path[32];
		snprintf(path, sizeof(path), "/dev/sr%d", i);
		int fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
		if (fd < 0) continue;

		int status = ioctl(fd, CDROM_DRIVE_STATUS, CDSL_CURRENT);
		if (status == CDS_DISC_OK)
		{
			uint64_t bytes = 0;
			if (ioctl(fd, BLKGETSIZE64, &bytes) < 0) bytes = 0;
			css_size = bytes;
			close(fd);
			snprintf(out, outsz, "%s", path);
			return 1;
		}
		close(fd);
	}
	return 0;
}

int physical_disc_css_open(void)
{
	if (css) return 1;
	if (!load_library()) return 0;

	char dev[32];
	if (!find_dvd_device(dev, sizeof(dev)))
	{
		printf("CSS: no readable disc in an optical drive\n");
		return 0;
	}

	css = p_open(dev);
	if (!css)
	{
		printf("CSS: dvdcss_open(%s) failed\n", dev);
		return 0;
	}
	css_pos = -1;
	printf("CSS: opened %s (%llu MB)\n", dev, (unsigned long long)(css_size >> 20));
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
			printf("CSS: seek to %u failed: %s\n", lba, p_error ? p_error(css) : "?");
			css_pos = -1;
			return -1;
		}
	}

	int n = p_read(css, buf, (int)count, DVDCSS_READ_DECRYPT);
	if (n < 0)
	{
		printf("CSS: read %u@%u failed: %s\n", count, lba, p_error ? p_error(css) : "?");
		css_pos = -1;
		return -1;
	}

	css_pos = (int)(lba + n);
	return n;
}

void physical_disc_css_close(void)
{
	if (css && p_close) p_close(css);
	css = NULL;
	css_pos = -1;
	css_size = 0;
}
