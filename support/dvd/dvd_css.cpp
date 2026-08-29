// dvd_css.cpp — see dvd_css.h.
//
// libdvdcss is dlopen'd at runtime (never linked). The only libdvdcss surface we
// use is its published API, declared here so the build needs no libdvdcss headers.

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <limits.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <dlfcn.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <scsi/sg.h>
#include <linux/cdrom.h>
#include <linux/fs.h>

#include "dvd_css.h"
#include "../../menu.h"   // ProgressMessage() — on-screen feedback during key crack

// Status logging: to stdout and to /tmp/dvdcss.log (the latter so the reason for
// a failed mount is visible over SSH, where the core's stdout is not).
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
#define DVDCSS_NOFLAGS       0
#define DVDCSS_READ_DECRYPT (1 << 0)
#define DVDCSS_SEEK_MPEG    (1 << 0)
#define DVDCSS_SEEK_KEY     (1 << 1)

typedef dvdcss_t(*fn_open_t)(const char *);
typedef int (*fn_close_t)(dvdcss_t);
typedef int (*fn_seek_t)(dvdcss_t, int, int);
typedef int (*fn_read_t)(dvdcss_t, void *, int, int);
typedef char *(*fn_error_t)(dvdcss_t);
typedef int (*fn_scram_t)(dvdcss_t);

static void *css_lib = NULL;
static fn_open_t  p_open  = NULL;
static fn_close_t p_close = NULL;
static fn_seek_t  p_seek  = NULL;
static fn_read_t  p_read  = NULL;
static fn_error_t p_error = NULL;
static fn_scram_t p_scram = NULL;   // dvdcss_is_scrambled (optional; absent on old libs)

static dvdcss_t css = NULL;
static uint64_t css_size = 0;   // bytes
static int css_pos = -1;        // last block position, to avoid redundant seeks
static int cur_vob = -1;        // VOB index whose title key is currently selected
static int raw_fd = -1;         // raw drive fd when libdvdcss is absent (no decrypt)
static int region_set = 1;      // 0 if the drive has no CSS region set (keys will crack)
static time_t warn_until = 0;   // deadline for re-asserting the "install libdvdcss" popup

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
		if (css_lib) break;
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
	p_scram = (fn_scram_t) dlsym(css_lib, "dvdcss_is_scrambled");

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

	if (fallback[0])   // disc present but still spinning up -> let dvdcss_open wait
	{
		css_size = fb_size;
		snprintf(out, outsz, "%s", fallback);
		return 1;
	}
	return 0;
}

static inline uint32_t rd_le32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// Raw READ(10) of `count` 2048-byte sectors at `lba` — used only in the no-
// libdvdcss fallback, which plays unencrypted DVDs (CSS discs come back
// scrambled and the core flags CSS ENCRYPTED). Returns sectors read or -1.
static int raw_read10(int fd, uint32_t lba, void *buf, int count)
{
	uint8_t cdb[10] = { 0x28, 0,
		(uint8_t)(lba >> 24), (uint8_t)(lba >> 16), (uint8_t)(lba >> 8), (uint8_t)lba,
		0, (uint8_t)(count >> 8), (uint8_t)count, 0 };
	uint8_t sense[32];
	struct sg_io_hdr io;
	memset(&io, 0, sizeof(io));
	io.interface_id = 'S';
	io.dxfer_direction = SG_DXFER_FROM_DEV;
	io.cmd_len = sizeof(cdb);
	io.cmdp = cdb;
	io.dxfer_len = (unsigned)count * 2048;
	io.dxferp = buf;
	io.sbp = sense;
	io.mx_sb_len = sizeof(sense);
	io.timeout = 5000;
	if (ioctl(fd, SG_IO, &io) < 0) return -1;
	if (io.status || io.host_status) return -1;
	return count;
}

// Does the drive have a CSS region set? RPC-II drives refuse the title-key ioctl
// (ReadTitleKey) unless a region matching the disc is set, which forces libdvdcss
// into the slow, sometimes-failing per-title crack. Read the RPC state via SG_IO
// REPORT KEY (format 0x08): region_mask 0xff means no region is set.
static int drive_region_set(const char *dev)
{
	int fd = open(dev, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
	if (fd < 0) return 1;   // can't check -> assume set (don't nag)

	uint8_t cdb[12] = { 0xA4, 0, 0, 0, 0, 0, 0, 0, 0, 8, 0x08, 0 };   // REPORT KEY, RPC state
	uint8_t buf[8] = { 0 }, sense[32];
	struct sg_io_hdr io;
	memset(&io, 0, sizeof(io));
	io.interface_id = 'S';
	io.dxfer_direction = SG_DXFER_FROM_DEV;
	io.cmd_len = sizeof(cdb);
	io.cmdp = cdb;
	io.dxfer_len = sizeof(buf);
	io.dxferp = buf;
	io.sbp = sense;
	io.mx_sb_len = sizeof(sense);
	io.timeout = 5000;

	int set = 1;
	if (ioctl(fd, SG_IO, &io) == 0 && io.status == 0)
		set = (buf[5] != 0xff);   // region_mask == 0xff -> no region set
	close(fd);
	return set;
}

// Is the disc CSS/CPPM-protected? READ DVD STRUCTURE (0xAD), format 0x01
// (copyright info): response byte 4 (CPST) is 0 for none, non-zero for CSS.
// Asked WITHOUT authentication (a status read), so it works with no libdvdcss —
// which is how we can warn instead of parking on a black screen: many drives
// refuse a plain READ(10) of the scrambled VOB area, so the core would starve
// with no scrambled PES ever reaching its CSS ENCRYPTED detector.
static int disc_is_encrypted(const char *dev)
{
	int fd = open(dev, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
	if (fd < 0) return 0;

	uint8_t cdb[12] = { 0xAD, 0, 0, 0, 0, 0, 0, 0x01, 0, 8, 0, 0 };   // READ DVD STRUCTURE, copyright
	uint8_t buf[8] = { 0 }, sense[32];
	struct sg_io_hdr io;
	memset(&io, 0, sizeof(io));
	io.interface_id = 'S';
	io.dxfer_direction = SG_DXFER_FROM_DEV;
	io.cmd_len = sizeof(cdb);
	io.cmdp = cdb;
	io.dxfer_len = sizeof(buf);
	io.dxferp = buf;
	io.sbp = sense;
	io.mx_sb_len = sizeof(sense);
	io.timeout = 5000;

	int css = (ioctl(fd, SG_IO, &io) == 0 && io.status == 0) ? (buf[4] != 0) : 0;
	close(fd);
	return css;   // caller logs the outcome (the encrypted-without-libdvdcss case)
}

// Read `count` raw (undecrypted) 2048-byte sectors at `lba` — for the unscrambled
// ISO9660 metadata used to enumerate VOB files. Returns sectors read or -1.
static int css_raw_read(uint32_t lba, void *buf, int count)
{
	if (p_seek(css, (int)lba, DVDCSS_NOFLAGS) < 0) return -1;
	return p_read(css, buf, count, DVDCSS_NOFLAGS);
}

static int name_eq(const char *nm, int nlen, const char *want)
{
	int wl = (int)strlen(want);
	if (nlen != wl) return 0;
	for (int i = 0; i < wl; i++)
		if ((nm[i] | 32) != (want[i] | 32)) return 0;
	return 1;
}

static int name_has_vob(const char *nm, int nlen)
{
	for (int i = 0; i + 4 <= nlen; i++)
		if (nm[i] == '.' && (nm[i + 1] | 32) == 'v' && (nm[i + 2] | 32) == 'o' && (nm[i + 3] | 32) == 'b')
			return 1;
	return 0;
}

// Find a directory child by name (want_dir=1 for a subdirectory).
static int iso_find(uint32_t dir_lba, uint32_t dir_len, const char *name, int want_dir,
                    uint32_t *out_lba, uint32_t *out_len)
{
	uint8_t sec[2048];
	uint32_t nsec = (dir_len + 2047) / 2048;
	for (uint32_t s = 0; s < nsec; s++)
	{
		if (css_raw_read(dir_lba + s, sec, 1) < 1) return 0;
		uint32_t off = 0;
		while (off + 33 < 2048)
		{
			uint8_t rlen = sec[off];
			if (rlen == 0) break;               // rest of the sector is padding
			uint8_t flags = sec[off + 25];
			uint8_t nlen = sec[off + 32];
			const char *nm = (const char *)(sec + off + 33);
			int is_dir = (flags & 0x02) != 0;
			if ((want_dir ? is_dir : !is_dir) && name_eq(nm, nlen, name))
			{
				*out_lba = rd_le32(sec + off + 2);
				*out_len = rd_le32(sec + off + 10);
				return 1;
			}
			off += rlen;
		}
	}
	return 0;
}

// Every *.VOB file's extent [start, start+nsec), discovered at mount. Used to
// (a) pre-crack each title key at its VOB start, and (b) decide per read whether
// to decrypt: VOB sectors get SEEK_KEY(lba)+DECRYPT (the SEEK_KEY is a fast cached
// lookup after the pre-crack); filesystem/IFO sectors are read raw (NOFLAGS) so
// dvdcss can't corrupt them with a wrong current title key.
#define MAX_VOBS 64
static struct { uint32_t start, nsec; } g_vobs[MAX_VOBS];
static int g_nvobs = 0;

// Collect every *.VOB file's extent (start LBA + length in sectors).
static void collect_vobs(uint32_t dir_lba, uint32_t dir_len)
{
	uint8_t sec[2048];
	uint32_t nsec = (dir_len + 2047) / 2048;
	for (uint32_t s = 0; s < nsec; s++)
	{
		if (css_raw_read(dir_lba + s, sec, 1) < 1) break;
		uint32_t off = 0;
		while (off + 33 < 2048)
		{
			uint8_t rlen = sec[off];
			if (rlen == 0) break;
			uint8_t flags = sec[off + 25];
			uint8_t nlen = sec[off + 32];
			const char *nm = (const char *)(sec + off + 33);
			if (!(flags & 0x02) && name_has_vob(nm, nlen) && g_nvobs < MAX_VOBS)
			{
				g_vobs[g_nvobs].start = rd_le32(sec + off + 2);
				g_vobs[g_nvobs].nsec = (rd_le32(sec + off + 10) + 2047) / 2048;
				g_nvobs++;
			}
			off += rlen;
		}
	}
}

// Index of the VOB extent containing `lba` (a sector that may be scrambled), or
// -1 for a filesystem/IFO sector (never scrambled -> must be read raw).
static int vob_index(uint32_t lba)
{
	for (int i = 0; i < g_nvobs; i++)
		if (lba >= g_vobs[i].start && lba < g_vobs[i].start + g_vobs[i].nsec) return i;
	return -1;
}

// Walk ISO9660 (root -> VIDEO_TS) and record every VOB extent, then crack each
// title key at its VOB start (fast cached lookups thereafter). Done once at mount.
static void build_vob_list(void)
{
	g_nvobs = 0;
	uint8_t sec[2048];
	if (css_raw_read(16, sec, 1) < 1) { css_log("vobs: PVD read failed"); return; }
	if (memcmp(sec + 1, "CD001", 5) != 0) { css_log("vobs: not ISO9660"); return; }

	uint32_t root_lba = rd_le32(sec + 156 + 2);
	uint32_t root_len = rd_le32(sec + 156 + 10);
	uint32_t vts_lba = 0, vts_len = 0;
	if (!iso_find(root_lba, root_len, "VIDEO_TS", 1, &vts_lba, &vts_len))
	{
		css_log("vobs: VIDEO_TS dir not found");
		return;
	}
	collect_vobs(vts_lba, vts_len);

	// Fetch each VOB's title key at its start sector. With a drive region set this
	// is instant (ioctl); with none it's a slow crack — say so on screen instead of
	// leaving an unexplained black screen. Also show a bar (blocks the main loop).
	// The region matters only for a scrambled disc — an unencrypted DVD needs no
	// key, so don't warn about cracking there. dvdcss_is_scrambled is optional;
	// assume scrambled if the lib is too old to tell (rather than hide a real warning).
	int scrambled = p_scram ? (p_scram(css) != 0) : 1;
	// Sidebar title fits ~9 chars; the main line is capped at 27 (ProgressMessage).
	const char *title = "DVD";
	const char *text  = (!region_set && scrambled) ? "No drive region: cracking" : "Preparing disc";
	ProgressMessage();   // reset so the first update renders
	int keyed = 0;
	for (int i = 0; i < g_nvobs; i++)
	{
		ProgressMessage(title, text, i, g_nvobs);
		if (p_seek(css, (int)g_vobs[i].start, DVDCSS_SEEK_KEY) >= 0) keyed++;
	}
	ProgressMessage();   // clear
	css_log("%d VOBs, %d title keys (region %s)", g_nvobs, keyed, region_set ? "set" : "NOT set");
	css_pos = -1;
}

int dvd_css_open(void)
{
	if (css || raw_fd >= 0) return 1;

	char dev[32];
	if (!find_dvd_device(dev, sizeof(dev)))
	{
		css_log("no readable disc in an optical drive");
		return 0;
	}

	region_set = drive_region_set(dev);
	if (!region_set)
		css_log("drive has NO CSS region set — title keys must be cracked (slow); see README");

	if (load_library())
	{
		// Use libdvdcss's default method: fetch each title key from the DRIVE via
		// the CSS key ioctls (fast — a REPORT KEY per title) when the drive
		// supports them, and fall back to CRACKING the key from the data only when
		// it doesn't (the crack can be slow, minutes on some discs). Either way the
		// key is fetched at the VOB START (build_vob_list below), where both paths
		// are reliable. (We used to force DVDCSS_METHOD=title, i.e. always crack;
		// that predates seeking at the VOB start and was needlessly slow.)

		// Persist keys per disc: a slow crack becomes a one-time cost, and
		// re-inserting the same disc reads the keys back instantly.
		mkdir("/media/fat/dvdcss/cache", 0755);
		setenv("DVDCSS_CACHE", "/media/fat/dvdcss/cache", 1);

		// Default method: fetch each title key from the drive via the CSS ioctls
		// (instant) when a region is set, falling back to cracking otherwise.
		css = p_open(dev);
		if (!css)
		{
			css_log("dvdcss_open(%s) failed", dev);
			return 0;
		}
	}
	else
	{
		// No libdvdcss: raw fallback. Unencrypted DVDs play; a CSS disc can't be
		// descrambled here — and many drives won't even hand over the scrambled
		// VOB sectors without auth, so the core would just sit black with no data
		// to trip its CSS ENCRYPTED detector. Detect CSS up front and say so.
		if (disc_is_encrypted(dev))
		{
			css_log("encrypted disc but no libdvdcss — run install_dvdcss");
			// InfoMessage only renders when menustate <= MENU_INFO (core idle), but
			// the mount runs mid-launch when the menu FSM is in a high state, so a
			// call here is dropped. Defer it: dvd_css_tick() (from user_io_poll)
			// re-asserts it until the launch settles to MENU_NONE1 and it shows.
			warn_until = time(NULL) + 8;
		}
		raw_fd = open(dev, O_RDONLY | O_CLOEXEC);
		if (raw_fd < 0)
		{
			css_log("cannot open %s", dev);
			return 0;
		}
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
	        css ? "" : " raw — no libdvdcss, unencrypted discs only");

	// Discover the VOB layout and pre-crack every title key at its VOB start.
	if (css) build_vob_list();
	return 1;
}

int dvd_css_active(void)
{
	return css != NULL || raw_fd >= 0;
}

// Called every user_io_poll. Re-asserts the "encrypted disc, install libdvdcss"
// popup ~once/second until its window closes — InfoMessage no-ops while the
// launch FSM is busy and renders once menustate settles to idle (core running).
void dvd_css_tick(void)
{
	if (!warn_until) return;
	time_t now = time(NULL);
	if (now >= warn_until) { warn_until = 0; return; }

	static time_t last = 0;
	if (now != last)   // at most once per second — the 2 s InfoMessage timeout bridges the gap
	{
		last = now;
		InfoMessage("Encrypted DVD\n\nRun install_dvdcss", 2000, "DVD");
	}
}

uint64_t dvd_css_size(void)
{
	return (css || raw_fd >= 0) ? css_size : 0;
}

int dvd_css_read(void *buf, uint32_t lba, uint32_t count)
{
	if (raw_fd >= 0) return raw_read10(raw_fd, lba, buf, (int)count);   // no-libdvdcss fallback
	if (!css) return -1;

	int vi = vob_index(lba);
	int decrypt = (vi >= 0);

	if (decrypt)
	{
		// Don't let one read span two titles (different keys): clamp to this VOB.
		uint32_t vob_end = g_vobs[vi].start + g_vobs[vi].nsec;
		if (lba + count > vob_end) count = vob_end - lba;

		// Select this VOB's (pre-cracked) title key. SEEK_KEY is a fast cached
		// lookup now; only needed on a title change or a discontinuity.
		if (vi != cur_vob || (int)lba != css_pos)
		{
			if (p_seek(css, (int)lba, DVDCSS_SEEK_KEY) < 0)
			{
				if (p_seek(css, (int)lba, DVDCSS_NOFLAGS) < 0)
				{
					static int sf = 0;
					if (sf < 10) { sf++; css_log("seek %u failed: %s", lba, p_error ? p_error(css) : "?"); }
					css_pos = -1; cur_vob = -1; return -1;
				}
				decrypt = 0;   // no key -> read raw rather than corrupt
			}
			cur_vob = vi;
		}
	}
	else
	{
		// Filesystem/IFO sector: raw positioning, NEVER decrypt (would corrupt it).
		if ((int)lba != css_pos)
		{
			if (p_seek(css, (int)lba, DVDCSS_NOFLAGS) < 0)
			{
				static int sf2 = 0;
				if (sf2 < 10) { sf2++; css_log("seek %u failed: %s", lba, p_error ? p_error(css) : "?"); }
				css_pos = -1; return -1;
			}
		}
		cur_vob = -1;
	}

	int n = p_read(css, buf, (int)count, decrypt ? DVDCSS_READ_DECRYPT : DVDCSS_NOFLAGS);
	if (n < 0)
	{
		static int rf = 0;
		if (rf < 10) { rf++; css_log("read %u@%u failed: %s", count, lba, p_error ? p_error(css) : "?"); }
		css_pos = -1;
		return -1;
	}

	css_pos = (int)(lba + n);
	return n;
}

void dvd_css_close(void)
{
	if (css && p_close) p_close(css);
	if (raw_fd >= 0) close(raw_fd);
	css = NULL;
	raw_fd = -1;
	css_pos = -1;
	cur_vob = -1;
	g_nvobs = 0;
	css_size = 0;
	warn_until = 0;
}
