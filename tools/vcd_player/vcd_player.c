// vcd_player - Video CD playback for MiSTer Physical Disc Support.
//
// Standalone framebuffer application launched by MiSTer Main (graphical script
// path) when a Video CD is inserted, or manually from Scripts/video_cd.sh.
//
// Video CDs carry MPEG-1 program streams (MPEG-1 video + MPEG-1 Layer II audio)
// in Mode 2 Form 2 sectors on tracks 2..N. None of the console cores contain an
// MPEG-1 decoder, so the stream is decoded in software on the HPS (pl_mpeg) and
// drawn to the Linux framebuffer that Main routes to the video output while the
// application runs. Audio is sent to the MiSTer ALSA output through aplay.
//
// Video CDs have no region lock: PAL (352x288 @ 25 fps) and NTSC
// (352x240 @ 29.97 fps) discs are both handled by the same decoder path.
//
// Usage: vcd_player [device]   (default: first /dev/srN that opens)
//
// Controls (gamepad buttons of the MiSTer menu mapping): any button shows the
// controls and a progress bar at the bottom of the screen for 10 seconds.
//   Start = play, Select = pause, B = stop, D-pad right/left = next/previous
//   chapter, R/L = fast forward/rewind 10x, X then A = exit to the menu.
// The OSD/Menu button still stops the player at once, like any other
// graphical script (MiSTer Main kills the process by name).

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <linux/cdrom.h>
#include <linux/fb.h>
#include <linux/input.h>
#include <scsi/sg.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <ucontext.h>

#define PL_MPEG_IMPLEMENTATION
#include "pl_mpeg.h"

#define RAW_SECTOR 2352
#define FORM2_DATA 2324
#define READ_BATCH 32
#define VCD_INFO_LBA 150
#define MAX_TRACKS 99

static volatile sig_atomic_t quit_requested = 0;

// Last step reached, reported if the player crashes (see on_crash).
static const char *volatile stage = "start";
#define STAGE(name) do { stage = (name); fprintf(stderr, "vcd_player: %s\n", stage); } while (0)

static void on_crash(int sig, siginfo_t *info, void *ctx)
{
	char msg[200];
	unsigned long pc = 0, lr = 0;
#if defined(__arm__)
	ucontext_t *uc = (ucontext_t *)ctx;
	pc = uc->uc_mcontext.arm_pc;
	lr = uc->uc_mcontext.arm_lr;
#else
	(void)ctx;
#endif
	int n = snprintf(msg, sizeof(msg), "vcd_player: crashed (signal %d) during: %s pc=%08lx lr=%08lx addr=%p\n",
		sig, stage, pc, lr, info ? info->si_addr : NULL);
	if (n > 0 && write(2, msg, n) < 0) n = 0;
	_exit(128 + sig);
}

static void on_signal(int sig)
{
	(void)sig;
	quit_requested = 1;
}

// ---------------------------------------------------------------------------
// Optical drive access
// ---------------------------------------------------------------------------

typedef struct
{
	int start;
	int end; // exclusive
	int data;
} track_t;

static int cd_fd = -1;
static track_t tracks[MAX_TRACKS + 1];
static int track_first = 0, track_last = 0;

static int open_drive(const char *dev)
{
	if (dev && *dev) return open(dev, O_RDONLY | O_NONBLOCK | O_CLOEXEC);

	for (int i = 0; i < 8; i++)
	{
		char path[32];
		snprintf(path, sizeof(path), "/dev/sr%d", i);
		int fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
		if (fd >= 0) return fd;
	}
	return -1;
}

static int read_toc(void)
{
	struct cdrom_tochdr hdr;
	if (ioctl(cd_fd, CDROMREADTOCHDR, &hdr)) return -1;
	track_first = hdr.cdth_trk0;
	track_last = hdr.cdth_trk1;
	if (track_first < 1 || track_last > MAX_TRACKS || track_first > track_last) return -1;

	for (int t = track_first; t <= track_last + 1; t++)
	{
		struct cdrom_tocentry e;
		memset(&e, 0, sizeof(e));
		e.cdte_track = (t > track_last) ? CDROM_LEADOUT : t;
		e.cdte_format = CDROM_LBA;
		if (ioctl(cd_fd, CDROMREADTOCENTRY, &e)) return -1;
		int idx = (t > track_last) ? 0 : t;
		tracks[idx].start = e.cdte_addr.lba;
		tracks[idx].data = (e.cdte_ctrl & CDROM_DATA_TRACK) != 0;
	}

	for (int t = track_first; t <= track_last; t++)
	{
		tracks[t].end = (t < track_last) ? tracks[t + 1].start : tracks[0].start;
	}
	return 0;
}

// READ CD (0xBE) returning full 2352-byte sectors of any type.
static int read_raw_sg(int lba, int count, uint8_t *dst)
{
	uint8_t cdb[12] = { 0xBE, 0 };
	uint8_t sense[32];
	sg_io_hdr_t io;

	cdb[2] = (lba >> 24) & 0xFF;
	cdb[3] = (lba >> 16) & 0xFF;
	cdb[4] = (lba >> 8) & 0xFF;
	cdb[5] = lba & 0xFF;
	cdb[6] = (count >> 16) & 0xFF;
	cdb[7] = (count >> 8) & 0xFF;
	cdb[8] = count & 0xFF;
	cdb[9] = 0xF8; // sync + all headers + user data + EDC/ECC

	memset(&io, 0, sizeof(io));
	io.interface_id = 'S';
	io.cmd_len = sizeof(cdb);
	io.cmdp = cdb;
	io.dxfer_direction = SG_DXFER_FROM_DEV;
	io.dxferp = dst;
	io.dxfer_len = count * RAW_SECTOR;
	io.sbp = sense;
	io.mx_sb_len = sizeof(sense);
	io.timeout = 10000;

	if (ioctl(cd_fd, SG_IO, &io) < 0) return -1;
	if ((io.info & SG_INFO_OK_MASK) != SG_INFO_OK) return -1;
	return 0;
}

// Fallback for bridges that reject SG_IO pass-through.
static int read_raw_ioctl(int lba, uint8_t *dst)
{
	union
	{
		struct cdrom_msf msf;
		uint8_t buf[CD_FRAMESIZE_RAW];
	} u;
	int a = lba + CD_MSF_OFFSET;
	memset(&u, 0, sizeof(u));
	u.msf.cdmsf_min0 = a / (CD_SECS * CD_FRAMES);
	u.msf.cdmsf_sec0 = (a / CD_FRAMES) % CD_SECS;
	u.msf.cdmsf_frame0 = a % CD_FRAMES;
	if (ioctl(cd_fd, CDROMREADRAW, &u)) return -1;
	memcpy(dst, u.buf, RAW_SECTOR);
	return 0;
}

static int read_raw(int lba, int count, uint8_t *dst)
{
	for (int attempt = 0; attempt < 3 && !quit_requested; attempt++)
	{
		if (!read_raw_sg(lba, count, dst)) return 0;
	}

	for (int i = 0; i < count; i++)
	{
		if (read_raw_ioctl(lba + i, dst + i * RAW_SECTOR)) memset(dst + i * RAW_SECTOR, 0, RAW_SECTOR);
	}
	return 0;
}

// Returns the user data of a raw sector and its length.
static const uint8_t *sector_payload(const uint8_t *raw, int *len)
{
	switch (raw[15])
	{
	case 1:
		*len = 2048;
		return raw + 16;
	case 2:
		// Mode 2 XA: sub-header byte 2 bit 5 selects Form 2.
		*len = (raw[18] & 0x20) ? FORM2_DATA : 2048;
		return raw + 24;
	default:
		*len = 0;
		return raw;
	}
}

static const char *detect_vcd_kind(void)
{
	uint8_t raw[RAW_SECTOR];
	int len = 0;
	if (read_raw(VCD_INFO_LBA, 1, raw)) return NULL;
	const uint8_t *p = sector_payload(raw, &len);
	if (len < 8) return NULL;
	if (!memcmp(p, "VIDEO_CD", 8)) return "Video CD";
	if (!memcmp(p, "SUPERVCD", 8)) return "Super Video CD";
	if (!memcmp(p, "HQ-VCD  ", 8)) return "HQ Video CD";
	return NULL;
}

// ---------------------------------------------------------------------------
// MPEG stream feed
// ---------------------------------------------------------------------------

// Each track is presented to pl_mpeg as a seekable file made of the 2324-byte
// user data of its sectors (Form 1 sectors are padded with zeros). Seeking lets
// pl_mpeg find the duration, jump to a time (chapters, fast forward/rewind)
// and resynchronise audio after a jump.
typedef struct
{
	int track;
	size_t pos;  // stream offset of the next byte to load
	size_t size; // stream size of the whole track
	uint8_t raw[READ_BATCH * RAW_SECTOR];
	uint8_t block[FORM2_DATA];
} feed_t;

static void sector_block(const uint8_t *raw, uint8_t *block)
{
	int len = 0;
	const uint8_t *p = sector_payload(raw, &len);
	if (len) memcpy(block, p, len);
	if (len < FORM2_DATA) memset(block + len, 0, FORM2_DATA - len);
}

static void feed_load(plm_buffer_t *buf, void *user)
{
	feed_t *f = (feed_t *)user;
	const track_t *t = &tracks[f->track];

	if (buf->discard_read_bytes) plm_buffer_discard_read_bytes(buf);

	size_t room = buf->capacity - buf->length;
	if (f->pos >= f->size || quit_requested)
	{
		buf->has_ended = TRUE;
		return;
	}
	if (!room) return;

	int first = (int)(f->pos / FORM2_DATA);
	int count = (t->end - t->start) - first;
	if (count > READ_BATCH) count = READ_BATCH;
	int need = (int)((room + f->pos % FORM2_DATA + FORM2_DATA - 1) / FORM2_DATA);
	if (count > need) count = need;
	if (count <= 0)
	{
		buf->has_ended = TRUE;
		return;
	}

	read_raw(t->start + first, count, f->raw);
	for (int i = 0; i < count && room; i++)
	{
		sector_block(f->raw + i * RAW_SECTOR, f->block);
		size_t off = f->pos % FORM2_DATA;
		size_t n = FORM2_DATA - off;
		if (n > room) n = room;
		memcpy(buf->bytes + buf->length, f->block + off, n);
		buf->length += n;
		room -= n;
		f->pos += n;
	}
}

static void feed_seek(plm_buffer_t *buf, size_t offset, void *user)
{
	(void)buf;
	feed_t *f = (feed_t *)user;
	f->pos = offset < f->size ? offset : f->size;
}

static size_t feed_tell(plm_buffer_t *buf, void *user)
{
	(void)buf;
	return ((feed_t *)user)->pos;
}

// ---------------------------------------------------------------------------
// Framebuffer output
// ---------------------------------------------------------------------------

static uint8_t *fb_mem = NULL;
static size_t fb_len = 0;
static int fb_w, fb_h, fb_stride, fb_bgr;
static uint8_t *frame_rgb = NULL;
static uint32_t *line_buf = NULL;
static int *x_map = NULL;
static int dst_x, dst_y, dst_w, dst_h, src_w, src_h;

static int fb_open(void)
{
	int fd = open("/dev/fb0", O_RDWR | O_CLOEXEC);
	if (fd < 0) return -1;

	struct fb_var_screeninfo var;
	struct fb_fix_screeninfo fix;
	if (ioctl(fd, FBIOGET_VSCREENINFO, &var) || ioctl(fd, FBIOGET_FSCREENINFO, &fix) || var.bits_per_pixel != 32)
	{
		close(fd);
		return -1;
	}

	fb_w = var.xres;
	fb_h = var.yres;
	fb_stride = fix.line_length;
	fb_bgr = var.red.offset == 16; // byte order B,G,R,A in memory
	fb_len = (size_t)fb_stride * fb_h;
	fprintf(stderr, "vcd_player: fb %dx%d stride %d bpp %d smem %u red.offset %d\n",
		fb_w, fb_h, fb_stride, var.bits_per_pixel, fix.smem_len, var.red.offset);
	// Never touch memory beyond what the driver reports for the framebuffer.
	if (fix.smem_len && fb_len > fix.smem_len)
	{
		fb_h = fix.smem_len / fb_stride;
		fb_len = (size_t)fb_stride * fb_h;
	}
	if (!fb_w || !fb_h || fb_stride < fb_w * 4)
	{
		close(fd);
		return -1;
	}
	fb_mem = (uint8_t *)mmap(NULL, fb_len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	close(fd);
	if (fb_mem == MAP_FAILED)
	{
		fb_mem = NULL;
		return -1;
	}

	memset(fb_mem, 0, fb_len);
	return 0;
}

static int have_frame = 0;

// On-screen controls: a band at the bottom of the screen, blended over the
// picture. ovl_a is the coverage and ovl_v the grey level of each pixel.
static int ovl_y = 0, ovl_h = 0, ovl_shown = 0;
static uint8_t *ovl_a = NULL, *ovl_v = NULL;
static uint32_t *full_line = NULL;

static void fb_layout(int w, int h)
{
	if (w == src_w && h == src_h) return;
	src_w = w;
	src_h = h;

	// Video CD pictures are 4:3 regardless of the PAL/NTSC storage size.
	dst_h = fb_h;
	dst_w = fb_h * 4 / 3;
	if (dst_w > fb_w)
	{
		dst_w = fb_w;
		dst_h = fb_w * 3 / 4;
	}
	dst_x = (fb_w - dst_w) / 2;
	dst_y = (fb_h - dst_h) / 2;

	free(frame_rgb);
	free(line_buf);
	free(x_map);
	frame_rgb = (uint8_t *)malloc((size_t)w * h * 4);
	line_buf = (uint32_t *)malloc((size_t)dst_w * 4);
	x_map = (int *)malloc((size_t)dst_w * sizeof(int));
	for (int x = 0; x < dst_w; x++) x_map[x] = x * w / dst_w;

	memset(fb_mem, 0, fb_len);
}

static inline uint32_t blend_px(uint32_t px, uint8_t v, uint8_t a)
{
	uint32_t ia = 255 - a, va = (uint32_t)v * a + 128;
	uint32_t c0 = (px & 0xFF) * ia + va;
	uint32_t c1 = ((px >> 8) & 0xFF) * ia + va;
	uint32_t c2 = ((px >> 16) & 0xFF) * ia + va;
	// x / 255 without a division
	c0 = (c0 + (c0 >> 8)) >> 8;
	c1 = (c1 + (c1 >> 8)) >> 8;
	c2 = (c2 + (c2 >> 8)) >> 8;
	return 0xFF000000u | (c2 << 16) | (c1 << 8) | c0;
}

// Scaled video line for framebuffer row y, or NULL outside the picture.
static const uint32_t *video_line(int y, int *prev_sy)
{
	if (!have_frame || !frame_rgb || !line_buf || y < dst_y || y >= dst_y + dst_h) return NULL;
	int sy = (y - dst_y) * src_h / dst_h;
	if (sy != *prev_sy)
	{
		const uint32_t *src = (const uint32_t *)(frame_rgb + (size_t)sy * src_w * 4);
		for (int x = 0; x < dst_w; x++) line_buf[x] = src[x_map[x]] | 0xFF000000u;
		*prev_sy = sy;
	}
	return line_buf;
}

// Draws the last decoded picture and, in the bottom band, the controls.
static void fb_present(void)
{
	int prev_sy = -1;
	for (int y = 0; y < fb_h; y++)
	{
		const uint32_t *line = video_line(y, &prev_sy);
		uint8_t *row = fb_mem + (size_t)y * fb_stride;

		if (y >= ovl_y && full_line)
		{
			// Bottom band: always rewrite the whole row so the controls also
			// disappear from the black borders when they are hidden.
			memset(full_line, 0, (size_t)fb_w * 4);
			if (line) memcpy(full_line + dst_x, line, (size_t)dst_w * 4);
			if (ovl_shown)
			{
				const uint8_t *a = ovl_a + (size_t)(y - ovl_y) * fb_w;
				const uint8_t *v = ovl_v + (size_t)(y - ovl_y) * fb_w;
				for (int x = 0; x < fb_w; x++)
				{
					if (a[x]) full_line[x] = blend_px(full_line[x], v[x], a[x]);
				}
			}
			memcpy(row, full_line, (size_t)fb_w * 4);
		}
		else if (line)
		{
			memcpy(row + (size_t)dst_x * 4, line, (size_t)dst_w * 4);
		}
	}
}

static void fb_draw(plm_frame_t *frame)
{
	fb_layout(frame->width, frame->height);
	if (!frame_rgb || !line_buf || !x_map) return;

	if (fb_bgr) plm_frame_to_bgra(frame, frame_rgb, src_w * 4);
	else plm_frame_to_rgba(frame, frame_rgb, src_w * 4);
	have_frame = 1;
	fb_present();
}

// ---------------------------------------------------------------------------
// On-screen controls
// ---------------------------------------------------------------------------

#include "vcd_font.h"

#define OSD_TIMEOUT 10.0

static int ui = 1; // scale: 1 for 640 wide, 2 for 1280, 3 for 1920

static void ovl_px(int x, int y, uint8_t v, uint8_t a)
{
	if (x < 0 || x >= fb_w || y < 0 || y >= ovl_h || !a) return;
	size_t i = (size_t)y * fb_w + x;
	unsigned da = ovl_a[i];
	unsigned under = da * (255 - a) / 255;
	unsigned ra = a + under;
	ovl_v[i] = (uint8_t)((v * a + ovl_v[i] * under) / ra);
	ovl_a[i] = (uint8_t)ra;
}

static void ovl_rect(int x, int y, int w, int h, uint8_t v, uint8_t a)
{
	for (int j = 0; j < h; j++)
		for (int i = 0; i < w; i++) ovl_px(x + i, y + j, v, a);
}

static void ovl_frame(int x, int y, int w, int h, int t, uint8_t v, uint8_t a)
{
	ovl_rect(x + t, y, w - 2 * t, t, v, a);
	ovl_rect(x + t, y + h - t, w - 2 * t, t, v, a);
	ovl_rect(x, y + t, t, h - 2 * t, v, a);
	ovl_rect(x + w - t, y + t, t, h - 2 * t, v, a);
}

#define TEXT_A 235
#define SHADE_A 120
#define GLYPH_W 8

static int text_width(const char *s)
{
	return (int)strlen(s) * GLYPH_W * ui;
}

static void ovl_glyphs(int x, int y, const char *s, uint8_t v, uint8_t a)
{
	for (; *s; s++, x += GLYPH_W * ui)
	{
		const unsigned char *g = vcd_font[(unsigned char)*s & 0x7F];
		for (int c = 0; c < 8; c++)
			for (int r = 0; r < 8; r++)
				if ((g[c] >> r) & 1) ovl_rect(x + c * ui, y + r * ui, ui, ui, v, a);
	}
}

// White text with a soft dark shadow so it stays readable on bright scenes.
static int ovl_text(int x, int y, const char *s)
{
	int d = ui > 1 ? ui / 2 : 1;
	ovl_glyphs(x + d, y + d, s, 0, SHADE_A);
	ovl_glyphs(x, y, s, 255, TEXT_A);
	return x + text_width(s);
}

// Filled triangle pointing right (dir > 0) or left, h pixels tall.
static void ovl_triangle(int x, int y, int h, int dir, uint8_t v, uint8_t a)
{
	int w = h * 3 / 4;
	for (int i = 0; i < w; i++)
	{
		int col = dir > 0 ? i : w - 1 - i;
		int half = (w - i) * h / (2 * w);
		ovl_rect(x + col, y + h / 2 - half, 1, 2 * half, v, a);
	}
}

enum { ST_PLAY, ST_PAUSE, ST_STOP, ST_FF, ST_REW };

static int icon_width(int state, int h)
{
	switch (state)
	{
	case ST_FF:
	case ST_REW: return h * 3 / 2;
	default: return h * 3 / 4;
	}
}

static void ovl_icon(int x, int y, int h, int state, uint8_t v, uint8_t a)
{
	int w = h * 3 / 4;
	switch (state)
	{
	case ST_PLAY: ovl_triangle(x, y, h, 1, v, a); break;
	case ST_PAUSE:
		ovl_rect(x, y, w / 3, h, v, a);
		ovl_rect(x + w - w / 3, y, w / 3, h, v, a);
		break;
	case ST_STOP: ovl_rect(x, y + (h - w) / 2, w, w, v, a); break;
	case ST_FF:
		ovl_triangle(x, y, h, 1, v, a);
		ovl_triangle(x + w, y, h, 1, v, a);
		break;
	case ST_REW:
		ovl_triangle(x, y, h, -1, v, a);
		ovl_triangle(x + w, y, h, -1, v, a);
		break;
	}
}

static void icon_with_shadow(int x, int y, int h, int state)
{
	int d = ui > 1 ? ui / 2 : 1;
	ovl_icon(x + d, y + d, h, state, 0, SHADE_A);
	ovl_icon(x, y, h, state, 255, TEXT_A);
}

// Button name in an outlined box followed by its action.
static int legend_item(int x, int y, const char *button, const char *label, int draw)
{
	int pad = 3 * ui;
	int bw = text_width(button) + 2 * pad;
	int bh = 8 * ui + 2 * pad;
	if (draw)
	{
		ovl_frame(x + 1, y + 1, bw, bh, ui, 0, SHADE_A / 2);
		ovl_rect(x, y, bw, bh, 255, 40);
		ovl_frame(x, y, bw, bh, ui, 255, 190);
		ovl_text(x + pad, y + pad, button);
		ovl_text(x + bw + 3 * ui, y + pad, label);
	}
	return bw + 3 * ui + text_width(label);
}

static const char *const legend[][2] =
{
	{ "START", "Play" },
	{ "SELECT", "Pausa" },
	{ "B", "Stop" },
	{ "\x10 \x11", "Capitolo" },
	{ "L", "\x10\x10 10x" },
	{ "R", "\x11\x11 10x" },
	{ "X", "Esci" },
};

static void legend_row(int y)
{
	int n = sizeof(legend) / sizeof(legend[0]);
	int total = 0;
	for (int i = 0; i < n; i++) total += legend_item(0, 0, legend[i][0], legend[i][1], 0);
	int avail = fb_w - 2 * 12 * ui;
	int gap = n > 1 ? (avail - total) / (n - 1) : 0;
	if (gap > 14 * ui) gap = 14 * ui;
	if (gap < 2 * ui) gap = 2 * ui;
	int x = (fb_w - (total + gap * (n - 1))) / 2;
	for (int i = 0; i < n; i++) x += legend_item(x, y, legend[i][0], legend[i][1], 1) + gap;
}

static void format_time(double t, char *out, size_t len)
{
	if (!(t > 0)) t = 0;
	int s = (int)t;
	if (s >= 3600) snprintf(out, len, "%d:%02d:%02d", s / 3600, (s / 60) % 60, s % 60);
	else snprintf(out, len, "%02d:%02d", s / 60, s % 60);
}

typedef struct
{
	int state;
	int confirm_exit;
	double time;
	double duration;
	int chapter;  // 1-based, 0 when unknown
	int chapters;
} osd_info_t;

static void osd_layout(void)
{
	ui = fb_w / 640;
	if (ui < 1) ui = 1;
	// From the bottom: margin, legend row, progress bar, status line.
	int legend_h = 8 * ui + 6 * ui;
	int legend_y = fb_h - 10 * ui - legend_h;
	int bar_y = legend_y - 10 * ui;
	int status_y = bar_y - 6 * ui - 8 * ui;
	ovl_y = status_y - 4 * ui;
	if (ovl_y < 0) ovl_y = 0;
	ovl_h = fb_h - ovl_y;
	ovl_a = (uint8_t *)calloc((size_t)fb_w * ovl_h, 1);
	ovl_v = (uint8_t *)calloc((size_t)fb_w * ovl_h, 1);
	full_line = (uint32_t *)malloc((size_t)fb_w * 4);
	if (!ovl_a || !ovl_v || !full_line) ovl_y = fb_h;
}

static void osd_build(const osd_info_t *info)
{
	memset(ovl_a, 0, (size_t)fb_w * ovl_h);
	memset(ovl_v, 0, (size_t)fb_w * ovl_h);

	int legend_h = 8 * ui + 6 * ui;
	int legend_y = fb_h - 10 * ui - legend_h - ovl_y;
	int bar_y = legend_y - 10 * ui;
	int status_y = bar_y - 6 * ui - 8 * ui;
	int mx = 24 * ui;
	char buf[64], t1[16], t2[16];

	// A light shade towards the bottom keeps the white readable on bright
	// pictures while leaving the image visible.
	for (int y = 0; y < ovl_h; y++) memset(ovl_a + (size_t)y * fb_w, 100 * y / ovl_h, fb_w);

	// Status line: state icon and name, chapter, time.
	static const char *const names[] = { "PLAY", "PAUSA", "STOP", "AVANTI 10x", "INDIETRO 10x" };
	int ih = 8 * ui;
	icon_with_shadow(mx, status_y, ih, info->state);
	int x = ovl_text(mx + icon_width(info->state, ih) + 4 * ui, status_y, names[info->state]);
	if (info->chapters > 0 && info->chapter > 0)
	{
		snprintf(buf, sizeof(buf), "Capitolo %d/%d", info->chapter, info->chapters);
		ovl_text(x + 16 * ui, status_y, buf);
	}
	format_time(info->time, t1, sizeof(t1));
	format_time(info->duration, t2, sizeof(t2));
	snprintf(buf, sizeof(buf), "%s / %s", t1, t2);
	ovl_text(fb_w - mx - text_width(buf), status_y, buf);

	// Progress bar.
	int bw = fb_w - 2 * mx;
	int bh = 3 * ui;
	double f = info->duration > 0 ? info->time / info->duration : 0;
	if (f < 0) f = 0;
	if (f > 1) f = 1;
	int fill = (int)(bw * f);
	ovl_rect(mx + 1, bar_y + 1, bw, bh, 0, SHADE_A / 2);
	ovl_rect(mx, bar_y, bw, bh, 255, 80);
	ovl_rect(mx, bar_y, fill, bh, 255, TEXT_A);
	ovl_rect(mx + fill - ui, bar_y - 2 * ui, 3 * ui, bh + 4 * ui, 255, TEXT_A);

	// Button legend, or the exit confirmation.
	if (info->confirm_exit)
	{
		const char *q = "Uscire dal Video CD?";
		int w = text_width(q) + 16 * ui + legend_item(0, 0, "A", "Conferma", 0) + 16 * ui + text_width("Altro tasto: Annulla");
		x = (fb_w - w) / 2;
		x = ovl_text(x, legend_y + 3 * ui, q) + 16 * ui;
		x += legend_item(x, legend_y, "A", "Conferma", 1) + 16 * ui;
		ovl_text(x, legend_y + 3 * ui, "Altro tasto: Annulla");
	}
	else
	{
		legend_row(legend_y);
	}
}

// ---------------------------------------------------------------------------
// Controller input
// ---------------------------------------------------------------------------

// While a graphical application runs, MiSTer Main turns the buttons of the
// menu mapping into key presses on its virtual keyboard: A = Enter, B = Esc,
// X = Tab, Y = Space, L = Page Up, R = Page Down, D-pad = arrows,
// Start = Play, Select = Pause. Only that device is read, so the raw gamepad
// codes (which differ between pads) never matter here.
#define MISTER_UINPUT_NAME "MiSTer virtual input"

static int input_fd = -1;
static double input_retry = 0;

static void input_open(void)
{
	for (int i = 0; i < 32 && input_fd < 0; i++)
	{
		char path[32], name[64] = { 0 };
		snprintf(path, sizeof(path), "/dev/input/event%d", i);
		int fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
		if (fd < 0) continue;
		if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1), name) >= 0 && !strcmp(name, MISTER_UINPUT_NAME))
		{
			input_fd = fd;
			fprintf(stderr, "vcd_player: controls on %s\n", path);
		}
		else close(fd);
	}
}

// Next key press, 0 when there is none.
static int input_key(double now)
{
	if (input_fd < 0)
	{
		if (now < input_retry) return 0;
		input_retry = now + 2.0;
		input_open();
		if (input_fd < 0) return 0;
	}

	struct input_event ev;
	for (;;)
	{
		ssize_t n = read(input_fd, &ev, sizeof(ev));
		if (n != (ssize_t)sizeof(ev))
		{
			if (n < 0 && errno != EAGAIN && errno != EINTR)
			{
				close(input_fd);
				input_fd = -1;
			}
			return 0;
		}
		if (ev.type == EV_KEY && ev.value == 1) return ev.code;
	}
}

// ---------------------------------------------------------------------------
// Audio output (aplay -> MiSTer ALSA)
// ---------------------------------------------------------------------------

#define AUDIO_RING (1 << 17) // stereo frames

static FILE *audio_pipe = NULL;
static int16_t audio_ring[AUDIO_RING * 2];
static volatile unsigned audio_rd = 0, audio_wr = 0;
static pthread_t audio_thread;
static volatile int audio_running = 0;
static pthread_mutex_t audio_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t audio_cond = PTHREAD_COND_INITIALIZER;

static void *audio_main(void *arg)
{
	(void)arg;
	int16_t chunk[2048 * 2];

	while (audio_running)
	{
		pthread_mutex_lock(&audio_lock);
		while (audio_running && audio_rd == audio_wr) pthread_cond_wait(&audio_cond, &audio_lock);
		unsigned n = 0;
		while (audio_rd != audio_wr && n < 2048)
		{
			unsigned i = audio_rd % AUDIO_RING;
			chunk[n * 2] = audio_ring[i * 2];
			chunk[n * 2 + 1] = audio_ring[i * 2 + 1];
			audio_rd++;
			n++;
		}
		pthread_mutex_unlock(&audio_lock);

		if (n && fwrite(chunk, 4, n, audio_pipe) != n) break;
	}
	return NULL;
}

static int audio_open(int samplerate)
{
	if (access("/usr/bin/aplay", X_OK) && access("/bin/aplay", X_OK))
	{
		fprintf(stderr, "vcd_player: aplay not found, playing without audio\n");
		return -1;
	}

	char cmd[128];
	snprintf(cmd, sizeof(cmd), "aplay -q -t raw -f S16_LE -c 2 -r %d - 2>/dev/null", samplerate);
	audio_pipe = popen(cmd, "w");
	if (!audio_pipe) return -1;

	audio_running = 1;
	if (pthread_create(&audio_thread, NULL, audio_main, NULL))
	{
		audio_running = 0;
		pclose(audio_pipe);
		audio_pipe = NULL;
		return -1;
	}
	return 0;
}

static void audio_close(void)
{
	if (!audio_pipe) return;
	pthread_mutex_lock(&audio_lock);
	audio_running = 0;
	pthread_cond_signal(&audio_cond);
	pthread_mutex_unlock(&audio_lock);
	pthread_join(audio_thread, NULL);
	pclose(audio_pipe);
	audio_pipe = NULL;
}

static int16_t to_s16(float v)
{
	if (v > 1.0f) v = 1.0f;
	if (v < -1.0f) v = -1.0f;
	return (int16_t)(v * 32767.0f);
}

static void audio_flush(void)
{
	pthread_mutex_lock(&audio_lock);
	audio_rd = audio_wr;
	pthread_mutex_unlock(&audio_lock);
}

// ---------------------------------------------------------------------------
// Chapters
// ---------------------------------------------------------------------------

// Chapters are the entry points of ENTRIES.VCD (LBA 151): one per track on a
// simple disc, more on discs authored with chapters. Without that file the
// MPEG tracks themselves are the chapters.
#define ENTRIES_LBA 151
#define MAX_ENTRIES 500

typedef struct
{
	int track;
	int lba;
} entry_t;

static entry_t entries[MAX_ENTRIES];
static int num_entries = 0;

static int from_bcd(uint8_t v)
{
	return (v >> 4) * 10 + (v & 0x0F);
}

static void read_entries(void)
{
	uint8_t raw[RAW_SECTOR];
	int len = 0;
	read_raw(ENTRIES_LBA, 1, raw);
	const uint8_t *p = sector_payload(raw, &len);
	if (len >= 12 && (!memcmp(p, "ENTRYVCD", 8) || !memcmp(p, "ENTRYSVD", 8)))
	{
		int n = (p[10] << 8) | p[11];
		if (n > MAX_ENTRIES) n = MAX_ENTRIES;
		for (int i = 0; i < n && 12 + 4 * i + 4 <= len; i++)
		{
			const uint8_t *e = p + 12 + 4 * i;
			int track = from_bcd(e[0]);
			int lba = (from_bcd(e[1]) * 60 + from_bcd(e[2])) * 75 + from_bcd(e[3]) - 150;
			if (track < 2 || track > track_last || !tracks[track].data) continue;
			if (lba < tracks[track].start || lba >= tracks[track].end) continue;
			if (num_entries && lba <= entries[num_entries - 1].lba) continue;
			entries[num_entries].track = track;
			entries[num_entries].lba = lba;
			num_entries++;
		}
	}

	if (!num_entries)
	{
		for (int t = 2; t <= track_last && num_entries < MAX_ENTRIES; t++)
		{
			if (!tracks[t].data) continue;
			entries[num_entries].track = t;
			entries[num_entries].lba = tracks[t].start;
			num_entries++;
		}
	}
	printf("vcd_player: %d chapter(s)\n", num_entries);
}

// ---------------------------------------------------------------------------
// Playback
// ---------------------------------------------------------------------------

static plm_frame_t *pending_frame = NULL;
static plm_t *plm = NULL;
static feed_t feed;
static int cur_track = 0;
static double cur_duration = 0;
static int cur_lead = 0; // empty sectors before the first MPEG pack of the track

static void on_video(plm_t *self, plm_frame_t *frame, void *user)
{
	(void)self;
	(void)user;
	// Only the most recent frame of a decode step is drawn, so a slow step
	// drops frames instead of falling further behind the audio.
	pending_frame = frame;
}

static void on_audio(plm_t *self, plm_samples_t *samples, void *user)
{
	(void)self;
	(void)user;
	if (!audio_pipe) return;

	pthread_mutex_lock(&audio_lock);
	for (unsigned i = 0; i < samples->count; i++)
	{
		if (audio_wr - audio_rd >= AUDIO_RING) audio_rd++; // overrun: drop oldest
		unsigned j = audio_wr % AUDIO_RING;
		audio_ring[j * 2] = to_s16(samples->interleaved[i * 2]);
		audio_ring[j * 2 + 1] = to_s16(samples->interleaved[i * 2 + 1]);
		audio_wr++;
	}
	pthread_cond_signal(&audio_cond);
	pthread_mutex_unlock(&audio_lock);
}

static double now_seconds(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec + ts.tv_nsec / 1e9;
}

static void close_track(void)
{
	if (plm) plm_destroy(plm);
	plm = NULL;
	cur_track = 0;
	cur_duration = 0;
}

static int track_lead(int track);

static int open_track(int track)
{
	close_track();
	memset(&feed, 0, sizeof(feed) - sizeof(feed.raw) - sizeof(feed.block));
	feed.track = track;
	feed.pos = 0;
	feed.size = (size_t)(tracks[track].end - tracks[track].start) * FORM2_DATA;

	plm_buffer_t *buf = plm_buffer_create_with_callbacks(feed_load, feed_seek, feed_tell, feed.size, &feed);
	if (!buf) return -1;
	STAGE("create MPEG decoder");
	plm = plm_create_with_buffer(buf, 1);
	if (!plm) return -1;
	// The stream comes from the drive: wait for the stream headers instead of
	// probing (plm_probe skips data it has not seen yet).
	STAGE("read MPEG headers");
	if (!plm_has_headers(plm))
	{
		fprintf(stderr, "vcd_player: track %d has no MPEG-1 stream\n", track);
		close_track();
		return -1;
	}

	// Video CD sectors carry one system header per stream type, so the first
	// header usually announces only the video stream and pl_mpeg would never
	// create the audio decoder. Every Video CD has exactly one MPEG-1 video
	// stream (0xE0) and one MPEG-1 Layer II audio stream (0xC0): declare both
	// and let pl_mpeg rebuild its decoders, as plm_probe() would.
	plm->demux->num_video_streams = 1;
	plm->demux->num_audio_streams = 1;
	plm->has_decoders = FALSE;
	plm->video_packet_type = 0;
	plm->audio_packet_type = 0;
	plm_init_decoders(plm);

	STAGE("open audio");
	plm_set_video_decode_callback(plm, on_video, NULL);
	if (plm_get_num_audio_streams(plm) > 0 && (audio_pipe || !audio_open(plm_get_samplerate(plm) > 0 ? plm_get_samplerate(plm) : 44100)))
	{
		plm_set_audio_decode_callback(plm, on_audio, NULL);
		plm_set_audio_lead_time(plm, 0.1);
	}
	else
	{
		plm_set_audio_enabled(plm, 0);
	}

	cur_track = track;
	cur_lead = track_lead(track);
	STAGE("read duration");
	cur_duration = plm_get_duration(plm);
	if (!(cur_duration > 0))
	{
		// Video CD streams are constant bit rate: 75 sectors per second.
		cur_duration = (tracks[track].end - tracks[track].start) / 75.0;
	}

	printf("vcd_player: track %d, %dx%d @ %.3f fps, audio %d Hz, %.0f s\n", track,
		plm_get_width(plm), plm_get_height(plm), plm_get_framerate(plm), plm_get_samplerate(plm), cur_duration);
	return 0;
}

// Video CD tracks start with a few empty sectors before the first MPEG pack.
static int track_lead(int track)
{
	const track_t *t = &tracks[track];
	for (int lba = t->start; lba < t->end && lba < t->start + 300; lba += READ_BATCH)
	{
		int count = t->end - lba;
		if (count > READ_BATCH) count = READ_BATCH;
		read_raw(lba, count, feed.raw);
		for (int i = 0; i < count; i++)
		{
			int len = 0;
			const uint8_t *p = sector_payload(feed.raw + i * RAW_SECTOR, &len);
			if (len >= 4 && !p[0] && !p[1] && p[2] == 1 && p[3] == 0xBA) return lba + i - t->start;
		}
	}
	return 0;
}

static int next_data_track(int track, int dir)
{
	for (int t = track + dir; t >= 2 && t <= track_last; t += dir)
	{
		if (tracks[t].data) return t;
	}
	return 0;
}

// A Video CD stream is played at exactly 75 sectors per second, the 1x CD
// speed, so disc position and playing time convert directly.
static int time_to_lba(double t)
{
	if (t < 0) t = 0;
	return tracks[cur_track].start + cur_lead + (int)(t * 75.0);
}

static double lba_to_time(int lba)
{
	return (lba - tracks[cur_track].start - cur_lead) / 75.0;
}

// Index of the chapter that contains lba, -1 before the first one.
static int chapter_at(int lba)
{
	int c = -1;
	for (int i = 0; i < num_entries && entries[i].lba <= lba; i++) c = i;
	return c;
}

static int seek_to(double t, int exact)
{
	audio_flush();
	if (t < 0) t = 0;
	if (t > cur_duration) t = cur_duration;
	return plm_seek(plm, t, exact);
}

static void goto_entry(int e)
{
	if (e < 0 || e >= num_entries) return;
	if (entries[e].track != cur_track && open_track(entries[e].track)) return;
	double t = lba_to_time(entries[e].lba);
	if (t < 0.5) t = 0;
	seek_to(t, TRUE);
	printf("vcd_player: chapter %d/%d\n", e + 1, num_entries);
}

typedef struct
{
	int state;
	int confirm_exit;
	double osd_until;
	double ff_time;
	double ff_step;
	int dirty;
} player_t;

static double play_time(const player_t *p)
{
	if (!plm) return 0;
	return (p->state == ST_FF || p->state == ST_REW) ? p->ff_time : plm_get_time(plm);
}

static int first_track(void)
{
	return num_entries ? entries[0].track : next_data_track(1, 1);
}

static void do_play(player_t *p)
{
	if (p->state == ST_STOP)
	{
		if (open_track(first_track())) return;
		audio_flush();
	}
	else if (p->state == ST_FF || p->state == ST_REW)
	{
		seek_to(p->ff_time, FALSE);
	}
	p->state = ST_PLAY;
}

static void do_stop(player_t *p)
{
	close_track();
	audio_flush();
	have_frame = 0;
	p->state = ST_STOP;
	memset(fb_mem, 0, fb_len);
	fb_present();
}

static void do_chapter(player_t *p, int dir)
{
	if (p->state == ST_STOP || !plm) return;
	int now = time_to_lba(play_time(p)) + 38;
	int c = chapter_at(now);
	int target;
	if (dir > 0)
	{
		target = c + 1;
		if (target >= num_entries) return;
	}
	else
	{
		// Like a CD player: back to the start of this chapter, or to the
		// previous one when it has only just started.
		target = (c >= 0 && now - entries[c].lba > 3 * 75) ? c : c - 1;
		if (target < 0) target = 0;
	}
	goto_entry(target);
	if (p->state == ST_FF || p->state == ST_REW) p->state = ST_PLAY;
}

static void do_scan(player_t *p, int state)
{
	if (p->state == ST_STOP || !plm) return;
	if (p->state == state)
	{
		do_play(p);
		return;
	}
	if (p->state != ST_FF && p->state != ST_REW) p->ff_time = plm_get_time(plm);
	audio_flush();
	p->state = state;
	p->ff_step = 0;
}

static void handle_key(player_t *p, int code, double now)
{
	p->osd_until = now + OSD_TIMEOUT;
	p->dirty = 1;

	if (p->confirm_exit)
	{
		p->confirm_exit = 0;
		if (code == KEY_ENTER) quit_requested = 1;
		return;
	}

	switch (code)
	{
	case KEY_PLAY:
	case KEY_PLAYCD:
		do_play(p);
		break;

	case KEY_PAUSE:
	case KEY_PAUSECD:
	case KEY_PLAYPAUSE:
		if (p->state == ST_PAUSE) p->state = ST_PLAY;
		else if (p->state == ST_PLAY) p->state = ST_PAUSE;
		else if (p->state == ST_FF || p->state == ST_REW)
		{
			seek_to(p->ff_time, FALSE);
			p->state = ST_PAUSE;
		}
		break;

	case KEY_ESC:
	case KEY_STOP:
	case KEY_STOPCD:
		do_stop(p);
		break;

	case KEY_RIGHT:
	case KEY_NEXTSONG:
		do_chapter(p, 1);
		break;

	case KEY_LEFT:
	case KEY_PREVIOUSSONG:
		do_chapter(p, -1);
		break;

	case KEY_PAGEDOWN:
	case KEY_FASTFORWARD:
		do_scan(p, ST_FF);
		break;

	case KEY_PAGEUP:
	case KEY_REWIND:
		do_scan(p, ST_REW);
		break;

	case KEY_TAB:
		p->confirm_exit = 1;
		break;
	}

	static const char *const names[] = { "play", "pause", "stop", "fast forward", "rewind" };
	fprintf(stderr, "vcd_player: key %d, %s at %.1f s%s\n", code, names[p->state], play_time(p),
		p->confirm_exit ? ", exit?" : "");
}

// Fast forward / rewind: jump through the stream at ten times the normal
// speed and show the intra frame found at each step, without sound.
static void scan_step(player_t *p, double elapsed)
{
	p->ff_step += elapsed;
	if (p->ff_step < 0.2) return;
	double step = p->ff_step * 10.0;
	p->ff_step = 0;

	if (p->state == ST_FF)
	{
		p->ff_time += step;
		if (p->ff_time >= cur_duration - 1.0)
		{
			int t = next_data_track(cur_track, 1);
			if (!t || open_track(t))
			{
				// End of the disc: play out the last second normally.
				p->ff_time = cur_duration - 1.0;
				seek_to(p->ff_time, FALSE);
				p->state = ST_PLAY;
				return;
			}
			p->ff_time = 0;
		}
	}
	else
	{
		p->ff_time -= step;
		if (p->ff_time <= 0)
		{
			int t = next_data_track(cur_track, -1);
			if (t && !open_track(t)) p->ff_time = cur_duration - 1.0;
			else
			{
				p->ff_time = 0;
				seek_to(0, FALSE);
				p->state = ST_PLAY;
				return;
			}
		}
	}

	plm_frame_t *frame = plm_seek_frame(plm, p->ff_time, FALSE);
	if (frame) pending_frame = frame;
}

static void osd_fill(const player_t *p, osd_info_t *info)
{
	memset(info, 0, sizeof(*info));
	info->state = p->state;
	info->confirm_exit = p->confirm_exit;
	if (plm)
	{
		info->time = play_time(p);
		info->duration = cur_duration;
		// Half a second of margin: a chapter jump lands on the frame just
		// before the entry point.
		info->chapter = chapter_at(time_to_lba(info->time) + 38) + 1;
	}
	info->chapters = num_entries;
}

static int play_disc(void)
{
	player_t p;
	memset(&p, 0, sizeof(p));

	if (open_track(first_track())) return -1;
	p.state = ST_PLAY;

	STAGE("decode");
	int first = 1;
	double last = now_seconds();
	double osd_built = 0;
	while (!quit_requested)
	{
		double t = now_seconds();
		double elapsed = t - last;
		last = t;
		if (elapsed > 0.5) elapsed = 0.5; // after a drive stall, resume rather than race

		pending_frame = NULL;
		for (int key; (key = input_key(t));) handle_key(&p, key, t);
		if (quit_requested) break;

		if (p.state == ST_PLAY && plm)
		{
			plm_decode(plm, elapsed);
			if (plm_has_ended(plm))
			{
				int next = next_data_track(cur_track, 1);
				if (!next || open_track(next)) break; // end of the disc
			}
		}
		else if ((p.state == ST_FF || p.state == ST_REW) && plm)
		{
			scan_step(&p, elapsed);
		}

		// Controls: shown after any button, hidden 10 s after the last one
		// (an unanswered exit question is cancelled then). They stay up while
		// stopped.
		if (p.confirm_exit && t >= p.osd_until) p.confirm_exit = 0;
		int show = p.state == ST_STOP || t < p.osd_until;
		if (show && (p.dirty || t - osd_built >= 0.25))
		{
			osd_info_t info;
			osd_fill(&p, &info);
			osd_build(&info);
			osd_built = t;
			p.dirty = 1;
		}
		if (show != ovl_shown)
		{
			ovl_shown = show;
			p.dirty = 1;
		}

		if (pending_frame)
		{
			if (first) STAGE("draw first frame");
			fb_draw(pending_frame);
			if (first) STAGE("playing");
			first = 0;
		}
		else if (p.dirty)
		{
			fb_present();
		}
		p.dirty = 0;

		usleep(p.state == ST_PLAY ? 2000 : 10000);
	}

	close_track();
	return 0;
}

int main(int argc, char *argv[])
{
	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	signal(SIGHUP, on_signal);
	signal(SIGPIPE, SIG_IGN);
	struct sigaction sa;
	memset(&sa, 0, sizeof(sa));
	sa.sa_sigaction = on_crash;
	sa.sa_flags = SA_SIGINFO;
	sigaction(SIGSEGV, &sa, NULL);
	sigaction(SIGBUS, &sa, NULL);
	sigaction(SIGILL, &sa, NULL);
	sigaction(SIGFPE, &sa, NULL);
	setvbuf(stdout, NULL, _IOLBF, 0);
	STAGE("open drive");

	cd_fd = open_drive(argc > 1 ? argv[1] : NULL);
	if (cd_fd < 0)
	{
		fprintf(stderr, "vcd_player: no optical drive found\n");
		return 1;
	}

	STAGE("read TOC");
	if (read_toc())
	{
		fprintf(stderr, "vcd_player: could not read the disc table of contents\n");
		return 1;
	}

	STAGE("detect Video CD");
	const char *kind = detect_vcd_kind();
	if (!kind)
	{
		fprintf(stderr, "vcd_player: inserted disc is not a Video CD\n");
		return 1;
	}
	if (track_last < 2)
	{
		fprintf(stderr, "vcd_player: %s has no MPEG tracks\n", kind);
		return 1;
	}
	printf("vcd_player: %s, %d MPEG track(s)\n", kind, track_last - 1);
	read_entries();

	STAGE("open framebuffer");
	if (fb_open())
	{
		fprintf(stderr, "vcd_player: 32-bit /dev/fb0 not available\n");
		return 1;
	}
	osd_layout();
	input_open();

	// Track 1 holds the VCD file system; the MPEG program streams are on
	// tracks 2..N and are played back in order.
	int ok = play_disc() == 0;
	if (!ok) fprintf(stderr, "vcd_player: no playable MPEG-1 stream found\n");

	audio_close();
	if (fb_mem)
	{
		memset(fb_mem, 0, fb_len);
		munmap(fb_mem, fb_len);
	}
	if (input_fd >= 0) close(input_fd);
	close(cd_fd);
	return ok ? 0 : 1;
}
