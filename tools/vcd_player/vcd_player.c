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
// Stop:  MiSTer Main kills the process by name ("vcd_player") when the user
//        presses OSD/Menu/Back, like any other graphical script.

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
#include <scsi/sg.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

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

static void on_crash(int sig)
{
	char msg[160];
	int n = snprintf(msg, sizeof(msg), "vcd_player: crashed (signal %d) during: %s\n", sig, stage);
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

typedef struct
{
	int track;
	int lba;
	uint8_t raw[READ_BATCH * RAW_SECTOR];
} feed_t;

// Feeds one MPEG track. Each track is a separate program stream whose clock
// starts again from zero, so every track gets its own decoder instance.
static void feed_load(plm_buffer_t *buf, void *user)
{
	feed_t *f = (feed_t *)user;
	const track_t *t = &tracks[f->track];

	while (!quit_requested && f->lba < t->end)
	{
		int count = t->end - f->lba;
		if (count > READ_BATCH) count = READ_BATCH;
		read_raw(f->lba, count, f->raw);
		f->lba += count;

		size_t written = 0;
		for (int i = 0; i < count; i++)
		{
			int len = 0;
			const uint8_t *p = sector_payload(f->raw + i * RAW_SECTOR, &len);
			if (len)
			{
				plm_buffer_write(buf, (uint8_t *)p, len);
				written += len;
			}
		}
		if (written) return;
	}

	plm_buffer_signal_end(buf);
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

static void fb_draw(plm_frame_t *frame)
{
	fb_layout(frame->width, frame->height);
	if (!frame_rgb || !line_buf || !x_map) return;

	if (fb_bgr) plm_frame_to_bgra(frame, frame_rgb, src_w * 4);
	else plm_frame_to_rgba(frame, frame_rgb, src_w * 4);

	// Nearest-neighbour scale. Build each source line once in cached memory and
	// copy it to every framebuffer line it covers.
	int prev_sy = -1;
	for (int y = 0; y < dst_h; y++)
	{
		int sy = y * src_h / dst_h;
		if (sy != prev_sy)
		{
			const uint32_t *src = (const uint32_t *)(frame_rgb + (size_t)sy * src_w * 4);
			for (int x = 0; x < dst_w; x++) line_buf[x] = src[x_map[x]] | 0xFF000000u;
			prev_sy = sy;
		}
		memcpy(fb_mem + (size_t)(dst_y + y) * fb_stride + (size_t)dst_x * 4, line_buf, (size_t)dst_w * 4);
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

// ---------------------------------------------------------------------------
// Playback
// ---------------------------------------------------------------------------

static plm_frame_t *pending_frame = NULL;

static void on_video(plm_t *plm, plm_frame_t *frame, void *user)
{
	(void)plm;
	(void)user;
	// Only the most recent frame of a decode step is drawn, so a slow step
	// drops frames instead of falling further behind the audio.
	pending_frame = frame;
}

static void on_audio(plm_t *plm, plm_samples_t *samples, void *user)
{
	(void)plm;
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

static int play_track(int track)
{
	static feed_t feed;
	feed.track = track;
	feed.lba = tracks[track].start;

	plm_buffer_t *buf = plm_buffer_create_with_capacity(512 * 1024);
	plm_buffer_set_load_callback(buf, feed_load, &feed);
	STAGE("create MPEG decoder");
	plm_t *plm = plm_create_with_buffer(buf, 1);
	if (!plm) return -1;
	// plm_probe() is only safe on seekable buffers; this one is a streaming
	// buffer fed from the drive, so wait for the stream headers instead.
	STAGE("read MPEG headers");
	if (!plm_has_headers(plm))
	{
		fprintf(stderr, "vcd_player: track %d has no MPEG-1 stream\n", track);
		plm_destroy(plm);
		return -1;
	}

	printf("vcd_player: track %d, %dx%d @ %.3f fps, audio %d Hz\n", track,
		plm_get_width(plm), plm_get_height(plm), plm_get_framerate(plm), plm_get_samplerate(plm));

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

	STAGE("decode");
	double last = now_seconds();
	while (!quit_requested && !plm_has_ended(plm))
	{
		double t = now_seconds();
		double elapsed = t - last;
		last = t;
		if (elapsed > 0.5) elapsed = 0.5; // after a drive stall, resume rather than race

		pending_frame = NULL;
		plm_decode(plm, elapsed);
		if (pending_frame)
		{
			static int first = 1;
			if (first) STAGE("draw first frame");
			fb_draw(pending_frame);
			if (first) STAGE("playing");
			first = 0;
		}

		usleep(2000);
	}

	plm_destroy(plm);
	return 0;
}

int main(int argc, char *argv[])
{
	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	signal(SIGHUP, on_signal);
	signal(SIGPIPE, SIG_IGN);
	signal(SIGSEGV, on_crash);
	signal(SIGBUS, on_crash);
	signal(SIGILL, on_crash);
	signal(SIGFPE, on_crash);
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

	STAGE("open framebuffer");
	if (fb_open())
	{
		fprintf(stderr, "vcd_player: 32-bit /dev/fb0 not available\n");
		return 1;
	}

	// Track 1 holds the VCD file system; the MPEG program streams are on
	// tracks 2..N and are played back in order.
	int played = 0;
	for (int track = 2; track <= track_last && !quit_requested; track++)
	{
		if (!tracks[track].data) continue;
		if (play_track(track) == 0) played++;
	}

	if (!played) fprintf(stderr, "vcd_player: no playable MPEG-1 stream found\n");

	audio_close();
	if (fb_mem)
	{
		memset(fb_mem, 0, fb_len);
		munmap(fb_mem, fb_len);
	}
	close(cd_fd);
	return played ? 0 : 1;
}
