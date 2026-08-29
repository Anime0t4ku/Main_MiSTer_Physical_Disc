// dvd_css.h — CSS-decrypted reads for a physical DVD-Video disc.
//
// The FPGA DVD core reads plaintext 2048-byte sectors over the generic sd_*
// block interface. This module supplies those sectors from the optical drive,
// decrypting CSS on the way, using a *user-supplied* libdvdcss that is loaded at
// runtime with dlopen(). No CSS code lives in this repository, and there is no
// build-time dependency on libdvdcss: if it is absent, dvd_css_open()
// simply fails and the caller falls back to a raw (scrambled) read, at which
// point the core shows its "CSS ENCRYPTED" notice — the user's cue to install
// the library (Scripts/install_dvdcss.sh).

#ifndef MISTER_PHYSICAL_DISC_CSS_H
#define MISTER_PHYSICAL_DISC_CSS_H

#include <stdint.h>

// Locate the DVD in the optical drive, open it through libdvdcss (running the
// CSS authentication handshake), and keep the handle for reads. Returns 1 on
// success, 0 if libdvdcss is unavailable or no readable DVD is present.
int dvd_css_open(void);

// True while a libdvdcss handle is open.
int dvd_css_active(void);

// Disc data size in bytes (sector count * 2048), reported to the core as the
// mounted-image size. 0 if unknown/closed.
uint64_t dvd_css_size(void);

// Read `count` 2048-byte sectors starting at `lba`, CSS-decrypted, into `buf`.
// Returns the number of sectors read (>0), or -1 on error.
int dvd_css_read(void *buf, uint32_t lba, uint32_t count);

void dvd_css_close(void);

// Call every user_io_poll: surfaces a deferred "encrypted disc, install
// libdvdcss" popup once the launch settles (the mount runs too early to show
// it). No-op unless such a warning is pending.
void dvd_css_tick(void);

#endif
