// physical_disc_css.h — CSS-decrypted reads for a physical DVD-Video disc.
//
// The FPGA DVD core reads plaintext 2048-byte sectors over the generic sd_*
// block interface. This module supplies those sectors from the optical drive,
// decrypting CSS on the way, using a *user-supplied* libdvdcss that is loaded at
// runtime with dlopen(). No CSS code lives in this repository, and there is no
// build-time dependency on libdvdcss: if it is absent, physical_disc_css_open()
// simply fails and the caller falls back to a raw (scrambled) read, at which
// point the core shows its "CSS ENCRYPTED" notice — the user's cue to install
// the library (Scripts/install_dvdcss.sh).

#ifndef MISTER_PHYSICAL_DISC_CSS_H
#define MISTER_PHYSICAL_DISC_CSS_H

#include <stdint.h>

// Locate the DVD in the optical drive, open it through libdvdcss (running the
// CSS authentication handshake), and keep the handle for reads. Returns 1 on
// success, 0 if libdvdcss is unavailable or no readable DVD is present.
int physical_disc_css_open(void);

// True while a libdvdcss handle is open.
int physical_disc_css_active(void);

// Disc data size in bytes (sector count * 2048), reported to the core as the
// mounted-image size. 0 if unknown/closed.
uint64_t physical_disc_css_size(void);

// Read `count` 2048-byte sectors starting at `lba`, CSS-decrypted, into `buf`.
// Returns the number of sectors read (>0), or -1 on error.
int physical_disc_css_read(void *buf, uint32_t lba, uint32_t count);

void physical_disc_css_close(void);

// Append a diagnostic line to the CSS log (/tmp/dvdcss.log) + stdout. Lets other
// modules (e.g. the sd block-read loop) record what they see, for debugging.
void physical_disc_css_diag(const char *fmt, ...);

#endif
