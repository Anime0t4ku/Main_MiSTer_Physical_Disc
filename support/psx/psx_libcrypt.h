#ifndef PSX_LIBCRYPT_H
#define PSX_LIBCRYPT_H

// LibCrypt key from the real subchannel (Q) of a disc or an image.
//
// The official PSX core already emulates LibCrypt: it only needs a 16 bit key
// telling which of the 16 LibCrypt sector pairs carry a modified Q. Until now
// the Main took that key only from an .sbi file. When no .sbi is found, the
// key can be read from the subchannel itself:
//   - physical disc: raw P-W of the 32 LibCrypt sectors, read from the drive;
//   - CHD with subcode (for example made by Disc Tools);
//   - BIN/CUE with a CloneCD style .sub file next to the .cue.
// The core is not changed: it receives the same key it would get from the .sbi.

#include <stdint.h>

#define PSX_LC_PAIRS 16
#define PSX_LC_SECTORS (PSX_LC_PAIRS * 2)

// absolute frame (MSF, i.e. LBA + 150) of LibCrypt sector i:
// i = 2*m -> first sector of pair m, i = 2*m+1 -> the same + 5
uint32_t psx_lc_frame(int i);

// state of one sector
#define PSX_LC_UNKNOWN (-1)
#define PSX_LC_GOOD    0
#define PSX_LC_BAD     1

// key from the 32 sector states. strict=0 (images): a pair counts if either
// sector has a broken Q. strict=1 (physical drive, raw subchannel may have read
// errors): both sectors must be broken, or one broken and the other unknown.
uint16_t psx_lc_key(const int8_t state[PSX_LC_SECTORS], int strict);

// Q helpers (12 bytes Q, CRC in the last two)
int  psx_lc_crc_ok(const uint8_t *q);
int  psx_lc_q_frame(const uint8_t *q);                 // absolute frame from Q bytes 7..9, -1 if invalid
void psx_lc_q_from_raw(const uint8_t *raw96, uint8_t *q); // interleaved raw P-W -> Q
void psx_lc_q_from_cooked(const uint8_t *sub96, uint8_t *q); // deinterleaved (CloneCD .sub) -> Q

// Image: sub96 of an absolute frame. Returns 0 on success.
typedef int (*psx_lc_sub_reader)(void *ctx, uint32_t frame, uint8_t *sub96);

// Sector states from an image. fmt: 0 = cooked (.sub), 1 = raw interleaved,
// 2 = unknown (decided per sector from the neighbour sectors, whose Q is valid).
// Returns the number of sectors that could be read (0 = no subchannel).
int psx_lc_scan_image(psx_lc_sub_reader rd, void *ctx, int fmt, int8_t state[PSX_LC_SECTORS]);

// Physical drive: raw P-W of count consecutive sectors starting at lba (not
// frame), subchannel only. Returns 0 on success.
typedef int (*psx_lc_window_reader)(void *ctx, int lba, int count, uint8_t *raw96);

// Sector states from a drive. The raw subchannel of a drive is not error
// corrected and many drives return it shifted by a sector or stale right after
// a seek, so every pair is read in a small window and the position of each
// entry is taken from the valid Q around it. A sector is GOOD as soon as one
// read returns its own valid Q; it is BAD when at least two reads return a
// broken Q in a correctly placed window. Returns the number of reads done, or
// -1 if the drive cannot read the subchannel.
int psx_lc_scan_drive(psx_lc_window_reader rd, void *ctx, int8_t state[PSX_LC_SECTORS]);

#endif
