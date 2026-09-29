// LibCrypt key from the real subchannel, see psx_libcrypt.h

#include <string.h>
#include "psx_libcrypt.h"

// first sector of each LibCrypt pair (absolute frame), same order as the key
// bits of the PSX core (bit 15 = 14105 ... bit 0 = 16167) and as libCryptSectors[]
static const uint32_t lc_first[PSX_LC_PAIRS] =
{
	14105, 14231, 14485, 14579, 14649, 14899, 15056, 15130,
	15242, 15312, 15378, 15628, 15919, 16031, 16101, 16167,
};

uint32_t psx_lc_frame(int i)
{
	return lc_first[i >> 1] + ((i & 1) ? 5 : 0);
}

uint16_t psx_lc_key(const int8_t state[PSX_LC_SECTORS], int strict)
{
	uint16_t key = 0;
	for (int m = 0; m < PSX_LC_PAIRS; m++)
	{
		int a = state[m * 2], b = state[m * 2 + 1];
		int set = strict ?
			((a == PSX_LC_BAD && b == PSX_LC_BAD) || (a == PSX_LC_BAD && b == PSX_LC_UNKNOWN) || (a == PSX_LC_UNKNOWN && b == PSX_LC_BAD)) :
			(a == PSX_LC_BAD || b == PSX_LC_BAD);
		if (set) key |= (uint16_t)(1 << (15 - m));
	}
	return key;
}

int psx_lc_crc_ok(const uint8_t *q)
{
	uint16_t crc = 0;
	for (int i = 0; i < 10; i++)
	{
		crc ^= (uint16_t)q[i] << 8;
		for (int b = 0; b < 8; b++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
	}
	return (uint16_t)~crc == (uint16_t)((q[10] << 8) | q[11]);
}

static int unbcd(uint8_t v)
{
	int hi = v >> 4, lo = v & 15;
	return (hi > 9 || lo > 9) ? -1 : hi * 10 + lo;
}

int psx_lc_q_frame(const uint8_t *q)
{
	if ((q[0] & 0x0F) != 1) return -1; // ADR 1: position
	int m = unbcd(q[7]), s = unbcd(q[8]), f = unbcd(q[9]);
	if (m < 0 || s < 0 || f < 0 || s > 59 || f > 74) return -1;
	return (m * 60 + s) * 75 + f;
}

void psx_lc_q_from_raw(const uint8_t *raw96, uint8_t *q)
{
	for (int b = 0; b < 12; b++)
	{
		uint8_t v = 0;
		for (int bit = 0; bit < 8; bit++) v = (uint8_t)((v << 1) | ((raw96[b * 8 + bit] >> 6) & 1));
		q[b] = v;
	}
}

void psx_lc_q_from_cooked(const uint8_t *sub96, uint8_t *q)
{
	memcpy(q, sub96 + 12, 12);
}

// ---------------------------------------------------------------------------- images

static int valid_named(const uint8_t *q, uint32_t frame)
{
	return psx_lc_crc_ok(q) && psx_lc_q_frame(q) == (int)frame;
}

// returns 0 cooked, 1 raw, -1 undecided; from a neighbour whose Q must be valid
static int detect_format(psx_lc_sub_reader rd, void *ctx, uint32_t frame)
{
	static const int near[] = { -1, 1, -2, 2, -3, 3 };
	for (unsigned i = 0; i < sizeof(near) / sizeof(near[0]); i++)
	{
		uint8_t sub[96], q[12];
		uint32_t f = frame + near[i];
		if (rd(ctx, f, sub)) continue;
		psx_lc_q_from_raw(sub, q);
		if (valid_named(q, f)) return 1;
		psx_lc_q_from_cooked(sub, q);
		if (valid_named(q, f)) return 0;
	}
	return -1;
}

int psx_lc_scan_image(psx_lc_sub_reader rd, void *ctx, int fmt, int8_t state[PSX_LC_SECTORS])
{
	int read = 0;
	for (int i = 0; i < PSX_LC_SECTORS; i++)
	{
		state[i] = PSX_LC_UNKNOWN;
		uint32_t frame = psx_lc_frame(i);
		int f = fmt;
		if (f == 2) f = detect_format(rd, ctx, frame);
		if (f < 0) continue;

		uint8_t sub[96], q[12];
		if (rd(ctx, frame, sub)) continue;
		read++;
		if (f) psx_lc_q_from_raw(sub, q);
		else psx_lc_q_from_cooked(sub, q);

		if (!psx_lc_crc_ok(q)) state[i] = PSX_LC_BAD;
		else if (psx_lc_q_frame(q) == (int)frame) state[i] = PSX_LC_GOOD;
		// a valid Q of another sector: misplaced subchannel, leave UNKNOWN
	}
	return read;
}

// ---------------------------------------------------------------------------- drive

#define LC_PRE      8   // sectors read before the first sector of a pair
#define LC_POST     4   // sectors read after the second one
#define LC_MAXWIN   (LC_PRE + 6 + 5 + LC_POST + 1)
#define LC_PASSES   4
#define LC_SKIP     2   // first entries of a read may hold a stale Q right after the seek
#define LC_MAXPHASE 3

// entry k of the read holds the Q of sector start + k + phase
static int window_phase(const uint8_t *raw, int start, int count, int *phase)
{
	int votes[2 * LC_MAXPHASE + 1] = {};
	int total = 0;
	for (int k = LC_SKIP; k < count; k++)
	{
		uint8_t q[12];
		psx_lc_q_from_raw(raw + k * 96, q);
		if (!psx_lc_crc_ok(q)) continue;
		int f = psx_lc_q_frame(q);
		if (f < 0) continue;
		int d = (f - 150) - (start + k);
		if (d < -LC_MAXPHASE || d > LC_MAXPHASE) continue;
		votes[d + LC_MAXPHASE]++;
		total++;
	}
	int best = 0;
	for (int i = 1; i < 2 * LC_MAXPHASE + 1; i++) if (votes[i] > votes[best]) best = i;
	if (votes[best] < 3 || votes[best] * 2 <= total) return 0;
	*phase = best - LC_MAXPHASE;
	return 1;
}

static int entry_names(const uint8_t *raw, int count, int k, int lba)
{
	if (k < 0 || k >= count) return 0;
	uint8_t q[12];
	psx_lc_q_from_raw(raw + k * 96, q);
	return psx_lc_crc_ok(q) && psx_lc_q_frame(q) == lba + 150;
}

int psx_lc_scan_drive(psx_lc_window_reader rd, void *ctx, int8_t state[PSX_LC_SECTORS])
{
	int bad[PSX_LC_SECTORS] = {};
	int good[PSX_LC_SECTORS] = {};
	uint8_t raw[LC_MAXWIN * 96];
	int reads = 0;

	for (int i = 0; i < PSX_LC_SECTORS; i++) state[i] = PSX_LC_UNKNOWN;

	for (int pass = 0; pass < LC_PASSES; pass++)
	{
		int pending = 0;
		for (int m = 0; m < PSX_LC_PAIRS; m++)
		{
			int a = m * 2, b = m * 2 + 1;
			int need_a = !good[a] && bad[a] < 2;
			int need_b = !good[b] && bad[b] < 2;
			if (!need_a && !need_b) continue;
			pending++;

			// every pass starts a bit earlier, so the drive sees a different command
			int x0 = (int)psx_lc_frame(a) - 150;
			int start = x0 - LC_PRE - 2 * pass;
			int count = (x0 + 5 + LC_POST) - start + 1;
			if (rd(ctx, start, count, raw))
			{
				if (!reads) return -1; // the drive cannot read the subchannel at all
				continue;
			}
			reads++;

			int phase;
			if (!window_phase(raw, start, count, &phase)) continue;

			for (int t = a; t <= b; t++)
			{
				int x = (int)psx_lc_frame(t) - 150;
				int k = x - start - phase;
				if (k < LC_SKIP || k >= count) continue;
				uint8_t q[12];
				psx_lc_q_from_raw(raw + k * 96, q);
				if (psx_lc_crc_ok(q))
				{
					if (psx_lc_q_frame(q) == x + 150) good[t]++;
				}
				else if (entry_names(raw, count, k - 1, x - 1) || entry_names(raw, count, k + 1, x + 1))
				{
					// broken Q in a correctly placed entry (a neighbour confirms the position)
					bad[t]++;
				}
			}
		}
		if (!pending) break;
	}

	for (int i = 0; i < PSX_LC_SECTORS; i++)
	{
		// a read error can break a Q, but never produce the valid Q of that very sector
		if (good[i]) state[i] = PSX_LC_GOOD;
		else if (bad[i] >= 2) state[i] = PSX_LC_BAD;
	}
	return reads;
}
