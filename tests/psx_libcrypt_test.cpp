// Host unit test for support/psx/psx_libcrypt.cpp (not part of the MiSTer build)
// build: g++ -O1 -Wall -o psx_libcrypt_test psx_libcrypt_test.cpp ../support/psx/psx_libcrypt.cpp
// run:   ./psx_libcrypt_test [cooked .sub of a LibCrypt disc] [expected key, hex]
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include "../support/psx/psx_libcrypt.h"

static int fails = 0;
#define CHECK(c, ...) do { if (c) { printf("ok   "); } else { printf("FAIL "); fails++; } printf(__VA_ARGS__); printf("\n"); } while (0)

static uint8_t bcd(int v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }
static uint16_t crc(const uint8_t *q)
{
	uint16_t c = 0;
	for (int i = 0; i < 10; i++) { c ^= (uint16_t)q[i] << 8; for (int b = 0; b < 8; b++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1); }
	return (uint16_t)~c;
}

// ---------------------------------------------------------------- disc model: Q of every LBA
static const int DISC = 20000;
static std::vector<std::vector<uint8_t>> disc_q;

static void make_clean_disc()
{
	disc_q.assign(DISC, std::vector<uint8_t>(12));
	for (int lba = 0; lba < DISC; lba++)
	{
		int f = lba + 150, r = lba;
		uint8_t *q = disc_q[lba].data();
		uint8_t t[12] = { 0x41, 0x01, 0x01, bcd(r / 4500), bcd((r / 75) % 60), bcd(r % 75), 0, bcd(f / 4500), bcd((f / 75) % 60), bcd(f % 75) };
		uint16_t c = crc(t); t[10] = c >> 8; t[11] = c & 0xFF;
		memcpy(q, t, 12);
	}
}

static void break_q(int frame) // like LibCrypt: modified position, broken CRC
{
	uint8_t *q = disc_q[frame - 150].data();
	q[9] ^= 0x01; q[10] ^= 0x80;
}

static void set_key(uint16_t key)
{
	for (int m = 0; m < 16; m++) if (key & (1 << (15 - m))) { break_q(psx_lc_frame(m * 2)); break_q(psx_lc_frame(m * 2 + 1)); }
}

static void raw_from_q(const uint8_t *q, uint8_t *raw)
{
	memset(raw, 0, 96);
	for (int b = 0; b < 12; b++) for (int bit = 0; bit < 8; bit++) if (q[b] & (0x80 >> bit)) raw[b * 8 + bit] |= 0x40;
	for (int i = 0; i < 96; i++) raw[i] |= (i & 1) ? 0x80 : 0x00; // P channel, other channels noise
	for (int i = 0; i < 96; i++) raw[i] |= (uint8_t)((i * 7) & 0x3F) & 0x15;
}

// ---------------------------------------------------------------- image readers
static int img_raw = 0;
static int rd_image(void *, uint32_t frame, uint8_t *sub)
{
	int lba = (int)frame - 150;
	if (lba < 0 || lba >= DISC) return -1;
	if (img_raw) raw_from_q(disc_q[lba].data(), sub);
	else { memset(sub, 0, 96); memcpy(sub + 12, disc_q[lba].data(), 12); }
	return 0;
}

// ---------------------------------------------------------------- drive model
struct drive_t
{
	int offset;       // entry k holds the Q of lba start+k+offset
	double err;       // chance of a broken Q per entry (random read error)
	int sticky;       // the same sectors fail on every read (drive cache / weak spot)
	int stale;        // first entries of a command repeat an old Q
	int phase_flip;   // some commands come back with the other phase
	int fail_all;     // drive cannot read subchannel
	unsigned seed;
	int reads;
};
static drive_t drv;

static unsigned rnd() { drv.seed = drv.seed * 1103515245u + 12345u; return (drv.seed >> 8) & 0xFFFFFF; }

static int rd_drive(void *, int start, int count, uint8_t *raw)
{
	if (drv.fail_all) return -1;
	drv.reads++;
	int off = drv.offset;
	if (drv.phase_flip && (rnd() % 3) == 0) off += (rnd() & 1) ? 1 : -1;
	for (int k = 0; k < count; k++)
	{
		int lba = start + k + off;
		if (k < drv.stale) lba = start - 40; // stale Q from before the seek
		uint8_t q[12];
		memcpy(q, disc_q[lba].data(), 12);
		double r = drv.sticky ? (((unsigned)lba * 2654435761u) >> 8 & 0xFFFFFF) / 16777216.0 : (rnd() / 16777216.0);
		if (r < drv.err) q[(rnd() % 12)] ^= (uint8_t)(1 << (rnd() % 8));
		raw_from_q(q, raw + k * 96);
	}
	return 0;
}

static uint16_t drive_key(int *reads = NULL)
{
	int8_t st[PSX_LC_SECTORS];
	drv.reads = 0;
	int r = psx_lc_scan_drive(rd_drive, NULL, st);
	if (reads) *reads = r;
	if (r < 0) return 0xFFFF;
	return psx_lc_key(st, 1);
}

int main(int argc, char **argv)
{
	// ---- frame table matches the core (bit 15 = 14105 ... bit 0 = 16167)
	CHECK(psx_lc_frame(0) == 14105 && psx_lc_frame(1) == 14110 && psx_lc_frame(31) == 16172, "LibCrypt sector table");

	// ---- key rules
	int8_t st[PSX_LC_SECTORS];
	memset(st, PSX_LC_GOOD, sizeof(st));
	CHECK(psx_lc_key(st, 0) == 0 && psx_lc_key(st, 1) == 0, "all good -> key 0");
	st[0] = PSX_LC_BAD;
	CHECK(psx_lc_key(st, 0) == 0x8000 && psx_lc_key(st, 1) == 0, "one broken sector of a pair: image yes, drive no");
	st[1] = PSX_LC_UNKNOWN;
	CHECK(psx_lc_key(st, 1) == 0x8000, "drive: broken + unknown counts");
	st[1] = PSX_LC_BAD;
	CHECK(psx_lc_key(st, 1) == 0x8000, "drive: both broken counts");

	// ---- images: clean disc, LibCrypt disc, cooked and raw layout
	make_clean_disc();
	for (img_raw = 0; img_raw < 2; img_raw++)
	{
		psx_lc_scan_image(rd_image, NULL, img_raw ? 2 : 0, st);
		CHECK(psx_lc_key(st, 0) == 0, "image %s, no LibCrypt -> key 0", img_raw ? "raw (auto)" : "cooked");
	}
	set_key(0x94CD);
	for (img_raw = 0; img_raw < 2; img_raw++)
	{
		psx_lc_scan_image(rd_image, NULL, img_raw ? 2 : 0, st);
		CHECK(psx_lc_key(st, 0) == 0x94CD, "image %s, LibCrypt 94CD -> %04X", img_raw ? "raw (auto)" : "cooked", psx_lc_key(st, 0));
	}

	// ---- drive: LibCrypt disc, realistic and hostile drives
	struct { const char *name; drive_t d; } cases[] = {
		{ "perfect drive", { 0, 0, 0, 0, 0, 0, 1, 0 } },
		{ "offset -1 (like the SuperStation drive)", { -1, 0, 0, 0, 0, 0, 1, 0 } },
		{ "offset -1, stale first 2 entries", { -1, 0, 0, 2, 0, 0, 1, 0 } },
		{ "offset -1, stale, phase flips", { -1, 0, 0, 2, 1, 0, 1, 0 } },
		{ "offset -1, stale, flips, 1% read errors", { -1, 0.01, 0, 2, 1, 0, 1, 0 } },
		{ "offset +1, stale, flips, 3% read errors", { 1, 0.03, 0, 2, 1, 0, 1, 0 } },
	};
	for (auto &c : cases)
	{
		int bad = 0, maxreads = 0;
		for (unsigned seed = 1; seed <= 200; seed++)
		{
			drv = c.d; drv.seed = seed * 7919;
			int reads;
			if (drive_key(&reads) != 0x94CD) bad++;
			if (reads > maxreads) maxreads = reads;
		}
		CHECK(bad == 0, "drive, LibCrypt 94CD, %s: %d/200 wrong, at most %d reads", c.name, bad, maxreads);
	}

	// ---- drive: clean disc must never get a key (false positives)
	make_clean_disc();
	struct { const char *name; drive_t d; } clean[] = {
		{ "offset -1, stale, flips, 1% random errors", { -1, 0.01, 0, 2, 1, 0, 1, 0 } },
		{ "offset -1, stale, flips, 5% random errors", { -1, 0.05, 0, 2, 1, 0, 1, 0 } },
		{ "offset -1, 2% errors always on the same sectors", { -1, 0.02, 1, 2, 0, 0, 1, 0 } },
	};
	for (auto &c : clean)
	{
		int bad = 0, reads = 0, typical = 0;
		for (unsigned seed = 1; seed <= 500; seed++)
		{
			drv = c.d; drv.seed = seed * 104729;
			if (drive_key(&reads) != 0) bad++;
			typical += reads;
		}
		CHECK(bad == 0, "drive, no LibCrypt, %s: %d/500 false keys, %.1f reads on average", c.name, bad, typical / 500.0);
	}
	{
		drv = { -1, 0, 0, 2, 0, 0, 1, 0 };
		int reads; uint16_t k = drive_key(&reads);
		CHECK(k == 0 && reads == 16, "drive, no LibCrypt, clean reads: one read per pair (%d reads)", reads);
		drv = { 0, 0, 0, 0, 0, 1, 1, 0 };
		CHECK(drive_key() == 0xFFFF, "drive without subchannel: reported, no key");
	}

	// ---- real rip: cooked .sub given on the command line
	if (argc >= 3)
	{
		FILE *f = fopen(argv[1], "rb");
		if (f)
		{
			fseek(f, 0, SEEK_END); long n = ftell(f) / 96; fseek(f, 0, SEEK_SET);
			std::vector<uint8_t> sub(n * 96);
			size_t got = fread(sub.data(), 96, n, f); fclose(f);
			disc_q.assign(DISC, std::vector<uint8_t>(12));
			for (int i = 0; i < DISC && i < (long)got; i++) memcpy(disc_q[i].data(), &sub[i * 96 + 12], 12);
			unsigned want = strtoul(argv[2], NULL, 16);
			img_raw = 0;
			psx_lc_scan_image(rd_image, NULL, 0, st);
			char map[33]; for (int i = 0; i < 32; i++) map[i] = st[i] == PSX_LC_BAD ? 'X' : st[i] == PSX_LC_GOOD ? '.' : '?'; map[32] = 0;
			CHECK(psx_lc_key(st, 0) == want, "real rip %s: key %04X [%s]", argv[1], psx_lc_key(st, 0), map);
			img_raw = 1;
			psx_lc_scan_image(rd_image, NULL, 2, st);
			CHECK(psx_lc_key(st, 0) == want, "real rip as raw CHD subcode: key %04X", psx_lc_key(st, 0));
		}
	}

	printf(fails ? "%d TESTS FAILED\n" : "ALL TESTS PASSED\n", fails);
	return fails != 0;
}
