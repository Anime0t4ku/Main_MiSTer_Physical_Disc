#ifndef PSX_H
#define PSX_H

void psx_mount_cd(int f_index, int s_index, const char *filename);
void psx_use_physical_cd();
void psx_fill_blanksave(uint8_t *buffer, uint32_t lba, int cnt);
void psx_read_cd(uint8_t *buffer, int lba, int cnt);
uint64_t psx_trace_now_us();
void psx_trace_delivery(uint32_t lba, int blks, int buffer_hit, uint64_t request_us,
	uint64_t read_us, uint64_t spi_us, uint64_t postfill_us);
int psx_chd_hunksize();
const char* psx_get_game_id();
void psx_poll();
void psx_reset();

#endif
