#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "../../menu.h"
#include "../physical_disc/physical_disc.h"
#include "vcd.h"

// INFO.VCD / INFO.SVD is always mastered at MSF 00:04:00 (LBA 150).
#define VCD_INFO_LBA 150

int vcd_player_installed(void)
{
	return access(VCD_PLAYER_PATH, X_OK) == 0;
}

int vcd_disc_detect(void)
{
	uint8_t info[2048];
	if (physical_disc_read_data2048(VCD_INFO_LBA, info)) return 0;

	return !memcmp(info, "VIDEO_CD", 8) || !memcmp(info, "SUPERVCD", 8) || !memcmp(info, "HQ-VCD  ", 8);
}

int vcd_launch_player(const char *device)
{
	char command[256];
	if (device && *device) snprintf(command, sizeof(command), "%s %s >/tmp/vcd_player.log 2>&1", VCD_PLAYER_PATH, device);
	else snprintf(command, sizeof(command), "%s >/tmp/vcd_player.log 2>&1", VCD_PLAYER_PATH);

	printf("DISC: Video CD detected, launching %s\n", VCD_PLAYER_PATH);
	return menu_launch_graphical_script_command(command, "Video CD", VCD_PLAYER_PROCESS);
}
