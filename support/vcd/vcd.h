#ifndef MISTER_VCD_H
#define MISTER_VCD_H

// Video CD support for Physical Disc Auto Disc Detection.
//
// Video CDs hold MPEG-1 streams that no console core can decode, so they are
// played by the standalone HPS application tools/vcd_player, installed at
// VCD_PLAYER_PATH. Without that file nothing changes: Video CDs are handled
// exactly as before.

#define VCD_PLAYER_PATH "/media/fat/linux/vcd_player"
#define VCD_PLAYER_PROCESS "vcd_player"

int vcd_player_installed(void);

// Requires the drive to be open through physical_disc_open().
// Returns 1 for Video CD, Super Video CD and HQ-VCD discs (any region).
int vcd_disc_detect(void);

// Queues the player as a graphical script (drive must be closed first).
int vcd_launch_player(const char *device);

#endif
