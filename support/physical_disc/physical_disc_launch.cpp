#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>

#include "../../user_io.h"
#include "../../menu.h"
#include "../../cfg.h"
#include "../../hardware.h"
#include "../megacd/megacd.h"
#include "../psx/psx.h"
#include "../saturn/saturn.h"
#include "../neogeo/neogeocd.h"
#include "../3do/3do.h"
#include "../pcecd/pcecd.h"
#include "../cdi/cdi.h"
#include "physical_disc.h"
#include "physical_disc_launch.h"
#include "physical_disc_acoustic.h"

#ifdef HAS_RCHEEVOS
#include "../../achievements.h"
#endif

enum
{
	SPINUP_IDLE = 0,
	SPINUP_SETTLING = 1,
	SPINUP_READY = 2,
};

static unsigned long mount_retry_at = 0;
static unsigned long mount_giveup_at = 0;
static unsigned long spinup_settle_at = 0;
static int spinup_stage = SPINUP_IDLE;
static int spinup_probe_lba = 0;
static toc_t spinup_toc;
static uint8_t spinup_probe_buf[PHYSICAL_DISC_RAW];
static int suppress_pce_boot_osd = 0;

static int sector_has_data(const uint8_t *data)
{
	for (int i = 0; i < PHYSICAL_DISC_RAW; i++)
	{
		if (data[i]) return 1;
	}
	return 0;
}

static void begin_mount_window(void)
{
	unsigned int wait_seconds = cfg.physical_disc_mount_delay;
	if (wait_seconds < 8) wait_seconds = 8;
	mount_giveup_at = GetTimer(wait_seconds * 1000);
}

static void clear_spinup_state(void)
{
	spinup_stage = SPINUP_IDLE;
	spinup_probe_lba = 0;
	spinup_settle_at = 0;
}

int physical_disc_mount_current_core(void)
{
	int known = 1;
	int ok = 0;

	if (is_megacd()) ok = mcd_set_image(0, PHYSICAL_DISC_SENTINEL);
	else if (is_psx()) ok = psx_mount_cd(1, 1, PHYSICAL_DISC_SENTINEL);
	else if (is_saturn()) ok = saturn_set_image(0, PHYSICAL_DISC_SENTINEL);
	else if (is_neogeo())
	{
		neocd_set_en(1);
		ok = neocd_set_image(PHYSICAL_DISC_SENTINEL);
	}
	else if (is_3do()) ok = p3do_set_image(0, PHYSICAL_DISC_SENTINEL);
	else if (is_pce())
	{
		pcecd_set_image(0, PHYSICAL_DISC_SENTINEL);
		ok = pcecdd.loaded;
		if (ok)
		{
			pcecd_reset();
			user_io_send_buttons(1);
		}
	}
	else if (is_cdi()) ok = cdi_mount_cd(0, PHYSICAL_DISC_SENTINEL);
	else known = 0;

	if (!known)
	{
		printf("DISC: core '%s' has no physical disc support yet\n", user_io_get_core_name());
		return 0;
	}

#ifdef HAS_RCHEEVOS
	if (ok > 0) achievements_load_game(PHYSICAL_DISC_SENTINEL, 0);
#endif

	return ok;
}

int physical_disc_swap_current_core(void)
{
	if (is_psx())
	{
		psx_swap_disc();
		return 1;
	}

	printf("DISC: swap_disc is a PSX-only manual path; '%s' auto-detects physical swaps if it supports them\n", user_io_get_core_name());
	return 0;
}

void physical_disc_launch_startup(void)
{
	physical_disc_acoustic_config(cfg.physical_disc_acoustic);
	const char *name = user_io_get_core_name();
	if (!name || strncasecmp(name, "CD-", 3)) return;
	if (!strcasecmp(name, "CD-SNES"))
	{
		physical_disc_launch_cancel();
		physical_disc_close();
		mount_retry_at = 0;
		mount_giveup_at = 0;
		return;
	}
	physical_disc_prepare_environment();

	suppress_pce_boot_osd = is_pce();

	begin_mount_window();

	if (is_3do())
	{
		clear_spinup_state();
		mount_retry_at = GetTimer(1);
		return;
	}

	do
	{
		if (physical_disc_mount_current_core())
		{
			mount_retry_at = 0;
			mount_giveup_at = 0;
			return;
		}
		usleep(100000);
	}
	while (!CheckTimer(mount_giveup_at));

	mount_retry_at = GetTimer(100);
}

void physical_disc_launch_poll(void)
{
	if (!mount_retry_at || !CheckTimer(mount_retry_at)) return;

	if (is_3do())
	{
		if (spinup_stage == SPINUP_IDLE)
		{
			memset(&spinup_toc, 0, sizeof(spinup_toc));
			if (!physical_disc_open(NULL) && !physical_disc_load_toc(&spinup_toc))
			{
				spinup_stage = SPINUP_SETTLING;
				spinup_probe_lba = spinup_toc.tracks[0].start;
				spinup_settle_at = GetTimer(500);
				mount_retry_at = GetTimer(10);
				return;
			}
			physical_disc_close();
		}
		else if (spinup_stage == SPINUP_SETTLING)
		{
			if (!CheckTimer(spinup_settle_at))
			{
				mount_retry_at = GetTimer(10);
				return;
			}

			if (spinup_probe_lba < spinup_toc.tracks[0].start + 32)
			{
				if (physical_disc_read_sector(spinup_probe_lba, spinup_probe_buf, NULL) ||
					(spinup_probe_lba == spinup_toc.tracks[0].start && !sector_has_data(spinup_probe_buf)))
				{
					physical_disc_close();
					clear_spinup_state();
					mount_retry_at = GetTimer(100);
					return;
				}
				spinup_probe_lba++;
				mount_retry_at = GetTimer(1);
				return;
			}

			spinup_stage = SPINUP_READY;
		}

		if (spinup_stage == SPINUP_READY && physical_disc_mount_current_core())
		{
			mount_retry_at = 0;
			mount_giveup_at = 0;
			clear_spinup_state();
			return;
		}
	}
	else if (physical_disc_mount_current_core())
	{
		mount_retry_at = 0;
		mount_giveup_at = 0;
		return;
	}

	if (!mount_giveup_at || CheckTimer(mount_giveup_at))
	{
		physical_disc_close();
		mount_retry_at = 0;
		mount_giveup_at = 0;
		clear_spinup_state();
		Info("Disc could not be read", 5000);
		return;
	}

	mount_retry_at = GetTimer(100);
}

int physical_disc_is_menu_row(const char *name)
{
	return name && !strcmp(name, PHYSICAL_DISC_MENU_SENTINEL);
}

int physical_disc_menu_row(char *out, int outsz)
{
	(void)out;
	(void)outsz;
	return 0;
}

int physical_disc_launch_load_disc(void)
{
	return 0;
}

int physical_disc_launch_busy(void)
{
	return mount_retry_at != 0;
}

void physical_disc_launch_cancel(void)
{
	suppress_pce_boot_osd = 0;
	if (spinup_stage) physical_disc_close();
	mount_retry_at = 0;
	mount_giveup_at = 0;
	clear_spinup_state();
}

void physical_disc_launch_reset(void)
{
	const char *name = user_io_get_core_name();
	if (!name || strncasecmp(name, "CD-", 3) || !strcasecmp(name, "CD-SNES")) return;

	physical_disc_launch_cancel();
	physical_disc_close();
	physical_disc_acoustic_config(cfg.physical_disc_acoustic);

	begin_mount_window();
	clear_spinup_state();
	mount_retry_at = GetTimer(100);
}

int physical_disc_launch_consume_startup_osd_suppression(void)
{
	if (!suppress_pce_boot_osd) return 0;
	suppress_pce_boot_osd = 0;
	return 1;
}

int physical_disc_launch_menu_tick(void)
{
	return 0;
}
