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

static unsigned long pending_mount = 0;
static unsigned long mount_deadline = 0;
static unsigned long p3do_prepare_timer = 0;
static int p3do_prepare_state = 0;
static int p3do_prepare_lba = 0;
static toc_t p3do_prepare_toc;
static uint8_t p3do_prepare_buf[PHYSICAL_DISC_RAW];

static int p3do_sector_ready(const uint8_t *data)
{
	for (int i = 0; i < PHYSICAL_DISC_RAW; i++)
	{
		if (data[i]) return 1;
	}
	return 0;
}

int physical_disc_mount_current_core(void)
{
	int recognised = 1;
	int mounted = 0;

	if (is_megacd()) mounted = mcd_set_image(0, PHYSICAL_DISC_SENTINEL);
	else if (is_psx()) mounted = psx_mount_cd(1, 1, PHYSICAL_DISC_SENTINEL);
	else if (is_saturn()) mounted = saturn_set_image(0, PHYSICAL_DISC_SENTINEL);
	else if (is_neogeo())
	{
		neocd_set_en(1);
		mounted = neocd_set_image(PHYSICAL_DISC_SENTINEL);
	}
	else if (is_3do()) mounted = p3do_set_image(0, PHYSICAL_DISC_SENTINEL);
	else if (is_pce())
	{
		pcecd_set_image(0, PHYSICAL_DISC_SENTINEL);
		mounted = pcecdd.loaded;
		if (mounted)
		{
			pcecd_reset();
			user_io_send_buttons(1);
		}
	}
	else if (is_cdi()) mounted = cdi_mount_cd(0, PHYSICAL_DISC_SENTINEL);
	else recognised = 0;

	if (!recognised)
	{
		printf("DISC: core '%s' has no physical disc support yet\n", user_io_get_core_name());
		return 0;
	}

#ifdef HAS_RCHEEVOS
	if (mounted > 0) achievements_load_game(PHYSICAL_DISC_SENTINEL, 0);
#endif

	return mounted;
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

	unsigned int wait_seconds = cfg.physical_disc_mount_delay;
	if (wait_seconds < 8) wait_seconds = 8;
	mount_deadline = GetTimer(wait_seconds * 1000);

	if (is_3do())
	{
		p3do_prepare_state = 0;
		p3do_prepare_lba = 0;
		p3do_prepare_timer = 0;
		pending_mount = GetTimer(1);
		return;
	}

	do
	{
		if (physical_disc_mount_current_core())
		{
			pending_mount = 0;
			mount_deadline = 0;
			return;
		}
		usleep(100000);
	}
	while (!CheckTimer(mount_deadline));

	pending_mount = GetTimer(100);
}

void physical_disc_launch_poll(void)
{
	if (!pending_mount || !CheckTimer(pending_mount)) return;

	if (is_3do())
	{
		if (!p3do_prepare_state)
		{
			memset(&p3do_prepare_toc, 0, sizeof(p3do_prepare_toc));
			if (!physical_disc_open(NULL) && !physical_disc_load_toc(&p3do_prepare_toc))
			{
				p3do_prepare_state = 1;
				p3do_prepare_lba = p3do_prepare_toc.tracks[0].start;
				p3do_prepare_timer = GetTimer(500);
				pending_mount = GetTimer(10);
				return;
			}
			physical_disc_close();
		}
		else if (p3do_prepare_state == 1)
		{
			if (!CheckTimer(p3do_prepare_timer))
			{
				pending_mount = GetTimer(10);
				return;
			}

			if (p3do_prepare_lba < p3do_prepare_toc.tracks[0].start + 32)
			{
				if (physical_disc_read_sector(p3do_prepare_lba, p3do_prepare_buf, NULL) ||
					(p3do_prepare_lba == p3do_prepare_toc.tracks[0].start && !p3do_sector_ready(p3do_prepare_buf)))
				{
					physical_disc_close();
					p3do_prepare_state = 0;
					pending_mount = GetTimer(100);
					return;
				}
				p3do_prepare_lba++;
				pending_mount = GetTimer(1);
				return;
			}

			p3do_prepare_state = 2;
		}

		if (p3do_prepare_state == 2 && physical_disc_mount_current_core())
		{
			pending_mount = 0;
			mount_deadline = 0;
			p3do_prepare_state = 0;
			p3do_prepare_timer = 0;
			return;
		}
	}
	else if (physical_disc_mount_current_core())
	{
		pending_mount = 0;
		mount_deadline = 0;
		return;
	}

	if (!mount_deadline || CheckTimer(mount_deadline))
	{
		physical_disc_close();
		pending_mount = 0;
		mount_deadline = 0;
		p3do_prepare_state = 0;
		p3do_prepare_timer = 0;
		Info("Disc could not be read", 5000);
		return;
	}

	pending_mount = GetTimer(100);
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
	return pending_mount != 0;
}

void physical_disc_launch_cancel(void)
{
	if (p3do_prepare_state) physical_disc_close();
	pending_mount = 0;
	mount_deadline = 0;
	p3do_prepare_state = 0;
	p3do_prepare_timer = 0;
}

int physical_disc_launch_menu_tick(void)
{
	return 0;
}
