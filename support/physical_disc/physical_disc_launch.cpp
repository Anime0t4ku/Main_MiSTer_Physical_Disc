#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>

#include "../../user_io.h"
#include "../../menu.h"
#include "../../cfg.h"
#include "../../hardware.h"
#include "../../file_io.h"
#include "../arcade/mra_loader.h"
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
static unsigned long suppress_neogeo_boot_osd_until = 0;
static unsigned long menu_poll_at = 0;
static int menu_detecting = 0;
static int menu_environment_ready = 0;
static int menu_boot_checked = 0;
static int menu_ignore_boot_disc = 0;
#define PHYSICAL_DISC_HANDLED_FILE "/tmp/physical_disc_menu_handled"
#define PHYSICAL_DISC_MGL_DIR "/media/fat/_Physical Disc Cores"

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
	if (!name || strncasecmp(name, "A0CD-", 5)) return;
	if (!strcasecmp(name, "A0CD-SNES") || !strcasecmp(name, "A0CD-MDPlus"))
	{
		int suppress_startup_osd = !strcasecmp(name, "A0CD-MDPlus");
		physical_disc_launch_cancel();
		physical_disc_close();
		mount_retry_at = 0;
		mount_giveup_at = 0;
		if (suppress_startup_osd) suppress_pce_boot_osd = 1;
		return;
	}
	physical_disc_prepare_environment();

	suppress_pce_boot_osd = is_pce();
	suppress_neogeo_boot_osd_until = 0;

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
			if (!strcasecmp(name, "A0CD-NeoGeoCD")) suppress_neogeo_boot_osd_until = GetTimer(3000);
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
		const char *name = user_io_get_core_name();
		if (name && !strcasecmp(name, "A0CD-NeoGeoCD")) suppress_neogeo_boot_osd_until = GetTimer(3000);
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

static unsigned int menu_disc_fingerprint(const toc_t *toc)
{
	unsigned int h = 2166136261u;
	h = (h ^ (unsigned int)toc->last) * 16777619u;
	h = (h ^ (unsigned int)toc->end) * 16777619u;
	for (int i = 0; i <= toc->last && i < 100; i++)
	{
		h = (h ^ (unsigned int)toc->tracks[i].start) * 16777619u;
		h = (h ^ (unsigned int)toc->tracks[i].end) * 16777619u;
		h = (h ^ (unsigned int)toc->tracks[i].type) * 16777619u;
	}
	return h;
}

static unsigned int menu_read_handled(void)
{
	FILE *f = fopen(PHYSICAL_DISC_HANDLED_FILE, "r");
	unsigned int value = 0;
	if (f) { fscanf(f, "%x", &value); fclose(f); }
	return value;
}

static void menu_write_handled(unsigned int value)
{
	FILE *f = fopen(PHYSICAL_DISC_HANDLED_FILE, "w");
	if (f) { fprintf(f, "%08x\n", value); fclose(f); }
}

static const char *menu_audio_mgl(void)
{
	const char *name = cfg.physical_disc_audio_cd;
	if (!name || !*name || !strcasecmp(name, "PSX") || !strcasecmp(name, "A0CD-PSX")) return "PSX.mgl";
	if (!strcasecmp(name, "PCE") || !strcasecmp(name, "A0CD-TurboGrafx16-CD")) return "TurboGrafx16-CD.mgl";
	if (!strcasecmp(name, "A0CD-MDPlus")) return "MDPlus.mgl";
	if (!strcasecmp(name, "A0CD-MegaCD")) return "MegaCD.mgl";
	if (!strcasecmp(name, "A0CD-Saturn")) return "Saturn.mgl";
	if (!strcasecmp(name, "A0CD-NeoGeoCD")) return "NeoGeoCD.mgl";
	if (!strcasecmp(name, "A0CD-3DO")) return "3DO.mgl";
	if (!strcasecmp(name, "A0CD-CDi")) return "CDi.mgl";
	if (!strcasecmp(name, "A0CD-SNES")) return "SNES-MSU1.mgl";
	return "PSX.mgl";
}

static const char *menu_mgl_for_disc(physical_disc_disc_t type)
{
	switch (type)
	{
	case PHYSICAL_DISC_DISC_MDPLUS: return "MDPlus.mgl";
	case PHYSICAL_DISC_DISC_MEGACD: return "MegaCD.mgl";
	case PHYSICAL_DISC_DISC_SATURN: return "Saturn.mgl";
	case PHYSICAL_DISC_DISC_PSX: return "PSX.mgl";
	case PHYSICAL_DISC_DISC_PCECD: return "TurboGrafx16-CD.mgl";
	case PHYSICAL_DISC_DISC_NEOGEO: return "NeoGeoCD.mgl";
	case PHYSICAL_DISC_DISC_3DO: return "3DO.mgl";
	case PHYSICAL_DISC_DISC_CDI: return "CDi.mgl";
	case PHYSICAL_DISC_DISC_SNES: return "SNES-MSU1.mgl";
	case PHYSICAL_DISC_DISC_AUDIO: return menu_audio_mgl();
	default: return NULL;
	}
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
	return mount_retry_at != 0 || menu_detecting;
}

void physical_disc_launch_cancel(void)
{
	suppress_pce_boot_osd = 0;
	suppress_neogeo_boot_osd_until = 0;
	if (spinup_stage) physical_disc_close();
	mount_retry_at = 0;
	mount_giveup_at = 0;
	clear_spinup_state();
}

void physical_disc_launch_reset(void)
{
	const char *name = user_io_get_core_name();
	if (!name || strncasecmp(name, "A0CD-", 5) || !strcasecmp(name, "A0CD-SNES") || !strcasecmp(name, "A0CD-MDPlus")) return;

	physical_disc_launch_cancel();
	physical_disc_close();
	physical_disc_acoustic_config(cfg.physical_disc_acoustic);

	begin_mount_window();
	clear_spinup_state();
	mount_retry_at = GetTimer(100);
}

int physical_disc_launch_consume_startup_osd_suppression(void)
{
	if (suppress_neogeo_boot_osd_until)
	{
		if (CheckTimer(suppress_neogeo_boot_osd_until)) suppress_neogeo_boot_osd_until = 0;
		else return 1;
	}

	if (!suppress_pce_boot_osd) return 0;
	suppress_pce_boot_osd = 0;
	return 1;
}

int physical_disc_launch_menu_tick(void)
{
	if (!is_menu()) return 0;
	if (menu_poll_at && !CheckTimer(menu_poll_at)) return 0;
	menu_poll_at = GetTimer(1000);
	menu_detecting = 1;

	if (!menu_environment_ready)
	{
		physical_disc_prepare_environment();
		menu_environment_ready = 1;
	}

	if (physical_disc_open(NULL))
	{
		physical_disc_close();
		unlink(PHYSICAL_DISC_HANDLED_FILE);
		menu_detecting = 0;
		return 0;
	}

	int disc_present = physical_disc_disc_present();
	if (!menu_boot_checked)
	{
		menu_boot_checked = 1;
		if (disc_present)
		{
			menu_ignore_boot_disc = 1;
			printf("DISC: startup disc detected, Auto Disc Discovery waits for a new insertion\n");
		}
	}

	if (menu_ignore_boot_disc)
	{
		if (!disc_present)
		{
			menu_ignore_boot_disc = 0;
			unlink(PHYSICAL_DISC_HANDLED_FILE);
			printf("DISC: startup disc removed, Auto Disc Discovery armed\n");
		}
		physical_disc_close();
		menu_detecting = 0;
		return 0;
	}

	toc_t toc;
	memset(&toc, 0, sizeof(toc));
	if (!disc_present || physical_disc_load_toc(&toc))
	{
		physical_disc_close();
		unlink(PHYSICAL_DISC_HANDLED_FILE);
		menu_detecting = 0;
		return 0;
	}

	unsigned int fingerprint = menu_disc_fingerprint(&toc);
	if (fingerprint && menu_read_handled() == fingerprint)
	{
		physical_disc_close();
		menu_detecting = 0;
		return 0;
	}

	physical_disc_disc_t type = physical_disc_identify();
	const char *mgl = menu_mgl_for_disc(type);
	menu_write_handled(fingerprint);
	physical_disc_close();
	menu_detecting = 0;

	if (!mgl) return 0;
	char path[512];
	snprintf(path, sizeof(path), "%s/%s", PHYSICAL_DISC_MGL_DIR, mgl);
	if (!FileExists(path)) return 0;
	xml_load(path);
	return 1;
}
