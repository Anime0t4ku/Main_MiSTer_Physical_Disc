#ifndef MISTER_PHYSICAL_DISC_INCLUDED
#define MISTER_PHYSICAL_DISC_INCLUDED

#include <stdint.h>
#include "../../cd.h"






















#define PHYSICAL_DISC_SENTINEL "*PHYSICAL_DISC*"
#define PHYSICAL_DISC_RAW  2352
#define PHYSICAL_DISC_SUB  96






#define PHYSICAL_DISC_SWAP_DWELL_MS 500




void physical_disc_set_device(const char *dev);




int physical_disc_open(const char *dev);


int physical_disc_disc_present();


int physical_disc_media_changed();



int physical_disc_drive_busy();






void physical_disc_swap_enable(int enable);
int physical_disc_swap_consume(void);
int physical_disc_swap_ejected(void);





int physical_disc_swap_happened(void);





int physical_disc_load_toc(toc_t *toc);




int physical_disc_current_toc(toc_t *toc);
int physical_disc_toc_audio_only(const toc_t *toc);





int physical_disc_read_sector(int lba, uint8_t *dst, uint8_t *sub96);






int physical_disc_read_sector_sub(int lba, uint8_t *dst, uint8_t *sub96);




int physical_disc_read_data2048(int lba, uint8_t *dst);




void physical_disc_seek_hint(int lba);


typedef enum {
	PHYSICAL_DISC_DISC_NONE = 0,
	PHYSICAL_DISC_DISC_MEGACD,
	PHYSICAL_DISC_DISC_SATURN,
	PHYSICAL_DISC_DISC_PSX,
	PHYSICAL_DISC_DISC_PCECD,
	PHYSICAL_DISC_DISC_NEOGEO,
	PHYSICAL_DISC_DISC_3DO,
	PHYSICAL_DISC_DISC_AUDIO,
	PHYSICAL_DISC_DISC_UNKNOWN,
} physical_disc_disc_t;

physical_disc_disc_t physical_disc_identify();
const char *physical_disc_disc_name(physical_disc_disc_t t);




typedef enum {
	PHYSICAL_DISC_REGION_UNKNOWN = 0,
	PHYSICAL_DISC_REGION_JP,
	PHYSICAL_DISC_REGION_US,
	PHYSICAL_DISC_REGION_EU,
} physical_disc_region_t;



physical_disc_region_t physical_disc_region();




physical_disc_region_t physical_disc_region_from_md_header(const uint8_t *hdr, int len);


const char *physical_disc_region_name(physical_disc_region_t r);








typedef enum {
	PHYSICAL_DISC_EV_NONE = 0,
	PHYSICAL_DISC_EV_DISC_IN,      
	PHYSICAL_DISC_EV_DISC_OUT,     
} physical_disc_event_t;

int physical_disc_watch_start(void);
void physical_disc_watch_stop(void);
int physical_disc_watching(void);






physical_disc_event_t physical_disc_poll_event(physical_disc_disc_t *type, physical_disc_region_t *region, int *initial);




void physical_disc_forget_disc(void);




int physical_disc_disc_label(char *out, int outsz);



int physical_disc_disc_serial(char *out, int outsz);
int physical_disc_save_name(physical_disc_disc_t type, char *out, int outsz);


const char *physical_disc_console_name(physical_disc_disc_t t);



int physical_disc_menu_status(char *name, int namesz, physical_disc_disc_t *type);



int physical_disc_menu_dirty(void);

void physical_disc_close();

#endif
