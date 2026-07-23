#ifndef PHYSICAL_DISC_MGL_H
#define PHYSICAL_DISC_MGL_H






















void physical_disc_launch_startup(void);



void physical_disc_launch_poll(void);



int physical_disc_launch_menu_tick(void);


int physical_disc_launch_busy(void);


void physical_disc_launch_cancel(void);

void physical_disc_launch_reset(void);

int physical_disc_launch_consume_startup_osd_suppression(void);



int physical_disc_launch_load_disc(void);



#define PHYSICAL_DISC_MENU_SENTINEL "\x01physical_disc"



int physical_disc_menu_row(char *out, int outsz);


int physical_disc_is_menu_row(const char *name);




int physical_disc_mount_current_core(void);





int physical_disc_swap_current_core(void);

#endif
