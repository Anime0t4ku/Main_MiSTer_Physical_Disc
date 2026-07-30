#ifndef MEGADRIVE_MDPLUS_H
#define MEGADRIVE_MDPLUS_H

void mdplus_init(const char *rom_path);
void mdplus_poll();
void mdplus_cd_session_poll();
int mdplus_cd_suppress_osd();

#endif
