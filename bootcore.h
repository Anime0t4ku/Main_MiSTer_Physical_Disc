



#ifndef __BOOTCORE_H__
#define __BOOTCORE_H__

char *getcoreName(char *path);
char *getcoreExactName(char *path);
char *replaceStr(const char *str, const char *oldstr, const char *newstr);
char *loadLastcore();
char *findCore(const char *name, char *coreName, int indent);
void bootcore_init(const char *path);






int find_core_rbf(const char *coreName, char *out, int outsz);

extern char bootcoretype[64];
extern int16_t btimeout;

#endif 
