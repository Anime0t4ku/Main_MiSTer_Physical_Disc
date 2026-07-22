#ifndef _FAT16_H_INCLUDED
#define _FAT16_H_INCLUDED

#include <stdio.h>
#include <unistd.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdbool.h>
#include "spi.h"

struct fileZipArchive;

struct fileTYPE
{
	fileTYPE();
	~fileTYPE();
	int opened();

	FILE           *filp;
	int             mode;
	int             type;
	fileZipArchive *zip;
	__off64_t       size;
	__off64_t       offset;
	char            path[1024];
	char            name[261];
};

struct direntext_t
{
	dirent de;
	int  cookie;
#define DT_EXT_ZIP    0x1
	unsigned int flags;
	char datecode[16];
	char altname[256];
};

struct fileTextReader
{
	fileTextReader();
	~fileTextReader();

	size_t size;
	char *buffer;
	char *pos;
};

int flist_nDirEntries();
void flist_select_by_name(const char *name);
int flist_iFirstEntry();
void flist_iFirstEntryInc();
int flist_iSelectedEntry();
direntext_t* flist_DirItem(int n);
direntext_t* flist_SelectedItem();
char* flist_Path();
char* flist_GetPrevNext(const char* base_path, const char* file, const char* ext, int next);


#define SCANF_INIT       0 
#define SCANF_NEXT       1 
#define SCANF_PREV      -1 
#define SCANF_NEXT_PAGE  2 
#define SCANF_PREV_PAGE -2 
#define SCANF_SET_ITEM   3 
#define SCANF_END        4 
#define SCANF_NEXT_CHAR  5 
#define SCANF_PREV_CHAR -5 


#define SCANO_DIR        0b000000001 
#define SCANO_UMOUNT     0b000000010 
#define SCANO_CORES      0b000000100 
#define SCANO_TXT        0b000001000
#define SCANO_NEOGEO     0b000010000
#define SCANO_NOENTER    0b000100000
#define SCANO_NOZIP      0b001000000
#define SCANO_CLEAR      0b010000000 
#define SCANO_SAVES      0b100000000

void FindStorage();
int  getStorage(int from_setting);
void setStorage(int dev);
int  isUSBMounted();

int  FileOpenZip(fileTYPE *file, const char *name, uint32_t crc32);
int  FileOpenEx(fileTYPE *file, const char *name, int mode, char mute = 0, int use_zip = 1);
int  FileOpen(fileTYPE *file, const char *name, char mute = 0);
void FileClose(fileTYPE *file);

__off64_t FileGetSize(fileTYPE *file);

int FileSeek(fileTYPE *file, __off64_t offset, int origin);
int FileSeekLBA(fileTYPE *file, uint32_t offset);

int FileReadAdv(fileTYPE *file, void *pBuffer, int length, int failres = 0);
int FileReadSec(fileTYPE *file, void *pBuffer);
int FileWriteAdv(fileTYPE *file, void *pBuffer, int length, int failres = 0);
int FileWriteSec(fileTYPE *file, void *pBuffer);
int FileCreatePath(const char *dir);

int FileExists(const char *name, int use_zip = 1);
int FileCanWrite(const char *name);
int PathIsDir(const char *name, int use_zip = 1);
struct stat64* getPathStat(const char *path);

#define SAVE_DIR "saves"
void FileGenerateSavePath(const char *name, char* out_name, int ext_replace = 1);

#define SAVESTATE_DIR "savestates"
void FileGenerateSavestatePath(const char *name, char* out_name, int sufx);

#define SCREENSHOT_DIR "screenshots"
#define SCREENSHOT_DEFAULT "screen"
void FileGenerateScreenshotName(const char *name, char *out_name, const char *extension, int buflen);

int FileSave(const char *name, void *pBuffer, int size);
int FileLoad(const char *name, void *pBuffer, int size); 
int FileDelete(const char *name);
int DirDelete(const char *name);


#define CONFIG_DIR "config"
const char* GetNameFromPath(char* path);
int FileSaveConfig(const char *name, void *pBuffer, int size);
int FileLoadConfig(const char *name, void *pBuffer, int size); 
int FileDeleteConfig(const char *name);

void AdjustDirectory(char *path);
int ScanDirectory(char* path, int mode, const char *extension, int options, const char *prefix = NULL, const char *filter = NULL);

void prefixGameDir(char *dir, size_t dir_len);
int findGamesDir(char *dir, size_t dir_len);
int findDocsDir(char *dir, size_t dir_len);

struct gameAssetValidator
{
	int (*fn)(const char *path, void *ctx);
	void *ctx;
};


int findGameAsset(char *path,
	size_t path_len,
	const char *rom_path,
	uint32_t crc,
	const char *ext,
	const char *core_dir,
	const char *pcecd_dir,
	gameAssetValidator *validator);

const char *getStorageDir(int dev);
const char *getRootDir();
const char *getFullPath(const char *name);

uint32_t getFileType(const char *name);
int isXmlName(const char *path); 

bool FileOpenTextReader(fileTextReader *reader, const char *path);
const char* FileReadLine(fileTextReader *reader);

#define LOADBUF_SZ (1024*1024)

#define COEFF_DIR "filters"
#define GAMMA_DIR "gamma"
#define AFILTER_DIR "filters_audio"
#define SMASK_DIR "shadow_masks"
#define PRESET_DIR "presets"
#define GAMES_DIR "games"
#define CIFS_DIR "cifs"
#define DOCS_DIR "docs"

void create_path(const char *base_dir, const char* sub_dir);

#endif
