// dvd_detect.h — detect a DVD-Video disc in the optical drive.

#ifndef MISTER_DVD_DETECT_INCLUDED
#define MISTER_DVD_DETECT_INCLUDED

// True if the open drive fd holds a DVD-Video disc: an ISO9660 volume with a
// VIDEO_TS directory in the root. Uses READ(10), which works on DVD media
// (unlike the CD layer's READ CD / 2352 path). Called from the shared disc
// identification dispatch (physical_disc_identify).
int dvd_video_probe(int fd);

#endif
