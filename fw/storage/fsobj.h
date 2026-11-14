#pragma once
#include "ff.h"

/* The one FATFS / FIL pair of the app (each carries a 4 KiB sector buffer). capsave and cfgfile use them
   one after the other, never at the same time. */
extern FATFS g_fs;
extern FIL g_fil;
