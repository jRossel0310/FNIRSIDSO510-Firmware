#pragma once
#include <stdint.h>

#define CFG_PATH "/DSO510.CFG"
/* Mount, read the settings file into buf (at most cap bytes), unmount. Returns the file's size in bytes
   (which may exceed cap: then only cap bytes were read), or -1 if there is no file or on any error. */
int cfgfile_load(uint8_t *buf, uint32_t cap);
/* Mount, replace the settings file with buf[0..len), unmount. Returns 0 on success, -1 on any error. */
int cfgfile_save(const uint8_t *buf, uint32_t len);
