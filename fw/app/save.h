#pragma once
#include <stdint.h>
#include "csv.h"
#include "msc.h"
/* Save one capture as CSV. If a USB host is attached, waits for it to go idle (<= 5 s), reports
   the medium as removed while writing, then re-inserts it (UNIT ATTENTION). Returns a 14-column
   toast text: " Saved CAPnnnn", " Drive full   ", " Save failed  " or " Busy, retry  ". */
const char *save_capture(const uint8_t *rec, int n, const csv_meta_t *m, msc_t *msc);
