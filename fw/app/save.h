#pragma once
#include <stdint.h>
#include "csv.h"
#include "msc.h"
/* Save one capture as CSV. The medium is reported removed while writing. If a USB host has configured
   the device (also while the bus is suspended), first waits for it to go idle (<= 5 s, ends early if
   the host goes away), then soft-disconnects (D+ off) for >= 0.5 s plus the write and re-attaches, so
   the host sees a real unplug/replug. poll (may be NULL) is called in every wait, e.g. for a long
   Power press. Returns a 14-column toast text: " Saved CAPnnnn", " Drive full   ", " Save failed  "
   or " Busy, retry  ". */
const char *save_capture(const uint8_t *rec, int n, const csv_meta_t *m, msc_t *msc, void (*poll)(void));
