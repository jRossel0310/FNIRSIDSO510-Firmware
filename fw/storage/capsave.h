#pragma once
#include <stdint.h>
#include "csv.h"
#include "capname.h"

typedef enum { CAP_OK = 0, CAP_FULL = 1, CAP_ERR = 2 } cap_res_t;

/* Mount volume 0, write rec[0..n) as CAP_DIR/CAPnnnn.CSV (next free number, folder created if
   missing), unmount. *num_out = number used (also on CAP_FULL, for the deleted partial file).
   CAP_FULL: drive or numbers exhausted (partial file deleted). CAP_ERR: any other failure. */
cap_res_t capsave_write(const uint8_t *rec, int n, const csv_meta_t *m, int *num_out);
