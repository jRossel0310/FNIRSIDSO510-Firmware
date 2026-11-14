#pragma once
#include <stdint.h>

#define MSC_BLOCK 4096u
#define MSC_PKT   64u

typedef struct {
    int (*read)(uint32_t lba, uint8_t *buf);          /* 0 ok */
    int (*write)(uint32_t lba, const uint8_t *buf);   /* 0 ok */
    uint32_t blocks;
} msc_disk_t;

typedef struct {
    const msc_disk_t *disk;
    uint8_t state, io;                 /* io: 0 none, 1 read pending, 2 write pending */
    uint8_t tag[4];
    uint32_t hi;                       /* dCBWDataTransferLength */
    uint32_t residue;
    uint8_t status;                    /* CSW status */
    uint8_t key, asc, ascq;            /* sense */
    uint8_t present, attention, wrote;
    uint32_t lba, nblk;                /* remaining READ/WRITE blocks */
    uint32_t pos, len;                 /* data-in: bytes sent / valid in buf; data-out: bytes received */
    uint32_t left;                     /* zero-fill / discard bytes remaining */
    uint32_t last_write_ms;
    uint8_t buf[MSC_BLOCK];
} msc_t;

void msc_init(msc_t *m, const msc_disk_t *d);       /* medium present */
void msc_reset(msc_t *m);                           /* USB reset / BOT reset: back to IDLE */
/* Host -> device bulk packet. 0 accepted; -1 not now (caller keeps it NAKed and retries). */
int msc_rx(msc_t *m, const uint8_t *pkt, int len, uint32_t now_ms);
/* Next device -> host packet into pkt[MSC_PKT]. Returns its length, or -1 if nothing to send. */
int msc_tx(msc_t *m, uint8_t *pkt);
/* Pending block read/write (main context only). */
void msc_task(msc_t *m);
/* 0: medium removed (02/3A/00). 0 -> 1: one UNIT ATTENTION (06/28/00), then ready. */
void msc_set_present(msc_t *m, int present);
/* 1 while a command is in progress or a WRITE was received within the last 1000 ms. */
int msc_busy(const msc_t *m, uint32_t now_ms);
