#pragma once
#include <stdint.h>

#define MSC_BLOCK 4096u
#define MSC_PKT   64u

typedef struct {
    int (*read)(uint32_t lba, uint8_t *buf);          /* 0 ok */
    int (*write)(uint32_t lba, const uint8_t *buf);   /* 0 ok */
    uint32_t blocks;
} msc_disk_t;

/* Concurrency: msc_rx, msc_tx and msc_reset run in the USB interrupt. msc_task, msc_set_present and
   msc_busy run in main context and take the lock (mask the USB IRQ) around every access to shared
   state, but never around the block read/write itself. While that I/O is in flight (`inflight`),
   msc_rx and msc_tx refuse (-1), so the ISR never touches buf, lba or the state; an msc_reset in that
   window bumps `gen` and the finished I/O's result is dropped. */
typedef struct {
    const msc_disk_t *disk;
    void (*lock)(void), (*unlock)(void);   /* NULL: no locking (host tests) */
    uint8_t state, io;                 /* io: 0 none, 1 read pending, 2 write pending; cleared after the I/O */
    uint8_t inflight, gen;             /* block I/O running in msc_task; bumped by msc_reset */
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

void msc_init(msc_t *m, const msc_disk_t *d);       /* medium present, no lock */
void msc_set_lock(msc_t *m, void (*lock)(void), void (*unlock)(void));
void msc_reset(msc_t *m);                           /* USB reset / BOT reset: back to IDLE (ISR context) */
/* Host -> device bulk packet. 0 accepted; -1 not now (caller keeps it NAKed and retries). */
int msc_rx(msc_t *m, const uint8_t *pkt, int len, uint32_t now_ms);
/* Next device -> host packet into pkt[MSC_PKT]. Returns its length, or -1 if nothing to send. */
int msc_tx(msc_t *m, uint8_t *pkt);
/* Pending block read/write (main context only). After it returns, the caller must re-offer a held
   OUT packet and start an IN packet: the finished I/O raises no USB event of its own. */
void msc_task(msc_t *m);
/* 0: medium removed (02/3A/00); a WRITE in progress fails with 02/3A/00 and the rest of its data is
   discarded. 0 -> 1: one UNIT ATTENTION (06/28/00), then ready. */
void msc_set_present(msc_t *m, int present);
/* 1 while a command is in progress or a WRITE was received within the last 1000 ms. */
int msc_busy(const msc_t *m, uint32_t now_ms);
