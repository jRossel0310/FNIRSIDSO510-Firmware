# Storage, CSV Export and USB Drive (Sub-project 2) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Long-press Run saves the displayed capture as `/CSV/CAPnnnn.CSV` on the scope's 5 MiB drive, and the drive mounts automatically on a computer over USB (with an eject/re-insert when the scope saves while connected).

**Architecture:** Pure logic, unit-tested on the PC, goes in `fw/core/`: flash map guard, file names, CSV formatter, the USB mass-storage protocol (BOT + SCSI) and USB standard requests. `fw/storage/` holds hardware-independent glue around FatFs: the volume block layer, FatFs disk I/O and the CSV save. It is host-tested against a RAM fake of the SPI flash. Hardware drivers live in `fw/common/`: SPI flash write/erase and a polled USB device driver. `fw/app/` wires long-press Run to the save and runs the USB driver from the main loop and from every blocking wait.

**Tech Stack:** `arm-none-eabi-gcc` (Cortex-M4F hard float), host `cc` for tests, FatFs R0.15a (ChaN), GNU make.

**Spec:** `docs/superpowers/specs/2026-11-13-dso510-firmware-design.md`. Read § "Sub-project 2: storage, CSV export, USB drive" and § "CSV format". Hardware facts are in `NOTES.md` § "SPI NOR flash" and § "first USB contact".

## Global Constraints

- Never write or erase internal flash below 0x08007000. Never touch the factory page 0x08006000.
- **SPI flash erase and program are only allowed inside the user volume `[0x000000, 0x500000)`.** The 3 MiB system volume at 0x500000 (stock UI graphics) is never written. The guard is `flashmap_writable()` and it is enforced inside the driver.
- User volume: FAT12, 4096-byte sectors, 1280 sectors, flash offset 0. It is used as is and never reformatted on the device.
- Every app must implement power-off: hold Power for about 1 s, which releases PD3.
- `fw/core/*` includes no hardware header and compiles with host `cc -std=c99 -Wall -Wextra -Werror`.
- `fw/storage/*` includes no hardware header. It only uses `spiflash.h`, `flashmap.h`, FatFs and core headers, so the host tests can link it against `tests/fake_spiflash.c`.
- `spiflash.c` must never call `delay_ms()`, because `delay_ms()` runs the USB idle hook, which may itself use the flash. Its waits use `millis()` directly.
- CSV format: exactly as in spec § CSV format and § CSV save. `FW_VERSION` is `"0.2.0"`.
- USB: VID:PID `2e3c:5720`, MSC Bulk-Only/SCSI, 64-byte bulk EP 0x81 IN / 0x01 OUT, 1280 × 4096-byte blocks, removable (RMB=1).
- Flashing: put the scope in update mode (hold the update button, press Power), then run `tools/flash.sh <bin>`.
- Commit after every task, adding files by name (never `git add -A`; never commit `docs/` photos). Trailer line: `Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp`.

## File Structure

| File | Responsibility |
|---|---|
| `fw/core/flashmap.[ch]` | Flash layout constants + `flashmap_writable()` guard |
| `fw/core/capname.[ch]` | `CAPnnnn.CSV` naming: parse, make, next number |
| `fw/core/csv.[ch]` | CSV header + row formatting (integer math, no printf) |
| `fw/core/msc.[ch]` | USB Mass Storage Bulk-Only transport + SCSI commands over a block-device interface |
| `fw/core/usb_std.[ch]` | USB descriptors + standard/class control requests |
| `fw/common/spiflash.[ch]` | SPI NOR: read, sector erase, page program, sector write (guarded) |
| `fw/common/usbd.[ch]` | Polled USB FS device driver (AT32 USBD, PMA, EP0 + EP1 bulk) |
| `fw/common/startup.c` | add `memcmp`, `strchr`, `strlen` (needed by FatFs) |
| `fw/common/board.[ch]` | `delay_ms()` calls weak `board_idle()` |
| `fw/third_party/fatfs/` | FatFs R0.15a unmodified, except `ffconf.h` (our configuration) |
| `fw/storage/vol.[ch]` | User volume as 4 KiB blocks on the SPI flash |
| `fw/storage/diskio_flash.c` | FatFs `disk_*` functions on top of `vol` |
| `fw/storage/capsave.[ch]` | Write one capture to `/CSV/CAPnnnn.CSV` via FatFs |
| `fw/app/save.[ch]` | Save flow incl. USB eject / re-insert around the write |
| `fw/app/main.c` | double-buffered record, capture-time metadata snapshot, long-press Run → save, USB polling |
| `fw/apps/flashtest/main.c` | On-device test: guarded write/erase on the last user sector with backup + restore |
| `fw/apps/usbdisk/main.c` | On-device test: the drive over USB, nothing else |
| `tests/fake_spiflash.[ch]` | RAM model of the 8 MiB NOR (erase → 0xFF, program ANDs bits, guard) |
| `tests/test_flashmap.c`, `test_capname.c`, `test_csv.c`, `test_capsave.c`, `test_msc.c`, `test_usb_std.c` | Host tests |
| `tests/Makefile`, `fw/Makefile` | Build changes for storage + FatFs |

---

### Task 1: `flashmap` guard and `capname`

**Files:**
- Create: `fw/core/flashmap.h`, `fw/core/flashmap.c`, `fw/core/capname.h`, `fw/core/capname.c`
- Create: `tests/test_flashmap.c`, `tests/test_capname.c`
- Modify: `tests/main.c` (register the two tests)

**Interfaces:**
- Produces: `FLASH_SECTOR`, `USER_VOL_BASE`, `USER_VOL_SECTORS`, `USER_VOL_BYTES`, `SYS_VOL_BASE`, `int flashmap_writable(uint32_t addr, uint32_t len)`; `CAP_DIR`, `CAP_MAX`, `int capname_number(const char *name)`, `void capname_make(char out[13], int n)`, `int capname_next(int highest)`.

- [ ] **Step 1: Write the failing tests**

`tests/test_flashmap.c`:
```c
#include "test.h"
#include "flashmap.h"

void test_flashmap(void) {
    CHECK_EQ(USER_VOL_BYTES, 0x500000u);
    CHECK_EQ(SYS_VOL_BASE, 0x500000u);
    CHECK_EQ(flashmap_writable(0, 4096), 1);
    CHECK_EQ(flashmap_writable(0x4FF000, 4096), 1);          /* last user sector */
    CHECK_EQ(flashmap_writable(0x4FFFFF, 1), 1);
    CHECK_EQ(flashmap_writable(0x4FF000, 4097), 0);          /* crosses into system volume */
    CHECK_EQ(flashmap_writable(0x500000, 1), 0);             /* system volume */
    CHECK_EQ(flashmap_writable(0x7FF000, 4096), 0);
    CHECK_EQ(flashmap_writable(0, 0), 0);                    /* empty range */
    CHECK_EQ(flashmap_writable(0, USER_VOL_BYTES), 1);
    CHECK_EQ(flashmap_writable(0, USER_VOL_BYTES + 1), 0);
    CHECK_EQ(flashmap_writable(0xFFFFFFFFu, 2), 0);          /* wraps */
    CHECK_EQ(flashmap_writable(0x100, 0xFFFFFF00u), 0);      /* wraps */
}
```

`tests/test_capname.c`:
```c
#include "test.h"
#include "capname.h"

void test_capname(void) {
    char b[13];
    CHECK_EQ(capname_number("CAP0001.CSV"), 1);
    CHECK_EQ(capname_number("CAP0042.CSV"), 42);
    CHECK_EQ(capname_number("CAP9999.CSV"), 9999);
    CHECK_EQ(capname_number("cap0042.csv"), 42);              /* case-insensitive */
    CHECK_EQ(capname_number("CAP0042.TXT"), -1);
    CHECK_EQ(capname_number("CAP042.CSV"), -1);
    CHECK_EQ(capname_number("CAPX042.CSV"), -1);
    CHECK_EQ(capname_number("LOG0042.CSV"), -1);
    CHECK_EQ(capname_number("CAP00420.CSV"), -1);
    CHECK_EQ(capname_number(""), -1);
    capname_make(b, 7);    CHECK_STR(b, "CAP0007.CSV");
    capname_make(b, 1234); CHECK_STR(b, "CAP1234.CSV");
    capname_make(b, 9999); CHECK_STR(b, "CAP9999.CSV");
    CHECK_EQ(capname_next(0), 1);
    CHECK_EQ(capname_next(41), 42);
    CHECK_EQ(capname_next(9998), 9999);
    CHECK_EQ(capname_next(9999), -1);                         /* exhausted */
}
```

In `tests/main.c`, add `void test_flashmap(void);` and `void test_capname(void);` to the declarations, and call `test_flashmap(); test_capname();` after `test_autoset();`.

- [ ] **Step 2: Run the tests and confirm they fail**

Run: `make -C tests`
Expected: compile error, `flashmap.h: No such file or directory`.

- [ ] **Step 3: Implement**

`fw/core/flashmap.h`:
```c
#pragma once
#include <stdint.h>
/* External SPI NOR layout (NOTES.md § SPI NOR flash). */
#define FLASH_SECTOR     4096u
#define USER_VOL_BASE    0x000000u
#define USER_VOL_SECTORS 1280u
#define USER_VOL_BYTES   (USER_VOL_SECTORS * FLASH_SECTOR)   /* 0x500000 */
#define SYS_VOL_BASE     0x500000u                           /* stock system volume: never written */

/* 1 if [addr, addr + len) is non-empty and lies entirely inside the user volume, else 0. */
int flashmap_writable(uint32_t addr, uint32_t len);
```

`fw/core/flashmap.c`:
```c
#include "flashmap.h"

int flashmap_writable(uint32_t addr, uint32_t len) {
    if (len == 0) return 0;
    if (addr >= USER_VOL_BYTES) return 0;
    if (len > USER_VOL_BYTES - addr) return 0;    /* also rejects wrap-around */
    return 1;
}
```

`fw/core/capname.h`:
```c
#pragma once
/* Capture files: CAP_DIR/CAP0001.CSV .. CAP9999.CSV (8.3 names, FatFs without LFN). */
#define CAP_DIR "/CSV"
#define CAP_MAX 9999

/* "CAP0042.CSV" (any case) -> 42; anything else -> -1. */
int capname_number(const char *name);
/* 42 -> "CAP0042.CSV" (out holds 12 chars + NUL). n must be 0..9999. */
void capname_make(char out[13], int n);
/* Number to use after the highest existing one (0 if none); -1 when 9999 is taken. */
int capname_next(int highest);
```

`fw/core/capname.c`:
```c
#include "capname.h"

static char up(char c) { return (c >= 'a' && c <= 'z') ? (char)(c - 32) : c; }

int capname_number(const char *name) {
    static const char pre[] = "CAP", suf[] = ".CSV";
    for (int i = 0; i < 3; i++) if (up(name[i]) != pre[i]) return -1;
    int n = 0;
    for (int i = 3; i < 7; i++) {
        if (name[i] < '0' || name[i] > '9') return -1;
        n = n * 10 + (name[i] - '0');
    }
    for (int i = 0; i < 4; i++) if (up(name[7 + i]) != suf[i]) return -1;
    if (name[11] != 0) return -1;
    return n;
}

void capname_make(char out[13], int n) {
    static const char tmpl[] = "CAP0000.CSV";
    for (int i = 0; i < 12; i++) out[i] = tmpl[i];
    for (int i = 6; i >= 3; i--) { out[i] = (char)('0' + n % 10); n /= 10; }
}

int capname_next(int highest) { return highest >= CAP_MAX ? -1 : highest + 1; }
```
Note that `capname_number` reads `name[i]` one character at a time and stops at the first mismatch, so a short string such as `""` returns -1 before it reads past its NUL.

- [ ] **Step 4: Run the tests and confirm they pass**

Run: `make -C tests`
Expected: `N checks, 0 failed`. N is the previous 458 plus the new checks.

- [ ] **Step 5: Commit**

```bash
git add fw/core/flashmap.h fw/core/flashmap.c fw/core/capname.h fw/core/capname.c tests/test_flashmap.c tests/test_capname.c tests/main.c
git commit -m "core: flash map write guard and CAPnnnn.CSV naming

Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp"
```

---

### Task 2: `csv` formatter

**Files:**
- Create: `fw/core/csv.h`, `fw/core/csv.c`, `tests/test_csv.c`
- Modify: `tests/main.c` (register `test_csv`)

**Interfaces:**
- Produces: `FW_VERSION`, `csv_meta_t`, `CSV_ROW_MAX`, `int csv_header(char *buf, int cap, const csv_meta_t *m)`, `int csv_row(char *buf, const csv_meta_t *m, int32_t i, uint8_t code)`.

Number rules, all integer math with round-half-away-from-zero:
- **Microvolts per ADC code** = `vdiv_mv * 40`, times 10 if `probe_x10`. This is exact, because one division is 25 codes.
- **`time_s`** = `(i − trig_index) / rate_hz`, printed with 9 decimals (ns resolution).
- **`volts`** = `(code − zero_code) × µV/code`, printed with 4 decimals.
- **`volts_per_div`** has 4 decimals and includes the probe factor.
- **`level_v`** = `(trig_level_code − zero_code) × µV/code`, with 4 decimals.
- **`volts_per_code`** has 6 decimals.

- [ ] **Step 1: Write the failing test**

`tests/test_csv.c`:
```c
#include "test.h"
#include "csv.h"

static csv_meta_t base(void) {
    csv_meta_t m;
    memset(&m, 0, sizeof m);
    m.rate_hz = 1000000; m.vdiv_mv = 1000; m.coupling_dc = 1; m.probe_x10 = 0;
    m.trig_mode = 0; m.edge = 0; m.zero_code = 128; m.trig_level_code = 153;
    m.trig_index = 100; m.offset_pwm = 2033;
    return m;
}

void test_csv(void) {
    char b[600];
    csv_meta_t m = base();
    int n = csv_header(b, sizeof b, &m);
    CHECK(n > 0);
    b[n > 0 ? n : 0] = 0;
    CHECK_STR(b,
        "# DSO-510 custom firmware 0.2.0\n"
        "# sample_rate_hz=1000000\n"
        "# volts_per_div=1.0000\n"
        "# coupling=DC\n"
        "# probe=1\n"
        "# trigger=Auto,rising,1.0000\n"
        "# trigger_index=100\n"
        "# cal=offset_pwm=2033,volts_per_code=0.040000\n"
        "time_s,volts,adc_code\n");
    CHECK_EQ(csv_header(b, 10, &m), -1);                       /* too small */

    m.coupling_dc = 0; m.probe_x10 = 1; m.trig_mode = 2; m.edge = 1; m.vdiv_mv = 10;
    m.trig_level_code = 103;
    n = csv_header(b, sizeof b, &m); b[n] = 0;
    CHECK(strstr(b, "# volts_per_div=0.1000\n") != 0);        /* 10 mV x10 probe */
    CHECK(strstr(b, "# coupling=AC\n") != 0);
    CHECK(strstr(b, "# probe=10\n") != 0);
    CHECK(strstr(b, "# trigger=Single,falling,-0.1000\n") != 0);   /* -25 codes * 4 mV */
    CHECK(strstr(b, "volts_per_code=0.004000\n") != 0);
    m = base(); m.trig_mode = 1;
    n = csv_header(b, sizeof b, &m); b[n] = 0;
    CHECK(strstr(b, "# trigger=Normal,rising,") != 0);

    m = base();
    n = csv_row(b, &m, 100, 128); b[n] = 0; CHECK_STR(b, "0.000000000,0.0000,128\n");
    n = csv_row(b, &m, 99, 153);  b[n] = 0; CHECK_STR(b, "-0.000001000,1.0000,153\n");
    n = csv_row(b, &m, 101, 103); b[n] = 0; CHECK_STR(b, "0.000001000,-1.0000,103\n");
    n = csv_row(b, &m, 1100, 0);  b[n] = 0; CHECK_STR(b, "0.001000000,-5.1200,0\n");
    m.rate_hz = 12000000;                                       /* 83.33 ns per sample */
    n = csv_row(b, &m, 101, 128); b[n] = 0; CHECK_STR(b, "0.000000083,0.0000,128\n");
    n = csv_row(b, &m, 99, 128);  b[n] = 0; CHECK_STR(b, "-0.000000083,0.0000,128\n");
    m.rate_hz = 2400; m.trig_index = 4800;
    n = csv_row(b, &m, 0, 128);   b[n] = 0; CHECK_STR(b, "-2.000000000,0.0000,128\n");
    m = base(); m.vdiv_mv = 20;                                 /* 800 uV per code */
    n = csv_row(b, &m, 100, 129); b[n] = 0; CHECK_STR(b, "0.000000000,0.0008,129\n");
    m.vdiv_mv = 10;                                             /* 400 uV per code: 0.0004 */
    n = csv_row(b, &m, 100, 127); b[n] = 0; CHECK_STR(b, "0.000000000,-0.0004,127\n");
    m = base(); m.vdiv_mv = 10000; m.probe_x10 = 1; m.zero_code = 255; /* worst-case width */
    n = csv_row(b, &m, -4700, 0);
    CHECK(n <= CSV_ROW_MAX);
    b[n] = 0; CHECK_STR(b, "-0.004800000,-1020.0000,0\n");
}
```
Register it in `tests/main.c` (`void test_csv(void);` and call `test_csv();`).

- [ ] **Step 2: Run the tests and confirm they fail**

Run: `make -C tests`
Expected: compile error, `csv.h: No such file or directory`.

- [ ] **Step 3: Implement**

`fw/core/csv.h`:
```c
#pragma once
#include <stdint.h>

#define FW_VERSION "0.2.0"
#define CSV_ROW_MAX 40          /* longest row incl. '\n' */

typedef struct {
    uint32_t rate_hz;           /* sample rate of the record */
    uint16_t vdiv_mv;           /* V/div table value (probe factor NOT applied) */
    uint8_t coupling_dc;        /* 1 DC, 0 AC */
    uint8_t probe_x10;          /* 1 = x10 probe */
    uint8_t trig_mode;          /* 0 Auto, 1 Normal, 2 Single */
    uint8_t edge;               /* 0 rising, 1 falling */
    int16_t zero_code;          /* ADC code of 0 V (128 + offset) */
    int16_t trig_level_code;    /* trigger level as ADC code */
    int32_t trig_index;         /* sample index of the trigger point */
    uint16_t offset_pwm;        /* offset PWM used for this record */
} csv_meta_t;

/* All header lines incl. the column-name line. Returns the length written (no NUL),
   or -1 if cap is too small. */
int csv_header(char *buf, int cap, const csv_meta_t *m);
/* Row for sample i with ADC code `code`, '\n'-terminated, no NUL. buf must hold CSV_ROW_MAX.
   Returns the length. */
int csv_row(char *buf, const csv_meta_t *m, int32_t i, uint8_t code);
```

`fw/core/csv.c`:
```c
#include "csv.h"

static int64_t rdiv(int64_t n, int64_t d) { return n >= 0 ? (n + d / 2) / d : -((-n + d / 2) / d); }

/* v in units of 10^-dec, printed as a decimal with exactly dec decimals. Returns length. */
static int put_fixed(char *p, int64_t v, int dec) {
    char t[24];
    int n = 0, k = 0;
    uint64_t u = v < 0 ? (uint64_t)(-v) : (uint64_t)v;
    do { t[n++] = (char)('0' + u % 10); u /= 10; } while (u || n <= dec);
    if (v < 0) p[k++] = '-';
    while (n--) {
        p[k++] = t[n];
        if (dec && n == dec) p[k++] = '.';
    }
    return k;
}

static int32_t uv_per_code(const csv_meta_t *m) { return (int32_t)m->vdiv_mv * 40 * (m->probe_x10 ? 10 : 1); }

typedef struct { char *p; int n, cap, err; } sb_t;
static void sb_s(sb_t *b, const char *s) {
    while (*s) { if (b->n >= b->cap) { b->err = 1; return; } b->p[b->n++] = *s++; }
}
static void sb_fx(sb_t *b, int64_t v, int dec) { char t[32]; int k = put_fixed(t, v, dec); t[k] = 0; sb_s(b, t); }

int csv_header(char *buf, int cap, const csv_meta_t *m) {
    static const char *modes[3] = {"Auto", "Normal", "Single"};
    sb_t b = {buf, 0, cap, 0};
    int32_t uvpc = uv_per_code(m);
    sb_s(&b, "# DSO-510 custom firmware " FW_VERSION "\n# sample_rate_hz=");
    sb_fx(&b, m->rate_hz, 0);
    sb_s(&b, "\n# volts_per_div=");
    sb_fx(&b, (int64_t)m->vdiv_mv * 10 * (m->probe_x10 ? 10 : 1), 4);   /* mV -> 1e-4 V */
    sb_s(&b, m->coupling_dc ? "\n# coupling=DC" : "\n# coupling=AC");
    sb_s(&b, m->probe_x10 ? "\n# probe=10" : "\n# probe=1");
    sb_s(&b, "\n# trigger=");
    sb_s(&b, modes[m->trig_mode < 3 ? m->trig_mode : 0]);
    sb_s(&b, m->edge ? ",falling," : ",rising,");
    sb_fx(&b, rdiv((int64_t)(m->trig_level_code - m->zero_code) * uvpc, 100), 4);
    sb_s(&b, "\n# trigger_index=");
    sb_fx(&b, m->trig_index, 0);
    sb_s(&b, "\n# cal=offset_pwm=");
    sb_fx(&b, m->offset_pwm, 0);
    sb_s(&b, ",volts_per_code=");
    sb_fx(&b, uvpc, 6);
    sb_s(&b, "\ntime_s,volts,adc_code\n");
    return b.err ? -1 : b.n;
}

int csv_row(char *buf, const csv_meta_t *m, int32_t i, uint8_t code) {
    int k = put_fixed(buf, rdiv((int64_t)(i - m->trig_index) * 1000000000, (int64_t)m->rate_hz), 9);
    buf[k++] = ',';
    k += put_fixed(buf + k, rdiv((int64_t)((int32_t)code - m->zero_code) * uv_per_code(m), 100), 4);
    buf[k++] = ',';
    k += put_fixed(buf + k, code, 0);
    buf[k++] = '\n';
    return k;
}
```

- [ ] **Step 4: Run the tests and confirm they pass**

Run: `make -C tests`
Expected: `… 0 failed`.

- [ ] **Step 5: Confirm the firmware still builds** (csv.c is in `fw/core`, so every app now compiles it. It needs 64-bit division from `-lgcc`, which is already linked.)

Run: `make -C fw everything 2>&1 | grep -ciE "warning|error"`
Expected: `0`

- [ ] **Step 6: Commit**

```bash
git add fw/core/csv.h fw/core/csv.c tests/test_csv.c tests/main.c
git commit -m "core: CSV header/row formatter (integer math, spec format)

Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp"
```

---

### Task 3: SPI flash erase/program (guarded), RAM fake, on-device `flashtest`

**Files:**
- Modify: `fw/common/spiflash.h`, `fw/common/spiflash.c`
- Create: `tests/fake_spiflash.h`, `tests/fake_spiflash.c` (used by Task 4; compiled into the test binary now)
- Create: `fw/apps/flashtest/main.c`

**Interfaces:**
- Consumes: `flashmap_writable()`, `FLASH_SECTOR`, `USER_VOL_BYTES`, `SYS_VOL_BASE` (Task 1). The existing `spiflash_init`, `spiflash_jedec_id` and `spiflash_read`.
- Produces: `SPIFLASH_PAGE`, `int spiflash_erase_sector(uint32_t addr)`, `int spiflash_program(uint32_t addr, const uint8_t *buf, uint32_t n)` and `int spiflash_write_sector(uint32_t addr, const uint8_t *buf)`. Return codes: 0 = ok, -1 = timeout, -2 = refused by the guard, -3 = verify mismatch. The fake adds `uint8_t *fake_flash_mem(void)`, `void fake_flash_reset(uint8_t fill)`, `int fake_flash_refused(void)` and `void fake_flash_fail_writes(int on)`.

- [ ] **Step 1: Extend the header**

Replace `fw/common/spiflash.h` with:
```c
#pragma once
#include <stdint.h>
/* External SPI NOR flash (Puya 8 MiB, JEDEC 0x856017) on SPI3 (remapped: SCK PC10, MISO PC11,
   MOSI PC12), CS = PD12. Erase/program are refused outside the user volume (flashmap.h).
   Never calls delay_ms() after init (delay_ms runs the USB idle hook). */
#define SPIFLASH_PAGE 256u

void spiflash_init(void);
uint32_t spiflash_jedec_id(void);              /* manufacturer << 16 | type << 8 | capacity */
/* Read n bytes from a 24-bit address. 0 ok, -1 SPI timeout. */
int spiflash_read(uint32_t addr, uint8_t *buf, uint32_t n);
/* Erase the 4 KiB sector at addr (must be 4096-aligned). 0 ok, -1 timeout, -2 refused. */
int spiflash_erase_sector(uint32_t addr);
/* Program n (1..256) bytes inside one 256-byte page. 0 ok, -1 timeout, -2 refused. */
int spiflash_program(uint32_t addr, const uint8_t *buf, uint32_t n);
/* Erase + program + read-back verify one 4 KiB sector (addr 4096-aligned).
   0 ok, -1 timeout, -2 refused, -3 verify mismatch. */
int spiflash_write_sector(uint32_t addr, const uint8_t *buf);
```

- [ ] **Step 2: Implement it in the driver**

In `fw/common/spiflash.c`, add `#include "flashmap.h"` after the existing includes and append:
```c
#define CMD_WREN 0x06
#define CMD_RDSR 0x05
#define CMD_SE   0x20
#define CMD_PP   0x02

/* Poll WIP (status bit 0) until clear or ms elapsed. Uses millis(), never delay_ms(). */
static int wait_ready(uint32_t ms) {
    uint32_t t0 = millis();
    for (;;) {
        cs(0); xfer(CMD_RDSR); uint8_t sr = xfer(0); cs(1);
        if (spi_err) return -1;
        if (!(sr & 1u)) return 0;
        if (millis() - t0 > ms) return -1;
        WDT_CMD = 0xAAAA;
    }
}

static void write_enable(void) { cs(0); xfer(CMD_WREN); cs(1); }

int spiflash_erase_sector(uint32_t addr) {
    if ((addr & (FLASH_SECTOR - 1)) || !flashmap_writable(addr, FLASH_SECTOR)) return -2;
    spi_err = 0;
    write_enable();
    cs(0); xfer(CMD_SE); xfer((uint8_t)(addr >> 16)); xfer((uint8_t)(addr >> 8)); xfer((uint8_t)addr); cs(1);
    return spi_err ? -1 : wait_ready(500);
}

int spiflash_program(uint32_t addr, const uint8_t *buf, uint32_t n) {
    if (n == 0 || n > SPIFLASH_PAGE || (addr & (SPIFLASH_PAGE - 1)) + n > SPIFLASH_PAGE) return -2;
    if (!flashmap_writable(addr, n)) return -2;
    spi_err = 0;
    write_enable();
    cs(0); xfer(CMD_PP); xfer((uint8_t)(addr >> 16)); xfer((uint8_t)(addr >> 8)); xfer((uint8_t)addr);
    for (uint32_t i = 0; i < n && !spi_err; i++) xfer(buf[i]);
    cs(1);
    return spi_err ? -1 : wait_ready(20);
}

int spiflash_write_sector(uint32_t addr, const uint8_t *buf) {
    static uint8_t chk[SPIFLASH_PAGE];
    int r = spiflash_erase_sector(addr);
    if (r) return r;
    for (uint32_t o = 0; o < FLASH_SECTOR; o += SPIFLASH_PAGE) {
        r = spiflash_program(addr + o, buf + o, SPIFLASH_PAGE);
        if (r) return r;
    }
    for (uint32_t o = 0; o < FLASH_SECTOR; o += SPIFLASH_PAGE) {
        if (spiflash_read(addr + o, chk, SPIFLASH_PAGE)) return -1;
        for (uint32_t i = 0; i < SPIFLASH_PAGE; i++) if (chk[i] != buf[o + i]) return -3;
    }
    return 0;
}
```
Also update the file's first comment to say it is no longer read-only.

- [ ] **Step 3: Write the RAM fake for the host tests**

`tests/fake_spiflash.h`:
```c
#pragma once
#include <stdint.h>
#include "spiflash.h"
/* Host test model of the 8 MiB NOR: erase sets 0xFF, program ANDs bits (NOR semantics),
   the same flashmap guard as the driver; refused calls are counted. */
uint8_t *fake_flash_mem(void);
void fake_flash_reset(uint8_t fill);
int fake_flash_refused(void);
void fake_flash_fail_writes(int on);       /* 1: every erase returns -1 (I/O error injection) */
```

`tests/fake_spiflash.c`:
```c
#include "fake_spiflash.h"
#include "flashmap.h"
#include <string.h>

#define FAKE_SIZE (8u * 1024 * 1024)
static uint8_t mem[FAKE_SIZE];
static int refused, fail;

uint8_t *fake_flash_mem(void) { return mem; }
void fake_flash_reset(uint8_t fill) { memset(mem, fill, sizeof mem); refused = 0; fail = 0; }
int fake_flash_refused(void) { return refused; }
void fake_flash_fail_writes(int on) { fail = on; }

void spiflash_init(void) { }
uint32_t spiflash_jedec_id(void) { return 0x856017u; }

int spiflash_read(uint32_t addr, uint8_t *buf, uint32_t n) {
    if (addr >= FAKE_SIZE || n > FAKE_SIZE - addr) return -1;
    memcpy(buf, mem + addr, n);
    return 0;
}

int spiflash_erase_sector(uint32_t addr) {
    if ((addr & (FLASH_SECTOR - 1)) || !flashmap_writable(addr, FLASH_SECTOR)) { refused++; return -2; }
    if (fail) return -1;
    memset(mem + addr, 0xFF, FLASH_SECTOR);
    return 0;
}

int spiflash_program(uint32_t addr, const uint8_t *buf, uint32_t n) {
    if (n == 0 || n > SPIFLASH_PAGE || (addr & (SPIFLASH_PAGE - 1)) + n > SPIFLASH_PAGE ||
        !flashmap_writable(addr, n)) { refused++; return -2; }
    for (uint32_t i = 0; i < n; i++) mem[addr + i] &= buf[i];
    return 0;
}

int spiflash_write_sector(uint32_t addr, const uint8_t *buf) {
    int r = spiflash_erase_sector(addr);
    if (r) return r;
    for (uint32_t o = 0; o < FLASH_SECTOR; o += SPIFLASH_PAGE)
        if ((r = spiflash_program(addr + o, buf + o, SPIFLASH_PAGE))) return r;
    return memcmp(mem + addr, buf, FLASH_SECTOR) ? -3 : 0;
}
```
The test Makefile compiles every `tests/*.c`, so it needs `-I../fw/common`. Task 4 rewrites the Makefile. For now, change only the `CFLAGS` line in `tests/Makefile` to append ` -I../fw/common`. Only `spiflash.h` from that directory is ever included on the host.

Add a small test of the fake's guard, so the fake itself is checked. Put it at the end of `tests/test_flashmap.c`:
```c
#include "fake_spiflash.h"
void test_fake_flash(void) {
    static uint8_t s[4096];
    fake_flash_reset(0xA5);
    memset(s, 0x3C, sizeof s);
    CHECK_EQ(spiflash_write_sector(0x4FF000, s), 0);
    CHECK_EQ(fake_flash_mem()[0x4FF000], 0x3C);
    CHECK_EQ(spiflash_write_sector(0x500000, s), -2);         /* system volume refused */
    CHECK_EQ(spiflash_erase_sector(0x500000), -2);
    CHECK_EQ(spiflash_program(0x500000, s, 16), -2);
    CHECK_EQ(spiflash_program(0x0000F0, s, 32), -2);           /* crosses a page */
    CHECK_EQ(fake_flash_mem()[0x500000], 0xA5);                /* untouched */
    CHECK_EQ(fake_flash_refused(), 4);
}
```
Move the `#include "fake_spiflash.h"` to the top of the file, next to the other includes. Register `test_fake_flash` in `tests/main.c`.

- [ ] **Step 4: Run the tests**

Run: `make -C tests`
Expected: `… 0 failed`.

- [ ] **Step 5: Write the on-device test app**

`fw/apps/flashtest/main.c`:
```c
// flashtest: guarded SPI NOR write/erase on the LAST user-volume sector, with backup + restore.
// The system volume (0x500000+) must stay byte-identical and the guard must refuse it.
#include "board.h"
#include "lcd.h"
#include "spiflash.h"
#include "flashmap.h"

#define TEST_ADDR (USER_VOL_BYTES - FLASH_SECTOR)   /* 0x4FF000 */
static uint8_t backup[FLASH_SECTOR], pat[FLASH_SECTOR], rd[FLASH_SECTOR], sys0[256], sys1[256];

static int row = 1;
static void line(const char *what, int ok, uint32_t val) {
    char b[12];
    int c = lcd_text(0, row, what, C_WHITE, C_BLACK);
    c = lcd_text(c, row, ok ? " PASS " : " FAIL ", ok ? C_GREEN : C_RED, C_BLACK);
    lcd_text(c, row, fmt_x(b, val, 8), C_GRAY, C_BLACK);
    row++;
}

int main(void) {
    board_init();
    keys_init();
    lcd_init();
    lcd_fill(0, 0, LCD_W, LCD_H, C_BLACK);
    backlight(1);
    lcd_text(0, 0, " flashtest 0x4FF000 (backup+restore)", C_CYAN, C_BLACK);
    spiflash_init();
    line(" jedec      ", spiflash_jedec_id() == 0x856017u, spiflash_jedec_id());
    int ok = spiflash_read(TEST_ADDR, backup, FLASH_SECTOR) == 0;
    line(" backup read", ok, 0);
    spiflash_read(SYS_VOL_BASE, sys0, sizeof sys0);
    if (ok) {
        for (uint32_t i = 0; i < FLASH_SECTOR; i++) pat[i] = (uint8_t)(i * 7 + 3);
        uint32_t t0 = millis();
        int r = spiflash_write_sector(TEST_ADDR, pat);
        uint32_t dt = millis() - t0;
        line(" write ms   ", r == 0, dt);
        spiflash_read(TEST_ADDR, rd, FLASH_SECTOR);
        int same = 1;
        for (uint32_t i = 0; i < FLASH_SECTOR; i++) if (rd[i] != pat[i]) same = 0;
        line(" readback   ", same, 0);
        line(" guard sys  ", spiflash_write_sector(SYS_VOL_BASE, pat) == -2, 0);
        line(" guard erase", spiflash_erase_sector(SYS_VOL_BASE) == -2, 0);
        r = spiflash_write_sector(TEST_ADDR, backup);
        line(" restore    ", r == 0, (uint32_t)r);
        spiflash_read(SYS_VOL_BASE, sys1, sizeof sys1);
        int sys_same = 1;
        for (uint32_t i = 0; i < sizeof sys0; i++) if (sys0[i] != sys1[i]) sys_same = 0;
        line(" sys intact ", sys_same, 0);
    }
    lcd_text(0, 14, " Photo this. Hold POWER=off", C_GRAY, C_BLACK);
    for (;;) {
        if (keys_read() & KEY_BIT(KEY_POWER)) {
            uint32_t t0 = millis();
            while (keys_read() & KEY_BIT(KEY_POWER))
                if (millis() - t0 > 1000) { backlight(0); power_off(); for (;;) { } }
        }
        WDT_CMD = 0xAAAA;
        delay_ms(20);
    }
}
```

- [ ] **Step 6: Build**

Run: `make -C fw everything 2>&1 | grep -ciE "warning|error"`
Expected: `0`, and `fw/build/APP_DSO-510_flashtest.bin` exists.

- [ ] **Step 7: Commit**

```bash
git add fw/common/spiflash.h fw/common/spiflash.c tests/fake_spiflash.h tests/fake_spiflash.c tests/test_flashmap.c tests/main.c tests/Makefile fw/apps/flashtest/main.c
git commit -m "spiflash: guarded sector erase/page program/verified sector write; RAM fake; flashtest app

Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp"
```

- [ ] **Step 8 (controller with the user, on device):** flash `APP_DSO-510_flashtest.bin`. Every line must read PASS. Record the write time in ms in the ledger.

---

### Task 4: FatFs, volume block layer, CSV save (host-tested on a fake flash)

**Files:**
- Create: `fw/third_party/fatfs/ff.c`, `ff.h`, `diskio.h`, `ffconf.h`, `LICENSE.txt`, `00readme.txt` (from FatFs R0.15a)
- Create: `fw/storage/vol.h`, `fw/storage/vol.c`, `fw/storage/diskio_flash.c`, `fw/storage/capsave.h`, `fw/storage/capsave.c`
- Create: `tests/test_capsave.c`
- Modify: `fw/common/startup.c` (add `memcmp`, `strchr`, `strlen`), `fw/Makefile`, `tests/Makefile`, `tests/main.c`

**Interfaces:**
- Consumes: the `spiflash_*` API (Task 3), `flashmap.h` (Task 1), `capname_*` and `CAP_DIR` (Task 1), `csv_header`, `csv_row`, `csv_meta_t` and `CSV_ROW_MAX` (Task 2).
- Produces:
  - `int vol_read(uint32_t lba, uint8_t *buf)` and `int vol_write(uint32_t lba, const uint8_t *buf)`. Blocks are 4096 bytes, LBAs run from 0 to 1279, and both return 0 on success and <0 on error.
  - `typedef enum { CAP_OK = 0, CAP_FULL = 1, CAP_ERR = 2 } cap_res_t;`
  - `cap_res_t capsave_write(const uint8_t *rec, int n, const csv_meta_t *m, int *num_out);`

- [ ] **Step 1: Vendor FatFs R0.15a**

```bash
mkdir -p /tmp/ffdl && cd /tmp/ffdl
curl -sSL -o ff15a.zip https://elm-chan.org/fsw/ff/arc/ff15a.zip
echo "74737b1cafa1a67a3f722dd0d1a44767c5b54d37b6300ad3825a904cbe88fc3c  ff15a.zip" | sha256sum -c
unzip -o -q ff15a.zip -d ff
cd - && mkdir -p fw/third_party/fatfs
cp /tmp/ffdl/ff/source/ff.c /tmp/ffdl/ff/source/ff.h /tmp/ffdl/ff/source/diskio.h \
   /tmp/ffdl/ff/source/ffconf.h /tmp/ffdl/ff/source/00readme.txt fw/third_party/fatfs/
cp /tmp/ffdl/ff/LICENSE.txt fw/third_party/fatfs/
```
Expected: `ff15a.zip: OK`. If the checksum fails, stop and report BLOCKED.

- [ ] **Step 2: Configure `ffconf.h`** (this is the only file of FatFs we change)

```bash
cd fw/third_party/fatfs
sed -i -e 's/^#define FF_CODE_PAGE\t932/#define FF_CODE_PAGE\t437/' \
       -e 's/^#define FF_MAX_SS\t\t512/#define FF_MAX_SS\t\t4096/' \
       -e 's/^#define FF_MIN_SS\t\t512/#define FF_MIN_SS\t\t4096/' \
       -e 's/^#define FF_FS_TINY\t\t0/#define FF_FS_TINY\t\t1/' \
       -e 's/^#define FF_FS_NORTC\t\t0/#define FF_FS_NORTC\t\t1/' \
       -e 's/^#define FF_NORTC_MDAY\t1/#define FF_NORTC_MDAY\t13/' \
       -e 's/^#define FF_NORTC_YEAR\t2024/#define FF_NORTC_YEAR\t2026/' \
       -e 's/^#define FF_USE_MKFS\t\t0/#ifndef FF_USE_MKFS\n#define FF_USE_MKFS\t\t0\t\/* host tests build with -DFF_USE_MKFS=1 *\/\n#endif/' ffconf.h
grep -nE "^#define FF_(CODE_PAGE|MAX_SS|MIN_SS|FS_TINY|FS_NORTC|NORTC_MON|NORTC_MDAY|NORTC_YEAR|USE_MKFS|USE_LFN|VOLUMES|FS_READONLY)\b" ffconf.h
cd -
```
Expected `grep` output. The values must be: CODE_PAGE 437, MAX_SS 4096, MIN_SS 4096, FS_TINY 1, FS_NORTC 1, NORTC_MON 11, NORTC_MDAY 13, NORTC_YEAR 2026, USE_MKFS 0 inside `#ifndef`, USE_LFN 0, VOLUMES 1, FS_READONLY 0. If a `sed` pattern did not match because of whitespace, fix that line by hand to those values.

- [ ] **Step 3: Add the libc functions FatFs needs**

In `fw/common/startup.c`, after the existing `memcpy`, add:
```c
int memcmp(const void *a, const void *b, size_t n) {
    const unsigned char *p = a, *q = b;
    for (; n; n--, p++, q++) if (*p != *q) return *p - *q;
    return 0;
}
char *strchr(const char *s, int c) {
    for (;; s++) { if (*s == (char)c) return (char *)s; if (!*s) return 0; }
}
size_t strlen(const char *s) { const char *p = s; while (*p) p++; return (size_t)(p - s); }
```

- [ ] **Step 4: Write the volume layer and the FatFs disk I/O**

`fw/storage/vol.h`:
```c
#pragma once
#include <stdint.h>
/* The user volume as 4096-byte blocks 0..USER_VOL_SECTORS-1 at flash offset USER_VOL_BASE.
   Shared by FatFs (diskio_flash.c) and the USB drive. 0 ok, <0 error. */
int vol_read(uint32_t lba, uint8_t *buf);
int vol_write(uint32_t lba, const uint8_t *buf);
```

`fw/storage/vol.c`:
```c
#include "vol.h"
#include "flashmap.h"
#include "spiflash.h"

int vol_read(uint32_t lba, uint8_t *buf) {
    if (lba >= USER_VOL_SECTORS) return -2;
    return spiflash_read(USER_VOL_BASE + lba * FLASH_SECTOR, buf, FLASH_SECTOR);
}

int vol_write(uint32_t lba, const uint8_t *buf) {
    if (lba >= USER_VOL_SECTORS) return -2;
    return spiflash_write_sector(USER_VOL_BASE + lba * FLASH_SECTOR, buf);
}
```

`fw/storage/diskio_flash.c`:
```c
/* FatFs media access for volume 0 = the user volume (4096-byte sectors). */
#include "ff.h"
#include "diskio.h"
#include "vol.h"
#include "flashmap.h"

DSTATUS disk_initialize(BYTE pdrv) { return pdrv ? STA_NOINIT : 0; }
DSTATUS disk_status(BYTE pdrv) { return pdrv ? STA_NOINIT : 0; }

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count) {
    if (pdrv) return RES_PARERR;
    for (UINT i = 0; i < count; i++)
        if (vol_read((uint32_t)sector + i, buff + i * FLASH_SECTOR)) return RES_ERROR;
    return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count) {
    if (pdrv) return RES_PARERR;
    for (UINT i = 0; i < count; i++)
        if (vol_write((uint32_t)sector + i, buff + i * FLASH_SECTOR)) return RES_ERROR;
    return RES_OK;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff) {
    if (pdrv) return RES_PARERR;
    switch (cmd) {
    case CTRL_SYNC: return RES_OK;
    case GET_SECTOR_COUNT: *(LBA_t *)buff = USER_VOL_SECTORS; return RES_OK;
    case GET_BLOCK_SIZE: *(DWORD *)buff = 1; return RES_OK;
    default: return RES_PARERR;
    }
}
```

- [ ] **Step 5: Write the failing save test**

`tests/test_capsave.c`:
```c
#include "test.h"
#include "fake_spiflash.h"
#include "flashmap.h"
#include "capsave.h"
#include "ff.h"
#include <stdlib.h>

static BYTE work[FF_MAX_SS];
static uint8_t rec[200000];

static void format(void) {
    fake_flash_reset(0xFF);
    memset(fake_flash_mem() + SYS_VOL_BASE, 0xA5, 0x300000);   /* system volume sentinel */
    MKFS_PARM opt = {FM_FAT, 2, 0, 512, 4096};
    FATFS fs;
    CHECK_EQ(f_mkfs("", &opt, work, sizeof work), FR_OK);
    CHECK_EQ(f_mount(&fs, "", 1), FR_OK);
    UINT bw; FIL f;
    CHECK_EQ(f_mkdir("/LOGO"), FR_OK);                          /* like the stock drive */
    CHECK_EQ(f_open(&f, "/KEEP.TXT", FA_WRITE | FA_CREATE_NEW), FR_OK);
    CHECK_EQ(f_write(&f, "stock", 5, &bw), FR_OK);
    f_close(&f);
    f_mount(0, "", 0);
}

static char *read_file(const char *path, UINT *len) {
    static char buf[300000];
    FATFS fs; FIL f;
    *len = 0;
    if (f_mount(&fs, "", 1) != FR_OK) return 0;
    if (f_open(&f, path, FA_READ) != FR_OK) { f_mount(0, "", 0); return 0; }
    f_read(&f, buf, sizeof buf - 1, len);
    buf[*len] = 0;
    f_close(&f);
    f_mount(0, "", 0);
    return buf;
}

void test_capsave(void) {
    csv_meta_t m;
    memset(&m, 0, sizeof m);
    m.rate_hz = 1000000; m.vdiv_mv = 1000; m.coupling_dc = 1; m.zero_code = 128;
    m.trig_level_code = 153; m.trig_index = 2; m.offset_pwm = 2033;
    for (int i = 0; i < 4800; i++) rec[i] = (uint8_t)(i * 13);

    format();
    int num = 0;
    CHECK_EQ(capsave_write(rec, 4800, &m, &num), CAP_OK);
    CHECK_EQ(num, 1);
    CHECK_EQ(capsave_write(rec, 5, &m, &num), CAP_OK);
    CHECK_EQ(num, 2);

    /* expected text of the 5-sample file */
    static char exp[2000];
    int k = csv_header(exp, sizeof exp, &m);
    for (int i = 0; i < 5; i++) k += csv_row(exp + k, &m, i, rec[i]);
    exp[k] = 0;
    UINT len;
    char *got = read_file("/CSV/CAP0002.CSV", &len);
    CHECK(got != 0);
    if (got) CHECK_STR(got, exp);
    got = read_file("/CSV/CAP0001.CSV", &len);
    CHECK(got != 0 && len > 4800 * 20);                         /* ~25 bytes per row */
    got = read_file("/KEEP.TXT", &len);
    CHECK(got != 0 && len == 5);                                /* other files untouched */

    /* numbering continues after the highest existing number */
    FATFS fs; FIL f;
    f_mount(&fs, "", 1);
    f_open(&f, "/CSV/CAP0041.CSV", FA_WRITE | FA_CREATE_NEW); f_close(&f);
    f_mount(0, "", 0);
    CHECK_EQ(capsave_write(rec, 5, &m, &num), CAP_OK);
    CHECK_EQ(num, 42);

    /* drive full: partial file is removed, CAP_FULL returned */
    for (int i = 0; i < (int)sizeof rec; i++) rec[i] = (uint8_t)i;
    int full = 0;
    for (int t = 0; t < 10 && !full; t++) full = capsave_write(rec, sizeof rec, &m, &num) == CAP_FULL;
    CHECK(full);
    f_mount(&fs, "", 1);
    char path[24] = "/CSV/";
    capname_make(path + 5, num);
    FILINFO fi;
    CHECK(f_stat(path, &fi) == FR_NO_FILE);                     /* partial file deleted */
    f_mount(0, "", 0);

    CHECK_EQ(fake_flash_refused(), 0);                          /* no write ever hit the guard */

    /* I/O error */
    format();
    fake_flash_fail_writes(1);
    CHECK_EQ(capsave_write(rec, 5, &m, &num), CAP_ERR);
    fake_flash_fail_writes(0);

    /* the system volume was never written */
    const uint8_t *sys = fake_flash_mem() + SYS_VOL_BASE;
    int intact = 1;
    for (uint32_t i = 0; i < 0x300000; i++) if (sys[i] != 0xA5) { intact = 0; break; }
    CHECK(intact);
    CHECK_EQ(fake_flash_refused(), 0);
}
```
Register `test_capsave` in `tests/main.c`.

- [ ] **Step 6: Rewrite `tests/Makefile`** (FatFs is compiled separately with warnings off; everything else stays `-Werror`)

```make
CC     ?= cc
INC     = -I../fw/core -I../fw/common -I../fw/storage -I../fw/third_party/fatfs
CFLAGS  = -std=c99 -D_DEFAULT_SOURCE -Wall -Wextra -Werror -O1 -g $(INC) -DFF_USE_MKFS=1
SRCS    = $(wildcard *.c) $(wildcard ../fw/core/*.c) $(wildcard ../fw/storage/*.c)
HDRS    = $(wildcard *.h) $(wildcard ../fw/core/*.h) $(wildcard ../fw/storage/*.h) ../fw/common/spiflash.h

run: build/tests
	./build/tests

test: run

build/ff.o: ../fw/third_party/fatfs/ff.c ../fw/third_party/fatfs/ffconf.h | build
	$(CC) -std=c99 -O1 -g -w $(INC) -DFF_USE_MKFS=1 -c $< -o $@

build/tests: $(SRCS) $(HDRS) build/ff.o | build
	$(CC) $(CFLAGS) $(SRCS) build/ff.o -lm -o $@

build:
	mkdir -p build

clean:
	rm -rf build

.PHONY: run test clean
```

- [ ] **Step 7: Run the tests and confirm they fail**

Run: `make -C tests`
Expected: compile error, `capsave.h: No such file or directory`.

- [ ] **Step 8: Implement `capsave`**

`fw/storage/capsave.h`:
```c
#pragma once
#include <stdint.h>
#include "csv.h"
#include "capname.h"

typedef enum { CAP_OK = 0, CAP_FULL = 1, CAP_ERR = 2 } cap_res_t;

/* Mount volume 0, write rec[0..n) as CAP_DIR/CAPnnnn.CSV (next free number, folder created if
   missing), unmount. *num_out = number used (also on CAP_FULL, for the deleted partial file).
   CAP_FULL: drive or numbers exhausted (partial file deleted). CAP_ERR: any other failure. */
cap_res_t capsave_write(const uint8_t *rec, int n, const csv_meta_t *m, int *num_out);
```

`fw/storage/capsave.c`:
```c
#include "capsave.h"
#include "ff.h"

static FATFS fs;
static FIL fil;
static DIR dir;
static FILINFO fno;
static char buf[4096];

static int highest_number(void) {
    int hi = 0;
    if (f_opendir(&dir, CAP_DIR) != FR_OK) return 0;
    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0]) {
        int k = capname_number(fno.fname);
        if (k > hi) hi = k;
    }
    f_closedir(&dir);
    return hi;
}

/* Write len bytes; CAP_FULL if the volume ran out of space. */
static cap_res_t put(const char *p, UINT len) {
    UINT bw;
    if (f_write(&fil, p, len, &bw) != FR_OK) return CAP_ERR;
    return bw == len ? CAP_OK : CAP_FULL;
}

cap_res_t capsave_write(const uint8_t *rec, int n, const csv_meta_t *m, int *num_out) {
    char path[5 + 13] = CAP_DIR "/";
    cap_res_t res = CAP_ERR;
    *num_out = 0;
    if (f_mount(&fs, "", 1) != FR_OK) return CAP_ERR;
    FRESULT fr = f_mkdir(CAP_DIR);
    if (fr != FR_OK && fr != FR_EXIST) goto out;
    int num = capname_next(highest_number());
    if (num < 0) { res = CAP_FULL; goto out; }
    *num_out = num;
    capname_make(path + sizeof CAP_DIR, num);           /* after "/CSV/" */
    if (f_open(&fil, path, FA_WRITE | FA_CREATE_NEW) != FR_OK) goto out;
    int k = csv_header(buf, sizeof buf, m);
    res = k < 0 ? CAP_ERR : CAP_OK;
    for (int i = 0; i < n && res == CAP_OK; i++) {
        k += csv_row(buf + k, m, i, rec[i]);
        if (k > (int)sizeof buf - CSV_ROW_MAX) { res = put(buf, (UINT)k); k = 0; }
    }
    if (res == CAP_OK && k) res = put(buf, (UINT)k);
    if (f_close(&fil) != FR_OK && res == CAP_OK) res = CAP_ERR;
    if (res != CAP_OK) f_unlink(path);                   /* never leave a partial file */
out:
    f_mount(0, "", 0);
    return res;
}
```
`sizeof CAP_DIR` is 5 (`"/CSV"` plus its NUL), so `path + 5` lands right after `"/CSV/"`.

- [ ] **Step 9: Run the tests and confirm they pass**

Run: `make -C tests`
Expected: `… 0 failed`. If `f_mkfs` fails to link because `ff_memalloc` is missing, the work buffer is not being used. Check the `f_mkfs(..., work, sizeof work)` call, not `ffsystem.c`.

- [ ] **Step 10: Change the firmware build** (FatFs is compiled once into `build/ff.o` with warnings off, because it is vendored code. Our code keeps the zero-warning rule.)

Replace `fw/Makefile` with:
```make
# Build:  make APP=<name>     name = app (the firmware) or a test app in apps/<name>
# Output: build/APP_DSO-510_<APP>.bin  -> copy onto the scope's IAP drive (tools/flash.sh)
APP     ?= app
CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
ARCH    = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS  = $(ARCH) -O2 -g -Wall -Wextra -ffreestanding -ffunction-sections -fdata-sections \
          -fno-tree-loop-distribute-patterns -Icommon -Icore -Iapp -Istorage -Ithird_party/fatfs
CFLAGS += $(EXTRA)
LDFLAGS = -nostdlib -T common/link.ld -Wl,--gc-sections -Wl,-Map=build/$(APP).map

COMMON  = $(wildcard common/*.c)
CORE    = $(wildcard core/*.c)
STORAGE = $(wildcard storage/*.c)
FF_OBJ  = build/ff.o
ifeq ($(APP),hello)
SRCS = apps/hello/main.c common/startup.c
OBJS =
else ifeq ($(APP),app)
SRCS = $(wildcard app/*.c) $(COMMON) $(CORE) $(STORAGE)
OBJS = $(FF_OBJ)
else
SRCS = apps/$(APP)/main.c $(COMMON) $(CORE) $(STORAGE)
OBJS = $(FF_OBJ)
endif

ELF = build/$(APP).elf
BIN = build/APP_DSO-510_$(APP).bin
APPS = $(notdir $(wildcard apps/*)) $(if $(wildcard app/main.c),app)
HDRS = $(wildcard common/*.h core/*.h app/*.h storage/*.h third_party/fatfs/*.h)

all: $(BIN)

$(FF_OBJ): third_party/fatfs/ff.c third_party/fatfs/ffconf.h third_party/fatfs/ff.h | build
	$(CC) $(CFLAGS) -w -c $< -o $@

$(ELF): $(SRCS) $(OBJS) $(HDRS) common/link.ld | build
	$(CC) $(CFLAGS) $(LDFLAGS) $(SRCS) $(OBJS) -lgcc -o $@

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@
	arm-none-eabi-size $<

build:
	mkdir -p build

everything:
	for a in $(APPS); do $(MAKE) --no-print-directory APP=$$a || exit 1; done

clean:
	rm -rf build

.PHONY: all clean everything
```

Run: `make -C fw clean everything 2>&1 | grep -ciE "warning|error"`
Expected: `0`. (`--gc-sections` drops FatFs from apps that never call it.)

- [ ] **Step 11: Commit**

```bash
git add fw/third_party/fatfs fw/storage tests/test_capsave.c tests/Makefile tests/main.c fw/common/startup.c fw/Makefile
git commit -m "storage: vendored FatFs R0.15a, volume block layer, CSV save (host-tested on fake NOR)

Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp"
```

---

### Task 5: `msc` — USB Mass Storage Bulk-Only Transport + SCSI (pure)

**Files:**
- Create: `fw/core/msc.h`, `fw/core/msc.c`, `tests/test_msc.c`
- Modify: `tests/main.c`

**Interfaces:**
- Produces: `MSC_BLOCK`, `MSC_PKT`, `msc_disk_t`, `msc_t`, and the functions `msc_init`, `msc_reset`, `msc_rx`, `msc_tx`, `msc_task`, `msc_set_present`, `msc_busy`. Their signatures are below.

Behaviour:
- **States:** IDLE (waiting for a CBW) → command → data phase (IN, OUT, zero-fill IN, or discard OUT) → STATUS (CSW) → IDLE.
- **Failure:** a failed command never stalls. If the host expects data-in, it gets `dCBWDataTransferLength` zero bytes. If it sends data-out, the data is discarded. The CSW then has status 1 and residue = Hi, and the sense data explains the failure. This is legal BOT behaviour (fill data) and it avoids endpoint-halt handling.
- **Block I/O** never runs inside `msc_rx` or `msc_tx`. Those set `need_io`, and `msc_task()` does the read or write in main context.
- **Medium:** `present = 0` gives sense 02/3A/00 to every medium command. The 0 → 1 transition queues one UNIT ATTENTION, 06/28/00.

- [ ] **Step 1: Write the failing test**

`tests/test_msc.c`:
```c
#include "test.h"
#include "msc.h"

#define NBLK 8
static uint8_t disk[NBLK][MSC_BLOCK];
static int fail_write;
static int rd(uint32_t lba, uint8_t *b) { memcpy(b, disk[lba], MSC_BLOCK); return 0; }
static int wr(uint32_t lba, const uint8_t *b) { if (fail_write) return -1; memcpy(disk[lba], b, MSC_BLOCK); return 0; }
static const msc_disk_t D = {rd, wr, NBLK};
static msc_t M;
static uint8_t in[70000];
static int in_len, csw_status;
static uint32_t csw_residue, csw_tag;

static void cbw(uint32_t tag, uint32_t hi, int dir_in, const uint8_t *cb, int cblen) {
    uint8_t p[31] = {0x55, 0x53, 0x42, 0x43};
    p[4] = (uint8_t)tag; p[5] = (uint8_t)(tag >> 8); p[6] = (uint8_t)(tag >> 16); p[7] = (uint8_t)(tag >> 24);
    p[8] = (uint8_t)hi; p[9] = (uint8_t)(hi >> 8); p[10] = (uint8_t)(hi >> 16); p[11] = (uint8_t)(hi >> 24);
    p[12] = dir_in ? 0x80 : 0; p[13] = 0; p[14] = (uint8_t)cblen;
    memcpy(p + 15, cb, (size_t)cblen);
    CHECK_EQ(msc_rx(&M, p, 31, 0), 0);
}

/* Pump IN packets (running msc_task) until the CSW arrives; data bytes go to in[]. */
static void run_in(void) {
    uint8_t pkt[64];
    in_len = 0; csw_status = -1;
    for (int guard = 0; guard < 5000; guard++) {
        msc_task(&M);
        int n = msc_tx(&M, pkt);
        if (n < 0) continue;
        if (n == 13 && pkt[0] == 0x55 && pkt[1] == 0x53 && pkt[2] == 0x42 && pkt[3] == 0x53) {
            csw_tag = pkt[4] | pkt[5] << 8 | pkt[6] << 16 | (uint32_t)pkt[7] << 24;
            csw_residue = pkt[8] | pkt[9] << 8 | pkt[10] << 16 | (uint32_t)pkt[11] << 24;
            csw_status = pkt[12];
            return;
        }
        memcpy(in + in_len, pkt, (size_t)n); in_len += n;
    }
}

/* Send hi bytes of data-out from src in 64-byte packets, running msc_task when held off. */
static void run_out(const uint8_t *src, int hi, uint32_t now) {
    for (int o = 0; o < hi; ) {
        int n = hi - o < 64 ? hi - o : 64;
        if (msc_rx(&M, src + o, n, now) == 0) o += n; else msc_task(&M);
    }
}

static void sense(int *key, int *asc, int *ascq) {
    uint8_t cb[6] = {0x03, 0, 0, 0, 18, 0};
    cbw(9, 18, 1, cb, 6); run_in();
    *key = in[2] & 0x0F; *asc = in[12]; *ascq = in[13];
}

void test_msc(void) {
    int k, a, q;
    msc_init(&M, &D);
    /* TEST UNIT READY */
    uint8_t tur[6] = {0};
    cbw(0x11223344, 0, 0, tur, 6); run_in();
    CHECK_EQ(csw_status, 0); CHECK_EQ(csw_tag, 0x11223344); CHECK_EQ(csw_residue, 0); CHECK_EQ(in_len, 0);
    /* INQUIRY */
    uint8_t inq[6] = {0x12, 0, 0, 0, 36, 0};
    cbw(1, 36, 1, inq, 6); run_in();
    CHECK_EQ(in_len, 36); CHECK_EQ(in[0], 0); CHECK_EQ(in[1], 0x80); CHECK_EQ(csw_status, 0);
    CHECK(memcmp(in + 8, "DSO-510 ", 8) == 0);
    /* READ CAPACITY(10) */
    uint8_t rc[10] = {0x25};
    cbw(2, 8, 1, rc, 10); run_in();
    CHECK_EQ(in_len, 8);
    CHECK_EQ(in[3], NBLK - 1); CHECK_EQ(in[6], 0x10); CHECK_EQ(in[7], 0x00);   /* 4096 */
    /* READ FORMAT CAPACITIES */
    uint8_t rfc[10] = {0x23, 0, 0, 0, 0, 0, 0, 0, 252, 0};
    cbw(3, 252, 1, rfc, 10); run_in();
    CHECK_EQ(in_len, 12); CHECK_EQ(in[3], 8); CHECK_EQ(in[7], NBLK); CHECK_EQ(in[8], 2);
    CHECK_EQ(csw_residue, 240);
    /* MODE SENSE(6): not write-protected */
    uint8_t ms[6] = {0x1A, 0, 0x3F, 0, 192, 0};
    cbw(4, 192, 1, ms, 6); run_in();
    CHECK_EQ(in_len, 4); CHECK_EQ(in[2], 0); CHECK_EQ(csw_residue, 188);
    /* WRITE(10) lba 2, 2 blocks, then READ(10) back */
    static uint8_t pat[2 * MSC_BLOCK];
    for (int i = 0; i < (int)sizeof pat; i++) pat[i] = (uint8_t)(i * 31 + 7);
    uint8_t w10[10] = {0x2A, 0, 0, 0, 0, 2, 0, 0, 2, 0};
    cbw(5, sizeof pat, 0, w10, 10);
    CHECK_EQ(msc_rx(&M, pat, 64, 1000), 0);
    run_out(pat + 64, (int)sizeof pat - 64, 1000);
    run_in();
    CHECK_EQ(csw_status, 0); CHECK_EQ(csw_residue, 0);
    CHECK(memcmp(disk[2], pat, sizeof pat) == 0);
    CHECK_EQ(msc_busy(&M, 1500), 1);                      /* wrote within the last second */
    CHECK_EQ(msc_busy(&M, 2001), 0);
    uint8_t r10[10] = {0x28, 0, 0, 0, 0, 2, 0, 0, 2, 0};
    cbw(6, sizeof pat, 1, r10, 10); run_in();
    CHECK_EQ(in_len, (int)sizeof pat); CHECK(memcmp(in, pat, sizeof pat) == 0); CHECK_EQ(csw_status, 0);
    /* READ past the end: zero fill, failed CSW, sense 05/21/00 */
    uint8_t rbad[10] = {0x28, 0, 0, 0, 0, NBLK - 1, 0, 0, 2, 0};
    cbw(7, 2 * MSC_BLOCK, 1, rbad, 10); run_in();
    CHECK_EQ(in_len, 2 * MSC_BLOCK); CHECK_EQ(in[0], 0); CHECK_EQ(csw_status, 1);
    CHECK_EQ(csw_residue, 2 * MSC_BLOCK);
    sense(&k, &a, &q); CHECK_EQ(k, 5); CHECK_EQ(a, 0x21);
    /* unknown opcode with data-in */
    uint8_t unk[12] = {0xA0};
    cbw(8, 16, 1, unk, 12); run_in();
    CHECK_EQ(in_len, 16); CHECK_EQ(csw_status, 1);
    sense(&k, &a, &q); CHECK_EQ(k, 5); CHECK_EQ(a, 0x20);
    /* medium removed: everything medium-related fails with 02/3A/00 */
    msc_set_present(&M, 0);
    cbw(10, 0, 0, tur, 6); run_in(); CHECK_EQ(csw_status, 1);
    sense(&k, &a, &q); CHECK_EQ(k, 2); CHECK_EQ(a, 0x3A); CHECK_EQ(q, 0);
    uint8_t r1[10] = {0x28, 0, 0, 0, 0, 0, 0, 0, 1, 0};
    cbw(11, MSC_BLOCK, 1, r1, 10); run_in();
    CHECK_EQ(in_len, MSC_BLOCK); CHECK_EQ(csw_status, 1);
    cbw(12, 8, 1, rc, 10); run_in(); CHECK_EQ(csw_status, 1);
    /* write while removed: data discarded, disk unchanged */
    uint8_t w1[10] = {0x2A, 0, 0, 0, 0, 0, 0, 0, 1, 0};
    uint8_t before = disk[0][0];
    static uint8_t junk[MSC_BLOCK];
    memset(junk, 0x77, sizeof junk);
    cbw(13, MSC_BLOCK, 0, w1, 10); run_out(junk, MSC_BLOCK, 3000); run_in();
    CHECK_EQ(csw_status, 1); CHECK_EQ(disk[0][0], before);
    /* re-inserted: one UNIT ATTENTION 06/28/00, then ready */
    msc_set_present(&M, 1);
    cbw(14, 0, 0, tur, 6); run_in(); CHECK_EQ(csw_status, 1);
    sense(&k, &a, &q); CHECK_EQ(k, 6); CHECK_EQ(a, 0x28);
    cbw(15, 0, 0, tur, 6); run_in(); CHECK_EQ(csw_status, 0);
    /* write error: 03/0C/00 */
    fail_write = 1;
    cbw(16, MSC_BLOCK, 0, w1, 10); run_out(junk, MSC_BLOCK, 4000); run_in();
    CHECK_EQ(csw_status, 1);
    sense(&k, &a, &q); CHECK_EQ(k, 3); CHECK_EQ(a, 0x0C);
    fail_write = 0;
    /* held off while a block write is pending (before msc_task runs) */
    cbw(17, MSC_BLOCK, 0, w1, 10);
    for (int o = 0; o < MSC_BLOCK; o += 64) CHECK_EQ(msc_rx(&M, junk + o, 64, 5000), 0);
    CHECK_EQ(msc_rx(&M, junk, 31, 5000), -1);             /* next CBW must wait */
    CHECK_EQ(msc_busy(&M, 9000), 1);                      /* command in progress */
    run_in(); CHECK_EQ(csw_status, 0);
    /* PREVENT/ALLOW, START STOP, VERIFY pass */
    uint8_t pa[6] = {0x1E, 0, 0, 0, 1, 0}, ss[6] = {0x1B, 0, 0, 0, 2, 0}, vf[10] = {0x2F};
    cbw(18, 0, 0, pa, 6); run_in(); CHECK_EQ(csw_status, 0);
    cbw(19, 0, 0, ss, 6); run_in(); CHECK_EQ(csw_status, 0);
    cbw(20, 0, 0, vf, 10); run_in(); CHECK_EQ(csw_status, 0);
    /* garbage instead of a CBW is ignored */
    uint8_t junk31[31] = {1, 2, 3};
    CHECK_EQ(msc_rx(&M, junk31, 31, 0), 0);
    uint8_t pkt[64];
    CHECK_EQ(msc_tx(&M, pkt), -1);
}
```
Register `test_msc` in `tests/main.c`.

- [ ] **Step 2: Run the tests and confirm they fail**

Run: `make -C tests`
Expected: compile error, `msc.h: No such file or directory`.

- [ ] **Step 3: Implement**

`fw/core/msc.h`:
```c
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
```

`fw/core/msc.c`:
```c
#include "msc.h"

enum { S_IDLE, S_IN, S_OUT, S_ZERO_IN, S_DISCARD, S_READ, S_WRITE, S_STATUS };

static uint32_t be32(const uint8_t *p) { return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]; }
static void put_be32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v; }
static uint32_t le32(const uint8_t *p) { return p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }

static void set_sense(msc_t *m, uint8_t k, uint8_t a, uint8_t q) { m->key = k; m->asc = a; m->ascq = q; }

void msc_reset(msc_t *m) { m->state = S_IDLE; m->io = 0; }

void msc_init(msc_t *m, const msc_disk_t *d) {
    m->disk = d; m->present = 1; m->attention = 0; m->wrote = 0;
    set_sense(m, 0, 0, 0);
    msc_reset(m);
}

void msc_set_present(msc_t *m, int present) {
    if (present && !m->present) m->attention = 1;
    m->present = (uint8_t)(present != 0);
}

int msc_busy(const msc_t *m, uint32_t now_ms) {
    if (m->state != S_IDLE) return 1;
    return m->wrote && (now_ms - m->last_write_ms) < 1000u;
}

/* Fail the command: fill/discard the data phase, CSW status 1, residue = Hi. */
static void fail(msc_t *m, int dir_in, uint8_t k, uint8_t a, uint8_t q) {
    set_sense(m, k, a, q);
    m->status = 1; m->residue = m->hi; m->left = m->hi;
    m->state = m->hi == 0 ? S_STATUS : dir_in ? S_ZERO_IN : S_DISCARD;
}

/* Medium usable? Otherwise fail with the right sense (consumes a pending UNIT ATTENTION). */
static int ready(msc_t *m, int dir_in) {
    if (!m->present) { fail(m, dir_in, 2, 0x3A, 0); return 0; }
    if (m->attention) { m->attention = 0; fail(m, dir_in, 6, 0x28, 0); return 0; }
    return 1;
}

/* Small data-in response already in buf[0..n): send min(n, Hi). */
static void respond(msc_t *m, uint32_t n) {
    if (n > m->hi) n = m->hi;
    m->len = n; m->pos = 0; m->status = 0; m->residue = m->hi - n;
    m->state = n ? S_IN : S_STATUS;
}

static void pass(msc_t *m) { m->status = 0; m->residue = m->hi; m->state = S_STATUS; }

static void command(msc_t *m, const uint8_t *cb, int dir_in) {
    uint8_t *b = m->buf;
    for (int i = 0; i < 36; i++) b[i] = 0;
    switch (cb[0]) {
    case 0x00:                                           /* TEST UNIT READY */
        if (ready(m, dir_in)) pass(m);
        break;
    case 0x03:                                           /* REQUEST SENSE */
        b[0] = 0x70; b[2] = m->key; b[7] = 10; b[12] = m->asc; b[13] = m->ascq;
        set_sense(m, 0, 0, 0);
        respond(m, 18);
        break;
    case 0x12: {                                         /* INQUIRY */
        static const char id[] = "DSO-510 Capture Drive   0.2 ";   /* vendor 8, product 16, rev 4 */
        b[1] = 0x80; b[2] = 0x02; b[3] = 0x02; b[4] = 31;
        for (int i = 0; i < 28; i++) b[8 + i] = (uint8_t)id[i];
        respond(m, 36);
        break;
    }
    case 0x1A:                                           /* MODE SENSE(6) */
        b[0] = 3;
        respond(m, 4);
        break;
    case 0x5A:                                           /* MODE SENSE(10) */
        b[1] = 6;
        respond(m, 8);
        break;
    case 0x1E: case 0x1B: case 0x2F:                     /* PREVENT/ALLOW, START STOP, VERIFY */
        pass(m);
        break;
    case 0x23:                                           /* READ FORMAT CAPACITIES */
        b[3] = 8; put_be32(b + 4, m->disk->blocks); b[8] = 2; b[10] = 0x10;   /* 0x001000 = 4096 */
        respond(m, 12);
        break;
    case 0x25:                                           /* READ CAPACITY(10) */
        if (!ready(m, dir_in)) break;
        put_be32(b, m->disk->blocks - 1); put_be32(b + 4, MSC_BLOCK);
        respond(m, 8);
        break;
    case 0x28: case 0x2A: {                              /* READ(10) / WRITE(10) */
        int rd = cb[0] == 0x28;
        if (!ready(m, rd)) break;
        uint32_t lba = be32(cb + 2), n = (uint32_t)cb[7] << 8 | cb[8];
        if (lba >= m->disk->blocks || n > m->disk->blocks - lba) { fail(m, rd, 5, 0x21, 0); break; }
        if (n == 0) { pass(m); break; }
        m->lba = lba; m->nblk = n; m->status = 0;
        m->residue = m->hi > n * MSC_BLOCK ? m->hi - n * MSC_BLOCK : 0;
        m->pos = 0;
        if (rd) { m->state = S_READ; m->io = 1; } else { m->state = S_WRITE; }
        break;
    }
    default:
        fail(m, dir_in, 5, 0x20, 0);                     /* invalid command opcode */
    }
}

int msc_rx(msc_t *m, const uint8_t *pkt, int len, uint32_t now_ms) {
    switch (m->state) {
    case S_IDLE:
        if (len != 31 || le32(pkt) != 0x43425355u || (pkt[14] & 0x1F) == 0 || (pkt[14] & 0x1F) > 16) return 0;
        for (int i = 0; i < 4; i++) m->tag[i] = pkt[4 + i];
        m->hi = le32(pkt + 8);
        command(m, pkt + 15, (pkt[12] & 0x80) != 0);
        return 0;
    case S_DISCARD:
        m->left = (uint32_t)len >= m->left ? 0 : m->left - (uint32_t)len;
        if (!m->left) m->state = S_STATUS;
        return 0;
    case S_WRITE:
        if (m->io) return -1;                            /* previous block still being written */
        for (int i = 0; i < len && m->pos < MSC_BLOCK; i++) m->buf[m->pos++] = pkt[i];
        m->wrote = 1; m->last_write_ms = now_ms;
        if (m->pos == MSC_BLOCK) m->io = 2;
        return 0;
    default:
        return -1;
    }
}

void msc_task(msc_t *m) {
    if (m->io == 1) {
        m->io = 0;
        if (m->disk->read(m->lba, m->buf)) {             /* unrecovered read error: zero-fill rest */
            set_sense(m, 3, 0x11, 0);
            m->status = 1; m->left = m->nblk * MSC_BLOCK; m->residue = m->hi; m->state = S_ZERO_IN;
            return;
        }
        m->len = MSC_BLOCK; m->pos = 0;
    } else if (m->io == 2) {
        m->io = 0;
        if (m->disk->write(m->lba, m->buf)) {            /* write error: discard the rest */
            set_sense(m, 3, 0x0C, 0);
            m->status = 1; m->residue = m->hi; m->left = (m->nblk - 1) * MSC_BLOCK;
            m->state = m->left ? S_DISCARD : S_STATUS;
            return;
        }
        m->lba++; m->pos = 0;
        if (--m->nblk == 0) m->state = S_STATUS;
    }
}

int msc_tx(msc_t *m, uint8_t *pkt) {
    uint32_t n;
    switch (m->state) {
    case S_IN:
        n = m->len - m->pos > MSC_PKT ? MSC_PKT : m->len - m->pos;
        for (uint32_t i = 0; i < n; i++) pkt[i] = m->buf[m->pos + i];
        m->pos += n;
        if (m->pos == m->len) m->state = S_STATUS;
        return (int)n;
    case S_READ:
        if (m->io || m->pos >= MSC_BLOCK) return -1;     /* waiting for msc_task */
        for (uint32_t i = 0; i < MSC_PKT; i++) pkt[i] = m->buf[m->pos + i];
        m->pos += MSC_PKT;
        if (m->pos == MSC_BLOCK) {
            m->lba++;
            if (--m->nblk == 0) m->state = S_STATUS; else m->io = 1;
        }
        return MSC_PKT;
    case S_ZERO_IN:
        n = m->left > MSC_PKT ? MSC_PKT : m->left;
        for (uint32_t i = 0; i < n; i++) pkt[i] = 0;
        m->left -= n;
        if (!m->left) m->state = S_STATUS;
        return (int)n;
    case S_STATUS:
        pkt[0] = 0x55; pkt[1] = 0x53; pkt[2] = 0x42; pkt[3] = 0x53;
        for (int i = 0; i < 4; i++) pkt[4 + i] = m->tag[i];
        pkt[8] = (uint8_t)m->residue; pkt[9] = (uint8_t)(m->residue >> 8);
        pkt[10] = (uint8_t)(m->residue >> 16); pkt[11] = (uint8_t)(m->residue >> 24);
        pkt[12] = m->status;
        m->state = S_IDLE;
        return 13;
    default:
        return -1;
    }
}
```
Notes for the implementer:
- In `S_READ` the first block is loaded by `msc_task` (`io = 1`, set in `command`). `msc_tx` returns -1 until `io` is cleared, and `msc_task` resets `pos = 0` when the block is ready.
- `S_ZERO_IN` with `left == 0` cannot happen, because `fail()` goes straight to `S_STATUS` when `hi == 0`.
- The read-error path in `msc_task` zero-fills everything not yet sent. `nblk` still counts the current block.

- [ ] **Step 4: Run the tests and confirm they pass**

Run: `make -C tests`
Expected: `… 0 failed`. If a check fails, fix `msc.c`, not the test. The test encodes the spec's sense codes and the BOT rules.

- [ ] **Step 5: Commit**

```bash
git add fw/core/msc.h fw/core/msc.c tests/test_msc.c tests/main.c
git commit -m "core: USB mass-storage BOT + SCSI state machine (fill/discard on failure, eject sense)

Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp"
```

---

### Task 6: `usb_std` — descriptors and control requests (pure)

**Files:**
- Create: `fw/core/usb_std.h`, `fw/core/usb_std.c`, `tests/test_usb_std.c`
- Modify: `tests/main.c`

**Interfaces:**
- Produces: `USB_VID`, `USB_PID`, `usb_dev_t`, `usb_act_t` (`USB_STALL`, `USB_DATA_IN`, `USB_STATUS_IN`), `usb_resp_t`, `void usb_std_init(usb_dev_t *d)`, `void usb_std_setup(usb_dev_t *d, const uint8_t setup[8], usb_resp_t *r)`.

- [ ] **Step 1: Write the failing test**

`tests/test_usb_std.c`:
```c
#include "test.h"
#include "usb_std.h"

static usb_dev_t D;
static usb_resp_t R;
static void setup(uint8_t type, uint8_t req, uint16_t val, uint16_t idx, uint16_t len) {
    uint8_t s[8] = {type, req, (uint8_t)val, (uint8_t)(val >> 8), (uint8_t)idx, (uint8_t)(idx >> 8),
                    (uint8_t)len, (uint8_t)(len >> 8)};
    usb_std_setup(&D, s, &R);
}

void test_usb_std(void) {
    usb_std_init(&D);
    setup(0x80, 6, 0x0100, 0, 64);                       /* device descriptor */
    CHECK_EQ(R.act, USB_DATA_IN); CHECK_EQ(R.len, 18);
    CHECK_EQ(R.data[0], 18); CHECK_EQ(R.data[1], 1); CHECK_EQ(R.data[7], 64);
    CHECK_EQ(R.data[8], 0x3C); CHECK_EQ(R.data[9], 0x2E); CHECK_EQ(R.data[10], 0x20); CHECK_EQ(R.data[11], 0x57);
    setup(0x80, 6, 0x0100, 0, 8);
    CHECK_EQ(R.len, 8);                                    /* truncated to wLength */
    setup(0x80, 6, 0x0200, 0, 9);                        /* config header only */
    CHECK_EQ(R.len, 9); CHECK_EQ(R.data[2], 32);
    setup(0x80, 6, 0x0200, 0, 255);
    CHECK_EQ(R.len, 32);
    CHECK_EQ(R.data[9 + 5], 0x08); CHECK_EQ(R.data[9 + 6], 0x06); CHECK_EQ(R.data[9 + 7], 0x50); /* MSC SCSI BOT */
    CHECK_EQ(R.data[18 + 2], 0x81); CHECK_EQ(R.data[18 + 3], 2); CHECK_EQ(R.data[18 + 4], 64);
    CHECK_EQ(R.data[25 + 2], 0x01);
    setup(0x80, 6, 0x0300, 0, 255);                      /* language ids */
    CHECK_EQ(R.len, 4); CHECK_EQ(R.data[2], 0x09); CHECK_EQ(R.data[3], 0x04);
    setup(0x80, 6, 0x0302, 0x0409, 255);                 /* product "DSO-510 Capture Drive" */
    CHECK_EQ(R.len, 2 + 2 * 21); CHECK_EQ(R.data[1], 3); CHECK_EQ(R.data[2], 'D'); CHECK_EQ(R.data[3], 0);
    setup(0x80, 6, 0x0303, 0x0409, 255);                 /* serial, 12 chars */
    CHECK_EQ(R.len, 2 + 2 * 12);
    setup(0x80, 6, 0x0304, 0x0409, 255);
    CHECK_EQ(R.act, USB_STALL);
    setup(0x80, 6, 0x0600, 0, 10);                       /* device qualifier: full-speed only */
    CHECK_EQ(R.act, USB_STALL);
    setup(0x00, 5, 7, 0, 0);                             /* SET_ADDRESS */
    CHECK_EQ(R.act, USB_STATUS_IN); CHECK_EQ(R.set_address, 1); CHECK_EQ(R.new_address, 7);
    setup(0x00, 9, 1, 0, 0);                             /* SET_CONFIGURATION 1 */
    CHECK_EQ(R.act, USB_STATUS_IN); CHECK_EQ(D.config, 1); CHECK_EQ(R.configured_changed, 1);
    setup(0x80, 8, 0, 0, 1);
    CHECK_EQ(R.act, USB_DATA_IN); CHECK_EQ(R.len, 1); CHECK_EQ(R.data[0], 1);
    setup(0x00, 9, 2, 0, 0);
    CHECK_EQ(R.act, USB_STALL);
    setup(0x80, 0, 0, 0, 2);                             /* GET_STATUS */
    CHECK_EQ(R.len, 2); CHECK_EQ(R.data[0], 0);
    setup(0x02, 1, 0, 0x81, 0);                          /* CLEAR_FEATURE ENDPOINT_HALT 0x81 */
    CHECK_EQ(R.act, USB_STATUS_IN); CHECK_EQ(R.clear_halt_ep, 0x81);
    setup(0xA1, 0xFE, 0, 0, 1);                          /* GET_MAX_LUN */
    CHECK_EQ(R.act, USB_DATA_IN); CHECK_EQ(R.len, 1); CHECK_EQ(R.data[0], 0);
    setup(0x21, 0xFF, 0, 0, 0);                          /* BOT reset */
    CHECK_EQ(R.act, USB_STATUS_IN); CHECK_EQ(R.bot_reset, 1);
    setup(0x01, 11, 0, 0, 0);                            /* SET_INTERFACE alt 0 */
    CHECK_EQ(R.act, USB_STATUS_IN);
    setup(0x40, 0x55, 0, 0, 0);                          /* vendor request */
    CHECK_EQ(R.act, USB_STALL);
    setup(0x00, 9, 0, 0, 0);                             /* deconfigure */
    CHECK_EQ(D.config, 0); CHECK_EQ(R.configured_changed, 1);
}
```
Register `test_usb_std` in `tests/main.c`.

- [ ] **Step 2: Run the tests and confirm they fail**

Run: `make -C tests`
Expected: compile error, `usb_std.h: No such file or directory`.

- [ ] **Step 3: Implement**

`fw/core/usb_std.h`:
```c
#pragma once
#include <stdint.h>

#define USB_VID 0x2E3C
#define USB_PID 0x5720

typedef enum { USB_STALL = 0, USB_DATA_IN, USB_STATUS_IN } usb_act_t;

typedef struct {
    usb_act_t act;
    const uint8_t *data;       /* USB_DATA_IN: bytes to send */
    uint16_t len;              /* already limited to wLength */
    uint8_t set_address, new_address;   /* apply the address after the status stage */
    uint8_t configured_changed;
    uint8_t clear_halt_ep;     /* 0 = none */
    uint8_t bot_reset;
} usb_resp_t;

typedef struct {
    uint8_t config;
    uint8_t tmp[2];
    uint8_t str[4][2 + 2 * 24];   /* string descriptors 0..3, built by usb_std_init */
} usb_dev_t;

void usb_std_init(usb_dev_t *d);
void usb_std_setup(usb_dev_t *d, const uint8_t setup[8], usb_resp_t *r);
```

`fw/core/usb_std.c`:
```c
#include "usb_std.h"

static const uint8_t dev_desc[18] = {
    18, 1, 0x00, 0x02, 0, 0, 0, 64,
    (uint8_t)USB_VID, (uint8_t)(USB_VID >> 8), (uint8_t)USB_PID, (uint8_t)(USB_PID >> 8),
    0x00, 0x02, 1, 2, 3, 1,
};

static const uint8_t cfg_desc[32] = {
    9, 2, 32, 0, 1, 1, 0, 0x80, 50,                 /* config: 1 interface, bus powered 100 mA */
    9, 4, 0, 0, 2, 0x08, 0x06, 0x50, 0,              /* interface: MSC, SCSI transparent, BOT */
    7, 5, 0x81, 0x02, 64, 0, 0,                      /* EP1 IN bulk 64 */
    7, 5, 0x01, 0x02, 64, 0, 0,                      /* EP1 OUT bulk 64 */
};

static void make_str(uint8_t *out, const char *s) {
    int n = 0;
    while (s[n]) { out[2 + 2 * n] = (uint8_t)s[n]; out[3 + 2 * n] = 0; n++; }
    out[0] = (uint8_t)(2 + 2 * n); out[1] = 3;
}

void usb_std_init(usb_dev_t *d) {
    d->config = 0;
    d->str[0][0] = 4; d->str[0][1] = 3; d->str[0][2] = 0x09; d->str[0][3] = 0x04;
    make_str(d->str[1], "DSO-510");
    make_str(d->str[2], "DSO-510 Capture Drive");
    make_str(d->str[3], "000000000001");
}

static void in(usb_resp_t *r, const uint8_t *p, uint16_t n, uint16_t wlen) {
    r->act = USB_DATA_IN; r->data = p; r->len = n < wlen ? n : wlen;
}

void usb_std_setup(usb_dev_t *d, const uint8_t s[8], usb_resp_t *r) {
    uint8_t type = s[0], req = s[1];
    uint16_t val = (uint16_t)(s[2] | s[3] << 8), idx = (uint16_t)(s[4] | s[5] << 8);
    uint16_t wlen = (uint16_t)(s[6] | s[7] << 8);
    r->act = USB_STALL; r->data = 0; r->len = 0;
    r->set_address = 0; r->new_address = 0; r->configured_changed = 0; r->clear_halt_ep = 0; r->bot_reset = 0;

    if ((type & 0x60) == 0x20) {                         /* class (MSC BOT) */
        if (type == 0xA1 && req == 0xFE) { d->tmp[0] = 0; in(r, d->tmp, 1, wlen); }
        else if (type == 0x21 && req == 0xFF) { r->act = USB_STATUS_IN; r->bot_reset = 1; }
        return;
    }
    if (type & 0x60) return;                             /* vendor: stall */
    switch (req) {
    case 6:                                              /* GET_DESCRIPTOR */
        if (type != 0x80) return;
        switch (val >> 8) {
        case 1: in(r, dev_desc, sizeof dev_desc, wlen); break;
        case 2: in(r, cfg_desc, sizeof cfg_desc, wlen); break;
        case 3: if ((val & 0xFF) < 4) in(r, d->str[val & 0xFF], d->str[val & 0xFF][0], wlen); break;
        default: break;                                  /* qualifier etc.: stall */
        }
        break;
    case 5:                                              /* SET_ADDRESS */
        r->act = USB_STATUS_IN; r->set_address = 1; r->new_address = (uint8_t)(val & 0x7F);
        break;
    case 9:                                              /* SET_CONFIGURATION */
        if (val > 1) return;
        d->config = (uint8_t)val; r->act = USB_STATUS_IN; r->configured_changed = 1;
        break;
    case 8:                                              /* GET_CONFIGURATION */
        d->tmp[0] = d->config; in(r, d->tmp, 1, wlen);
        break;
    case 0:                                              /* GET_STATUS */
        d->tmp[0] = 0; d->tmp[1] = 0; in(r, d->tmp, 2, wlen);
        break;
    case 1: case 3:                                      /* CLEAR_FEATURE / SET_FEATURE */
        r->act = USB_STATUS_IN;
        if (req == 1 && (type & 0x1F) == 2 && val == 0) r->clear_halt_ep = (uint8_t)idx;
        break;
    case 11:                                             /* SET_INTERFACE */
        if (val == 0) r->act = USB_STATUS_IN;
        break;
    case 10:                                             /* GET_INTERFACE */
        d->tmp[0] = 0; in(r, d->tmp, 1, wlen);
        break;
    default:
        break;
    }
}
```

- [ ] **Step 4: Run the tests and confirm they pass**

Run: `make -C tests`
Expected: `… 0 failed`.

- [ ] **Step 5: Commit**

```bash
git add fw/core/usb_std.h fw/core/usb_std.c tests/test_usb_std.c tests/main.c
git commit -m "core: USB descriptors (MSC BOT, 2e3c:5720) and standard/class control requests

Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp"
```

---

### Task 7: Polled USB device driver + `usbdisk` test app

**Files:**
- Create: `fw/common/usbd.h`, `fw/common/usbd.c`, `fw/apps/usbdisk/main.c`
- Modify: `fw/common/board.h`, `fw/common/board.c` (weak `board_idle()` called from `delay_ms`)

**Interfaces:**
- Consumes: `msc_*` (Task 5), `usb_std_*` (Task 6), `vol_read`/`vol_write` (Task 4), `USER_VOL_SECTORS` (Task 1).
- Produces:
  - `void usbd_init(msc_t *m)`, `void usbd_poll(void)`, `int usbd_configured(void)` and `uint32_t usbd_last_sof_ms(void)`.
  - `void board_idle(void)`. It is weak and empty, the app overrides it, and `delay_ms` calls it.

Hardware facts. They come from stock V2.0.5 `FUN_0802…` around `decomp.c` lines 28686–28725 and 14786–15155. This is an STM32F1-compatible USBD:
- **Registers:** base 0x40005C00, `EPnR` at +4n, `CNTR` +0x40, `ISTR` +0x44, `FNR` +0x48, `DADDR` +0x4C, `BTABLE` +0x50.
- **AT32 `USB_CFG` +0x60:** bit 1 = 1 disconnects the internal D+ pull-up. Stock sets `CNTR |= 2` (PDWN) and `CFG |= 2` to disconnect, and clears both to connect.
- **Packet memory:** at 0x40006000, with a 2× address stride (16-bit word at PMA offset `o` lives at `0x40006000 + 2*o`).
- **USB clock = HICK 48 MHz**, trimmed by the ACC unit against USB SOF. Stock init sequence:
  - `CRM_MISC1 |= 1<<25`, then `CRM_MISC3 |= 0x200` and `|= 0x100`, then `CRM_APB2EN |= 1<<22` (ACC).
  - `ACC_C1 = 0x1F2C`, `ACC_C2 = 8000`, `ACC_C3 = 0x1F54` (0x4001580C, 0x40015810, 0x40015814), then `ACC_CTRL1 |= 2` and `|= 1` (ACC_CTRL1 is at 0x40015804).
  - Finally `CRM_APB1EN |= 1<<23` (USB).
- The driver is **polled**: no NVIC interrupt is enabled. `usbd_poll()` runs from the main loop and from `board_idle()`.

- [ ] **Step 1: Add the idle hook**

In `fw/common/board.h`, after `void delay_ms(uint32_t ms);`, add:
```c
void board_idle(void);           // weak, empty by default; called from delay_ms() (apps service USB here)
```
In `fw/common/board.c`, replace `delay_ms` with:
```c
__attribute__((weak)) void board_idle(void) { }

void delay_ms(uint32_t ms) {
    uint32_t t0 = g_ms;
    while (g_ms - t0 < ms) { WDT_CMD = 0xAAAA; board_idle(); }   // watchdog + background work
}
```

- [ ] **Step 2: Write the driver**

`fw/common/usbd.h`:
```c
#pragma once
#include <stdint.h>
#include "msc.h"
/* Polled USB FS device: EP0 control + EP1 bulk (0x81 IN / 0x01 OUT) for mass storage.
   No interrupts: call usbd_poll() often (main loop and board_idle()). Re-entrant calls return. */
void usbd_init(msc_t *m);        /* clocks, peripheral reset, D+ pull-up on */
void usbd_poll(void);
int usbd_configured(void);       /* configured by a host and SOFs seen within the last second */
uint32_t usbd_last_sof_ms(void);
```

`fw/common/usbd.c`:
```c
#include "usbd.h"
#include "usb_std.h"
#include "board.h"

#define USB_BASE   0x40005C00u
#define USB_EPR(n) REG32(USB_BASE + 4u * (n))
#define USB_CNTR   REG32(USB_BASE + 0x40)
#define USB_ISTR   REG32(USB_BASE + 0x44)
#define USB_DADDR  REG32(USB_BASE + 0x4C)
#define USB_BTABLE REG32(USB_BASE + 0x50)
#define USB_CFG    REG32(USB_BASE + 0x60)      /* AT32: bit 1 = 1 disables the D+ pull-up */
#define PMA(off)   (*(volatile uint32_t *)(0x40006000u + 2u * (off)))
#define ACC_CTRL1  REG32(0x40015804)
#define ACC_C1     REG32(0x4001580C)
#define ACC_C2     REG32(0x40015810)
#define ACC_C3     REG32(0x40015814)

#define ISTR_CTR   0x8000u
#define ISTR_SUSP  0x0800u
#define ISTR_WKUP  0x1000u
#define ISTR_RESET 0x0400u
#define ISTR_SOF   0x0200u
#define EP_CTR_RX  0x8000u
#define EP_SETUP   0x0800u
#define EP_CTR_TX  0x0080u
#define EP_MASK    0x8F8Fu                      /* non-toggle bits: CTR_RX SETUP TYPE KIND CTR_TX EA */
#define EP_CONTROL 0x0200u
#define ST_STALL 1u
#define ST_NAK   2u
#define ST_VALID 3u

/* PMA layout (byte offsets): BTABLE 0x00, EP0 RX 0x40, EP0 TX 0x80, EP1 RX 0xC0, EP1 TX 0x100 */
#define EP0_RX 0x40u
#define EP0_TX 0x80u
#define EP1_RX 0xC0u
#define EP1_TX 0x100u
#define RX64   0x8400u                          /* COUNT_RX: BL_SIZE=1, NUM_BLOCK=1 -> 64 bytes */

static msc_t *msc;
static usb_dev_t dev;
static uint8_t inited, busy, suspended, tx_busy, rx_held, rx_len, pending_addr, addr;
static uint8_t rxbuf[64], txbuf[64];
static const uint8_t *ctl_ptr;
static uint16_t ctl_left;
static uint8_t ctl_zlp;
static uint32_t last_sof;

static void ep_set_tx(int n, uint32_t s) { uint32_t v = USB_EPR(n) & (EP_MASK | 0x0030u); v ^= s << 4; USB_EPR(n) = v | 0x8080u; }
static void ep_set_rx(int n, uint32_t s) { uint32_t v = USB_EPR(n) & (EP_MASK | 0x3000u); v ^= s << 12; USB_EPR(n) = v | 0x8080u; }
static void ep_clr_ctr_rx(int n) { USB_EPR(n) = (USB_EPR(n) & EP_MASK & ~EP_CTR_RX) | EP_CTR_TX; }
static void ep_clr_ctr_tx(int n) { USB_EPR(n) = (USB_EPR(n) & EP_MASK & ~EP_CTR_TX) | EP_CTR_RX; }
static void ep_clr_dtog(int n) { uint32_t v = USB_EPR(n); USB_EPR(n) = (v & EP_MASK) | (v & 0x4040u) | 0x8080u; }

static void pma_write(uint32_t off, const uint8_t *b, int n) {
    for (int i = 0; i < n; i += 2) PMA(off + (uint32_t)i) = b[i] | (uint32_t)(i + 1 < n ? b[i + 1] : 0) << 8;
}
static void pma_read(uint32_t off, uint8_t *b, int n) {
    for (int i = 0; i < n; i += 2) {
        uint32_t w = PMA(off + (uint32_t)i);
        b[i] = (uint8_t)w;
        if (i + 1 < n) b[i + 1] = (uint8_t)(w >> 8);
    }
}
static int rx_count(int ep) { return (int)(PMA((uint32_t)ep * 8u + 6u) & 0x3FFu); }

static void bus_reset(void) {
    USB_BTABLE = 0;
    PMA(0) = EP0_TX; PMA(2) = 0; PMA(4) = EP0_RX; PMA(6) = RX64;
    PMA(8) = EP1_TX; PMA(10) = 0; PMA(12) = EP1_RX; PMA(14) = RX64;
    USB_EPR(0) = EP_CONTROL | 0u;
    ep_clr_dtog(0); ep_set_rx(0, ST_VALID); ep_set_tx(0, ST_NAK);
    USB_EPR(1) = 0x0000u | 1u;                          /* bulk, address 1 */
    ep_clr_dtog(1); ep_set_rx(1, ST_VALID); ep_set_tx(1, ST_NAK);
    USB_DADDR = 0x80u;                                  /* enable, address 0 */
    usb_std_init(&dev);
    msc_reset(msc);
    tx_busy = rx_held = pending_addr = ctl_left = ctl_zlp = 0;
    suspended = 0;
}

static void ctl_send(void) {
    uint16_t n = ctl_left > 64 ? 64 : ctl_left;
    pma_write(EP0_TX, ctl_ptr, n);
    PMA(2) = n;
    ctl_ptr += n; ctl_left = (uint16_t)(ctl_left - n);
    if (n < 64) ctl_zlp = 0;                            /* a short packet ends the transfer */
    ep_set_tx(0, ST_VALID);
}

static void ep0_setup(void) {
    uint8_t s[8];
    usb_resp_t r;
    pma_read(EP0_RX, s, 8);
    ep_clr_ctr_rx(0);
    usb_std_setup(&dev, s, &r);
    if (r.bot_reset) msc_reset(msc);
    if (r.set_address) { pending_addr = 1; addr = r.new_address; }
    switch (r.act) {
    case USB_DATA_IN: {
        uint16_t wlen = (uint16_t)(s[6] | s[7] << 8);
        ctl_ptr = r.data; ctl_left = r.len;
        ctl_zlp = (uint8_t)(r.len < wlen && (r.len % 64) == 0);   /* end with a ZLP */
        ctl_send();
        ep_set_rx(0, ST_VALID);                         /* status OUT */
        break;
    }
    case USB_STATUS_IN:
        PMA(2) = 0; ep_set_tx(0, ST_VALID); ep_set_rx(0, ST_VALID);
        break;
    default:
        ep_set_tx(0, ST_STALL); ep_set_rx(0, ST_VALID);  /* stall; next SETUP still accepted */
        break;
    }
}

static void service_ctr(void) {
    while (USB_ISTR & ISTR_CTR) {
        int ep = (int)(USB_ISTR & 0xFu);
        uint32_t v = USB_EPR(ep);
        if (ep == 0) {
            if (v & EP_CTR_TX) {
                ep_clr_ctr_tx(0);
                if (pending_addr) { USB_DADDR = 0x80u | addr; pending_addr = 0; }
                if (ctl_left || ctl_zlp) ctl_send();
            }
            if (v & EP_CTR_RX) {
                if (v & EP_SETUP) ep0_setup();
                else { ep_clr_ctr_rx(0); ep_set_rx(0, ST_VALID); }   /* status OUT */
            }
        } else if (ep == 1) {
            if (v & EP_CTR_RX) {                        /* hardware NAKs further OUT packets now */
                ep_clr_ctr_rx(1);
                rx_len = (uint8_t)rx_count(1);
                pma_read(EP1_RX, rxbuf, rx_len);
                rx_held = 1;
            }
            if (v & EP_CTR_TX) { ep_clr_ctr_tx(1); tx_busy = 0; }
        } else {
            USB_EPR(ep) = 0;                            /* unexpected endpoint */
        }
    }
}

void usbd_init(msc_t *m) {
    msc = m;
    CRM_MISC1 |= 1u << 25;                              /* stock: HICK 48 MHz for USB */
    CRM_MISC3 |= 0x200u;
    CRM_MISC3 |= 0x100u;
    CRM_APB2EN |= 1u << 22;                             /* ACC: trim HICK to USB SOF */
    ACC_C1 = 0x1F2C; ACC_C2 = 8000; ACC_C3 = 0x1F54;
    ACC_CTRL1 |= 2u;
    ACC_CTRL1 |= 1u;
    CRM_APB1EN |= 1u << 23;                             /* USB */
    USB_CNTR = 1u;                                      /* FRES, power up (PDWN = 0) */
    for (uint32_t t0 = millis(); millis() - t0 < 2; ) WDT_CMD = 0xAAAA;
    USB_CNTR = 0u;
    USB_ISTR = 0u;
    bus_reset();
    USB_CFG &= ~2u;                                     /* D+ pull-up on: host may enumerate */
    inited = 1;
}

void usbd_poll(void) {
    if (!inited || busy) return;
    busy = 1;
    uint32_t is = USB_ISTR;
    if (is & ISTR_RESET) { USB_ISTR = (uint16_t)~ISTR_RESET; bus_reset(); }
    if (is & ISTR_SOF) { USB_ISTR = (uint16_t)~ISTR_SOF; last_sof = millis(); suspended = 0; }
    if (is & ISTR_SUSP) { USB_ISTR = (uint16_t)~ISTR_SUSP; suspended = 1; }
    if (is & ISTR_WKUP) { USB_ISTR = (uint16_t)~ISTR_WKUP; suspended = 0; }
    USB_ISTR = (uint16_t)~(0x6000u | 0x0100u);          /* clear PMAOVR, ERR, ESOF */
    service_ctr();
    if (rx_held && msc_rx(msc, rxbuf, rx_len, millis()) == 0) { rx_held = 0; ep_set_rx(1, ST_VALID); }
    msc_task(msc);
    if (rx_held && msc_rx(msc, rxbuf, rx_len, millis()) == 0) { rx_held = 0; ep_set_rx(1, ST_VALID); }
    if (!tx_busy && dev.config) {
        int n = msc_tx(msc, txbuf);
        if (n >= 0) { pma_write(EP1_TX, txbuf, n); PMA(10) = (uint32_t)n; ep_set_tx(1, ST_VALID); tx_busy = 1; }
    }
    busy = 0;
}

int usbd_configured(void) {
    return inited && dev.config && !suspended && (millis() - last_sof) < 1000u;
}

uint32_t usbd_last_sof_ms(void) { return last_sof; }
```

- [ ] **Step 3: Write the `usbdisk` test app**

`fw/apps/usbdisk/main.c`:
```c
// usbdisk: the 5 MiB user volume as a USB drive, nothing else. Status on screen; hold POWER = off.
#include "board.h"
#include "lcd.h"
#include "spiflash.h"
#include "flashmap.h"
#include "vol.h"
#include "msc.h"
#include "usbd.h"

static msc_t msc;
static uint32_t nread, nwrite;
static int rd(uint32_t lba, uint8_t *b) { nread++; return vol_read(lba, b); }
static int wr(uint32_t lba, const uint8_t *b) { nwrite++; return vol_write(lba, b); }
static const msc_disk_t disk = {rd, wr, USER_VOL_SECTORS};

void board_idle(void) { usbd_poll(); }

int main(void) {
    board_init();
    keys_init();
    lcd_init();
    lcd_fill(0, 0, LCD_W, LCD_H, C_BLACK);
    backlight(1);
    lcd_text(0, 0, " usbdisk: plug USB into a computer", C_CYAN, C_BLACK);
    spiflash_init();
    msc_init(&msc, &disk);
    usbd_init(&msc);
    uint32_t next = 0;
    char b[12];
    for (;;) {
        usbd_poll();
        uint32_t now = millis();
        if ((int32_t)(now - next) >= 0) {
            next = now + 250;
            lcd_text(0, 2, usbd_configured() ? " host: CONFIGURED " : " host: none       ",
                     usbd_configured() ? C_GREEN : C_GRAY, C_BLACK);
            int c = lcd_text(0, 3, " blocks read  ", C_GRAY, C_BLACK);
            lcd_text(c, 3, fmt_x(b, nread, 6), C_WHITE, C_BLACK);
            c = lcd_text(0, 4, " blocks write ", C_GRAY, C_BLACK);
            lcd_text(c, 4, fmt_x(b, nwrite, 6), C_WHITE, C_BLACK);
            lcd_text(0, 14, " Hold POWER = off", C_GRAY, C_BLACK);
        }
        if (keys_read() & KEY_BIT(KEY_POWER)) {
            uint32_t t0 = millis();
            while (keys_read() & KEY_BIT(KEY_POWER)) {
                usbd_poll();
                if (millis() - t0 > 1000) { backlight(0); power_off(); for (;;) usbd_poll(); }
            }
        }
        WDT_CMD = 0xAAAA;
    }
}
```

- [ ] **Step 4: Build**

Run: `make -C fw everything 2>&1 | grep -ciE "warning|error"` and `make -C tests`
Expected: `0` warnings, and the tests still pass.

- [ ] **Step 5: Commit**

```bash
git add fw/common/usbd.h fw/common/usbd.c fw/common/board.h fw/common/board.c fw/apps/usbdisk/main.c
git commit -m "usbd: polled AT32 USB FS device (HICK+ACC 48 MHz, PMA, EP0 + bulk EP1); usbdisk test app

Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp"
```

- [ ] **Step 6 (controller with the user, on device):**
  1. Before any writing, take a byte-exact backup of the drive. Flash `usbdisk`, plug in USB, and identify the device with `lsblk -o NAME,SIZE,MODEL,VENDOR`. Then run `sudo dd if=/dev/sdX of=dumps/userdrive-backup.img bs=4096 count=1280` and keep the image out of git.
  2. Check `dmesg | tail -30`: the device should appear as `2e3c:5720`, 1280 × 4096-byte logical blocks, removable.
  3. Mount the drive, list it, and compare with `dumps/storage/`.
  4. Copy a 1 MB random file onto the drive, unmount, remount, then compare `sha256sum` before and after. Delete the file afterwards.
  5. Power the scope off and on, then run the stock-like check again: mount and list.

---

### Task 8: App integration — double-buffered record, save flow, USB in the scope app

**Files:**
- Create: `fw/app/save.h`, `fw/app/save.c`
- Modify: `fw/app/main.c`

**Interfaces:**
- Consumes: `capsave_write`, `cap_res_t` (Task 4); `csv_meta_t` (Task 2); `msc_*` (Task 5); `usbd_*` and `board_idle` (Task 7); `vol_*` (Task 4); `spiflash_init` (Task 3).
- Produces: `const char *save_capture(const uint8_t *rec, int n, const csv_meta_t *m, msc_t *msc)`. It returns a 14-character toast text.

- [ ] **Step 1: Write the save flow**

`fw/app/save.h`:
```c
#pragma once
#include <stdint.h>
#include "csv.h"
#include "msc.h"
/* Save one capture as CSV. If a USB host is attached, waits for it to go idle (<= 5 s), reports
   the medium as removed while writing, then re-inserts it (UNIT ATTENTION). Returns a 14-column
   toast text: " Saved CAPnnnn", " Drive full   ", " Save failed  " or " Busy, retry  ". */
const char *save_capture(const uint8_t *rec, int n, const csv_meta_t *m, msc_t *msc);
```

`fw/app/save.c`:
```c
#include "save.h"
#include "capsave.h"
#include "usbd.h"
#include "board.h"

const char *save_capture(const uint8_t *rec, int n, const csv_meta_t *m, msc_t *msc) {
    static char msg[15] = " Saved CAP0000";
    int host = usbd_configured();
    if (host) {
        uint32_t t0 = millis();
        while (msc_busy(msc, millis())) {
            usbd_poll();
            WDT_CMD = 0xAAAA;
            if (millis() - t0 > 5000) return " Busy, retry  ";
        }
        msc_set_present(msc, 0);                        /* host sees MEDIUM NOT PRESENT */
    }
    int num = 0;
    cap_res_t r = capsave_write(rec, n, m, &num);
    if (host) msc_set_present(msc, 1);                  /* host sees UNIT ATTENTION, re-reads */
    if (r == CAP_FULL) return " Drive full   ";
    if (r != CAP_OK) return " Save failed  ";
    for (int i = 13; i >= 10; i--) { msg[i] = (char)('0' + num % 10); num /= 10; }
    return msg;
}
```
`msc_set_present(msc, 0)` is called only when `msc_busy()` is false, which means the BOT state is IDLE. That is safe because the next command the host sends will simply get NOT READY.

- [ ] **Step 2: Wire it into `fw/app/main.c`**

Make these edits:
1. Includes: add `#include "spiflash.h"`, `#include "flashmap.h"`, `#include "vol.h"`, `#include "msc.h"`, `#include "usbd.h"` and `#include "save.h"`.
2. Double-buffer the record. Replace `static uint8_t rec[REC_MAX];` with:
```c
static uint8_t recbuf[2][REC_MAX];
static uint8_t *rec = recbuf[0];        /* buffer being captured into */
static uint8_t *shown;                  /* last processed capture (on screen) */
static int shown_n;
static csv_meta_t shown_meta;           /* settings the shown capture was taken with */
```
3. In `process_capture()`, once the capture is accepted (after the `start` / Normal-Single early `return`, before rendering), add the following. The metadata snapshot is taken at capture time, so later knob changes don't corrupt a saved file. The buffers are swapped, so the next capture never overwrites what is shown and saved.
```c
    shown = rec; shown_n = rec_n;
    shown_meta.rate_hz = rc.rate_hz;
    shown_meta.vdiv_mv = vdiv_mv[st.vdiv];
    shown_meta.coupling_dc = st.coupling_dc;
    shown_meta.probe_x10 = st.probe_x10;
    shown_meta.trig_mode = st.trig_mode;
    shown_meta.edge = st.edge;
    shown_meta.zero_code = (int16_t)(ADC_CENTER + st.offset);
    shown_meta.trig_level_code = st.trig_level;
    shown_meta.trig_index = t >= 0 ? t : start + pre;
    shown_meta.offset_pwm = fe_pwm;
    rec = (rec == recbuf[0]) ? recbuf[1] : recbuf[0];
```
In the rest of `process_capture()`, `render_spans(…)` and `measure(…)` must use `shown` instead of `rec`. They run after the swap, so they read the buffer that was just captured.
4. `minmax()`, `mean_x16_at_pwm()`, the auto-set loop and the auto-zero code all capture with `capture_blocking()` into `rec` and read `rec` straight away. That stays correct, because they never swap.
5. Globals: add
```c
static msc_t msc;
static const msc_disk_t vol_disk = {vol_read, vol_write, USER_VOL_SECTORS};
void board_idle(void) { usbd_poll(); }
```
6. In `capture_blocking()`'s wait loop and in `wait_power_aware()`'s loop, call `board_idle();` next to the `WDT_CMD` reload, so USB keeps being serviced during auto-set and auto-zero.
7. In `main()`, after `cal_out_init();` add:
```c
    spiflash_init();
    msc_init(&msc, &vol_disk);
    usbd_init(&msc);
```
8. In the main loop, call `usbd_poll();` once per iteration, before the key handling.
9. Replace `if (chg & ACT_SAVE) toast(" CSV: step 2  ", 1500);` with:
```c
        if (chg & ACT_SAVE) {
            if (!have_trace || !shown) toast(" No capture   ", 1500);
            else {
                adc_abort(); armed = 0;
                toast(" Saving...    ", 60000);
                const char *r = save_capture(shown, shown_n, &shown_meta, &msc);
                toast(r, 2500);
            }
        }
```
`toast()` keeps the pointer, and `save_capture` returns either a string literal or its own static buffer, so the pointer stays valid.

- [ ] **Step 3: Build and run the tests**

Run: `make -C fw everything 2>&1 | grep -ciE "warning|error"` and `make -C tests`
Expected: `0`, and all checks pass.

- [ ] **Step 4: Commit**

```bash
git add fw/app/save.h fw/app/save.c fw/app/main.c
git commit -m "app: long-press Run saves the shown capture as CSV; USB drive with eject/re-insert on save

Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp"
```

---

### Task 9: On-device verification (controller with the user)

**Files:**
- Modify: `NOTES.md`, `docs/superpowers/specs/2026-11-13-dso510-firmware-design.md` (status line)

- [ ] **Step 1: Standalone save.** Flash `APP_DSO-510_app.bin`. Generator sine, Auto, then long-press Run. The toast shows " Saved CAP0001". Save two more times, including once in STOP mode. Expect CAP0002 and CAP0003. Time a save at the slowest timebase (50 ms/div, 4800 rows); the spec target is ≤ 2 s. Record the measured time.
- [ ] **Step 2: Auto-mount.** Plug USB into Linux. The drive appears and auto-mounts, and the scope keeps capturing. `/CSV/CAP0001..3.CSV` exist. Check the header against the spec. Plot one file with Python: `python3 -c "import csv,sys; ..."`, or just `head`/`wc -l` it. The row count should match the record length, 4 × samples per screen.
- [ ] **Step 3: Save while mounted.** With the drive mounted, long-press Run. The file manager briefly loses the drive, then it comes back with the new file. Then copy a file onto the drive and immediately long-press Run: you get " Busy, retry  " or a delayed save, and no corruption. Run `fsck.vfat -n` on the unmounted device and confirm no errors.
- [ ] **Step 4: Charger.** Use a phone charger or another power-only source. Nothing appears and the scope works normally.
- [ ] **Step 4b (optional): Windows.** If a Windows machine is available, repeat Steps 2–3 on it.
- [ ] **Step 5: Stock compatibility.** Reflash the stock `APP_DSO-510_V2.0.5_250114.bin`. It boots normally, with UI graphics intact, and its USB menu shows the drive including the `CSV` folder. Then reflash our app.
- [ ] **Step 6: Record and commit.** Add the results to `NOTES.md` and change the spec status line to mark sub-project 2 done. Commit both:
```bash
git add NOTES.md docs/superpowers/specs/2026-11-13-dso510-firmware-design.md
git commit -m "Notes: sub-project 2 verified on hardware; spec status

Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp"
```
