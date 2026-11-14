# DSO-510 replacement firmware — design

Date: 2026-11-13
Status: approved. Sub-project 1 (core scope) done and verified on hardware 2026-11-13 (merge 0c3fc14). Sub-project 2 (storage, CSV, USB drive) done and verified on hardware 2026-11-13 (stock-compat and Windows checks not run).

## Goal

Replace FNIRSI's DSO-510 application firmware with our own, keeping the stock feature set the
user relies on, and adding:

- **CSV export** of captured waveforms (replaces BMP screenshots, which are dropped).
- **Live USB streaming** to a PC, plus a **Python script** that plots voltage vs time live.
- **Scope and signal generator running at the same time.**

The FNIRSI IAP bootloader (0x08000000–0x08006FFF) is kept and never modified. Our app is linked at
0x08007000 and flashed by copying a `.bin` onto the bootloader's USB drive. Recovery is always:
hold the update button while powering on, copy the official `APP_DSO-510_V2.0.5_250114.bin`.

## Approach

Own firmware written from scratch (bare-metal C, `arm-none-eabi-gcc`, no vendor SDK), borrowing
from the stock firmware where that saves effort: register sequences, generator waveform formulas,
measurement behaviour, and the **factory calibration constants**. Binary-patching the stock
firmware was rejected: fragile, and it cannot deliver streaming or simultaneous scope + generator.

Hardware facts this design relies on are in `NOTES.md` (all confirmed on hardware unless noted).

## Sub-projects and order

Each sub-project gets its own spec section/plan and is flashed and verified on the device before
the next starts.

1. **Core scope** — detailed below.
2. **Storage + CSV** — SPI NOR flash (SPI3, CS PD12) + FatFs on the stock volume layout; long-press
   Run saves CSV; USB mass-storage mode to copy files off.
3. **Signal generator** — stock waveform set (sine, square/rectangular, sawtooth, half-wave,
   full-wave, step, anti-step, exp rise, exp fall, DC, multi-tone, sinc pulse, Lorentz),
   0–50 kHz, duty 0–100 %, amplitude 0.1–3.0 V; runs concurrently with the scope.
4. **USB serial streaming + Python live plotter** (`tools/`).
5. **Polish** — measurement set (12 stock measurements), persistence, roll mode for ≥100 ms/div,
   settings menu (brightness, sound, startup mode, auto-shutdown), battery icon, buzzer.

### CSV format (sub-project 2, decided now)

One file per capture. A `#`-prefixed header with settings, then one row per sample:

```
# DSO-510 custom firmware <version>
# sample_rate_hz=<n>
# volts_per_div=<v>
# coupling=<AC|DC>
# probe=<1|10>
# trigger=<mode>,<edge>,<level_v>
# trigger_index=<sample index of trigger point>
# triggered=<0|1>
# cal=offset_pwm=<n>,volts_per_code=<v>
time_s,volts,adc_code
<t>,<v>,<code>
...
```

`time_s` is relative to the trigger point (negative = pre-trigger). The whole record is saved,
not just the visible window. `adc_code` is the raw 8-bit value so volts can be re-derived.

## Code structure

```
fw/
  common/          drivers, one responsibility each
    board.[ch]     clocks, power latch, keys, backlight, battery ADC (exists)
    lcd.[ch]       ST7789 on XMC, text (exists)
    adc.[ch]       external ADC capture (single-DMA and 4-way interleaved modes)
    frontend.[ch]  range relays/gain pins, coupling, offset PWM
    cal.[ch]       read-only parser for the factory calibration page
    gen.[ch]       DAC1 generator (sub-project 3)
    spiflash, fatfs, usb   (sub-projects 2 and 4)
  core/            hardware-independent logic, unit-tested on the PC
    trigger.[ch]   trigger search on a sample buffer
    units.[ch]     ADC code <-> volts, sample index <-> time, timebase/V-div tables
    measure.[ch]   measurements (frequency, Vpp, duty, ... )
    csv.[ch]       CSV formatting (sub-project 2)
  app/             the firmware: UI state machine + screens, wires core/ to common/
  apps/            hardware test apps (hello, keytest, scopetest, caldump) — kept
tests/             host-side unit tests for fw/core (gcc, runs on the PC)
tools/             PC scripts (font generator, Python plotter later)
```

`core/` must not include any hardware header; that is what makes it testable on the PC.

## Sub-project 1: core scope

### Clocks

Same as stock: HEXT 8 MHz / 2 × 60 = 240 MHz SYSCLK; APB1 = APB2 = 120 MHz; timer clocks
240 MHz. Flash wait states as required by the AT32F403A at 240 MHz.

### Acquisition

- Timebase range 50 ns/div … 20 s/div in 1-2-5 steps; 30 px per division (300 px = 10 div).
- Each timebase maps to a sample rate (table, like stock `FUN_0801A95C`).
- **≤ 12 MS/s:** single stream — TMR4 CH3 PWM on PB8 clocks the ADC, TMR4 overflow requests
  DMA1 ch7 (flex request 0x4F) reading GPIOB->IDT byte-wide. Proven in `scopetest`.
- **24 / 48 MS/s (sub-project 1b, separate plan):** stock 4-way interleave — four phase-staggered timers (TMR8/5/3/1 in stock)
  pacing DMA2 ch1, DMA2 ch2, DMA1 ch3, DMA1 ch5 into four sub-buffers, re-interleaved in software.
  Exact timer phasing to be reproduced from stock `FUN_0801AD00`.
- Sub-project 1 targets 120 samples/div (rate = 120 / (s/div), capped at 12 MS/s), so timebases
  500 ns/div … 50 ms/div; below 30 samples/div the display connects samples with straight lines.
  50–200 ns/div arrive with 1b.
- **ADC clock runs continuously** between captures so the pipelined 3PA1030 never returns stale
  conversions (cause of the trigger-jump bug found in `scopetest`).
- **Record:** 4 screens of samples per capture (max 4 × 1200 = 4800 bytes), split pre/post trigger
  by the trigger-position setting. Fits easily in 96 KB SRAM, so the 224 KB option is not needed.
- Timebases ≥ 100 ms/div are deferred to roll mode (sub-project 5); until then the slowest
  supported timebase is 50 ms/div.

### Trigger

Software trigger over the captured buffer:

- Level (in volts, converted to ADC code via calibration), edge rising/falling.
- Hysteresis: arm when the signal is beyond the level by ≥ 1/8 of peak-to-peak (minimum 4
  codes), fire on crossing the level.
- **Auto:** if no trigger within ~2 screen-times, display an untriggered capture.
- **Normal:** display only triggered captures; otherwise keep the previous trace.
- **Single:** capture one triggered record, then STOP; Run re-arms.

### Front end and calibration

- Ranges 10 mV/div … 10 V/div (10 settings): range index r drives stock pin state r
  (PC4, PE6, PE5, PE4) and is r in the stock mV/div table @ 0x08034A2E
  `[10,20,50,100,200,500,1000,2000,5000,10000]`. Gain is nominal: 25 ADC codes per division,
  0 V at code 128 + offset (offset −100…+100 codes), as in stock `FUN_08015D08` display code.
- Coupling: PC5 (which level = AC is to be confirmed on hardware). Probe ×1/×10 is a display
  multiplier only.
- Offset: PA7 PWM (TMR14 CH1). Stock formula (from `FUN_08008834`):
  `pwm = (hi[r] - lo[r]) / K * (user_offset + 100) + lo[r]` with per-range tables.
- **Factory calibration** is stored by stock in internal flash page **0x08006000** (2 KB, below our
  app; the bootloader does not rewrite it). `cal` reads it **read-only**. First step: a `caldump`
  test app shows the raw page on screen to confirm it is present. Layout (from stock writer
  `FUN_0801B218`): word 0 low byte 0x55 (saved) / 0xAA (defaults); words 8..0x25 hold six
  10-entry u16 tables (bytes = RAM 0x20000202..0x20000279): offset-PWM lo/hi per range for three
  timebase classes (slow ≤ 2.4 MS/s, 12 MS/s class, interleaved). Offset PWM =
  lo + (hi − lo) · (offset + 100) / 200 (stock `FUN_08008834`, K = 200). Factory data covers
  offset only; gain is nominal.
- **We never erase or write page 0x08006000.** Our own settings live on the SPI flash
  (sub-project 2); until then, settings are not persisted.
- Fallback if the factory block is missing/invalid: zero-offset calibration via long-press Auto
  (probe shorted, as the manual instructs) and gain from the cal pad (3.3 V, 1 kHz, 50 %),
  held in RAM until sub-project 2 adds persistence.

### Screen

Stock-like layout, 320×240:

- Top bar: timebase, RUN/STOP, active control-mode icon, battery.
- Plot: 300×200 px, 10×8 div grid; markers for trigger level, trigger position, 0 V baseline.
- Bottom bar: V/div, trigger mode, probe ×1/×10, coupling; one measurement line
  (frequency, Vpp, duty+).
- Trace drawn column by column, each column erased and redrawn in one LCD window write
  (no full-screen clear, no flicker).

### Controls

| Key | Short press | Long press |
|---|---|---|
| Mode | cycle control mode: ① timebase / V-div, ② trigger position / vertical offset, ③ trigger position / trigger level | settings menu: coupling, probe, trigger mode, edge |
| Wheel ← → | horizontal item of current control mode | — |
| Wheel press | confirm in menus | — |
| Up / Down | vertical item of current control mode; menu navigation | — |
| Run | run / stop | save CSV (sub-project 2) |
| Auto | auto-set (V/div + timebase from signal) | zero-offset calibration |
| Power | — | power off (release PD3 latch); long press avoids accidental shutdown |

Every app build must implement power-off (lesson from `hello`).

### Error handling

- Capture DMA timeout → show an on-screen error, keep running (as `scopetest` does).
- Invalid calibration block → fall back to defaults + on-screen "uncalibrated" marker.
- Watchdog: reloaded in the main loop in case the bootloader enabled it.
- HardFault handler: show the fault on screen and wait for the power key, so a crash is visible
  rather than a frozen screen.

### Testing

- **Host unit tests** (`tests/`, built with the PC's gcc, run via `make test`): trigger search,
  units conversion, measurements, later CSV formatting. Written test-first.
- **On-device checks** per feature against known signals: cal pad (1 kHz, 50 %, 3.3 V) and the
  generator at synchronous and non-synchronous frequencies (`scopetest` already provides these).
  Each plan task lists what to look for on screen.

## Out of scope for sub-project 1

CSV/storage, USB, generator UI, measurements beyond the single line, persistence, roll mode,
settings menu beyond the trigger/coupling/probe items, language/themes.

## Sub-project 2: storage, CSV export, USB drive

Approved in conversation 2026-11-13. Hardware facts: `NOTES.md` § SPI NOR flash and § first USB contact.

### Storage

- **Flash:** Puya 8 MiB SPI NOR (JEDEC 0x856017) on SPI3 (PC10/11/12, CS PD12), mode 0, 4 KiB erase
  sectors, 256-byte program pages. Driver `fw/common/spiflash.[ch]`: read, 4 KiB sector erase, page
  program, busy polling with timeouts.
- **Write guard:** every erase/program address must be `< 0x500000` (the 5 MiB user volume), checked
  in the driver itself and unit-tested. The 3 MiB system volume at 0x500000 (stock UI graphics) is
  never written, so the stock firmware keeps working after a reflash.
- **File system:** FatFs (ChaN, vendored unmodified in `fw/third_party/fatfs/` with its licence),
  one volume = the existing FAT12 user volume at flash offset 0: 4096-byte sectors, 1280 sectors.
  No reformatting; stock folders (`LOGO`, `Screenshot file`, …) are left alone.
- **Block layer:** `disk_read/disk_write` map 4 KiB FAT sectors 1:1 onto flash erase sectors
  (write = erase + 16 page programs). Shared by FatFs and USB so both see the same bytes.

### CSV save (long-press Run)

- Freezes the current record, then writes `/CSV/CAP<nnnn>.CSV`, `nnnn` = 1 + highest existing number
  (folder created if missing; 9999 max → "Drive full").
- Format as in § CSV format. Values: `time_s` = (i − trigger_index) / sample_rate with 9 decimals,
  `volts` with 4 decimals (probe factor applied), `adc_code` 0–255. `cal=` line gives the offset PWM
  and V/div actually used. The whole record (4 screens, ≤ 4800 rows, ≈ 120 KB) is saved.
- `triggered=1`: a trigger was found at `trigger_index`. `triggered=0`: an untriggered Auto capture;
  `trigger_index` is then the screen trigger position (pre-trigger samples), so `time_s` = 0 marks
  where the trigger would sit on screen, not a level crossing.
- File data goes to FatFs in whole 4096-byte chunks (one FAT sector each; only the tail is shorter),
  with `FF_FS_TINY 0`, so the FAT is written once at close instead of once per chunk.
- The formatter lives in `fw/core/csv.[ch]` (no printf; host-tested), producing the file in chunks.
- Toasts (14 columns): " Saved CAPnnnn", " Drive full   ", " Save failed  " (any FatFs/flash error),
  " Busy, retry  " (host did not go idle). Target ≤ 2 s.
- A long Power press during the save waits still switches the scope off.
- Saving does not change the scope's run/stop state after it finishes.

### USB drive (auto-mount)

- USB FS device, Mass Storage Class / Bulk-Only / SCSI transparent, 64-byte bulk endpoints, 48 MHz
  USB clock = HICK 48 MHz trimmed to the host's SOF by the ACC (as stock). Interrupt driven (IRQ20):
  bus reset, EP0 (incl. SET_ADDRESS right after its status stage) and EP1 packets are handled in the
  ISR; 4 KiB block reads/writes run in main context. Exposes the 5 MiB user volume only, as 1280 × 4096-byte blocks (as stock),
  removable (RMB=1). Own VID:PID strings are not required; reuse stock `2e3c:5720`.
- **Auto-mount:** the device is enabled with the D+ pull-up on except during a save's soft-disconnect and at power-off. A computer enumerates it; a
  charger never does, so nothing happens there. The scope keeps running normally while connected.
- **Ownership while connected:** the host owns the volume (read/write). The scope's FatFs volume is
  unmounted while the host is attached.
- **Save while connected (soft-disconnect):** if a host has configured the device (also while the bus
  is suspended): wait until the host has no command in progress and has issued no WRITE for ≥ 1 s
  (max 5 s, else " Busy, retry  "; the wait ends early if the host goes away). Then report MEDIUM
  NOT PRESENT (02/3A/00; a WRITE still in progress fails and the rest of its data is discarded),
  switch the D+ pull-up off (USB_CFG bit 1) for ≥ 0.5 s, mount FatFs, write the file, unmount,
  report the medium present again (one UNIT ATTENTION 06/28/00) and switch D+ back on. The host
  sees a real unplug + re-plug, drops its mount and FAT cache, re-enumerates and mounts afresh; this
  is the only mechanism both Linux and Windows honour without depending on their TUR poll interval.
  Cost: an OS "device disconnected" notification per save while plugged in.
- Without a configured host the save only brackets the write with medium not present / present.
- Power-off switches the D+ pull-up off first, so a host on USB power sees an unplug, not a dead drive.
- Detach (USB suspend or no SOF for 1 s) returns ownership to the scope.
- Linux is the primary test host; Windows behaviour is designed for (stock used the same 4 KiB block
  geometry with Windows) and checked when available.

### Testing

- Host tests: CSV formatter (header, rounding, negative times, chunking), next-file-number logic,
  SCSI command handler against a RAM disk (INQUIRY, READ CAPACITY, READ/WRITE(10), TEST UNIT READY,
  REQUEST SENSE, MODE SENSE, PREVENT/ALLOW, eject/re-insert sense sequence), flash write guard.
- On device: save several captures; files open and plot; auto-mount on Linux; save while mounted shows
  the new file; plain charger does nothing; stock firmware still boots and shows its files after
  reflashing it.

### Out of scope for sub-project 2

Live streaming (sub-project 4), generator UI (3), screenshots (dropped), deleting files on the scope.

## Sub-project 4: live USB streaming + Python plotter

Approved in conversation 2026-11-13 (built before sub-project 3; the two are independent).

### USB: composite drive + serial port

- The device becomes a composite: the existing MSC drive (interface 0, bulk 0x81/0x01, unchanged) plus a
  CDC-ACM serial port (communication interface with a notification interrupt IN endpoint, data interface
  with one bulk IN and one bulk OUT endpoint, 64-byte packets), grouped with an Interface Association
  Descriptor. Same VID:PID 2e3c:5720. Native drivers on Linux (`/dev/ttyACMn`), Windows 10+ (`COMn`) and
  macOS (`/dev/cu.usbmodem*`); no driver install.
- CDC class requests: SET_LINE_CODING / GET_LINE_CODING (accepted and echoed, baud rate ignored) and
  SET_CONTROL_LINE_STATE. DTR set = a host program has the port open; DTR cleared or USB detach = closed.
- The existing interrupt-driven driver serves both functions; drive and serial share the bus bandwidth.

### Protocol

Scope → PC: binary packets `A5 5A | type u8 | len u16 LE | payload[len] | crc16 LE`, CRC-16/CCITT-FALSE
(poly 0x1021, init 0xFFFF) over type, len and payload. The receiver resynchronises on the next `A5 5A`
after a bad CRC. Multi-byte payload fields are little-endian.

| Type | Name | Payload |
|---|---|---|
| 0x01 | INFO | ASCII `key=value` lines: `fw=<version>`, `mode=<idle\|frames\|log>`, `rates=<comma list>` |
| 0x02 | FRAME | meta (below) + `n u16` + n raw ADC codes |
| 0x03 | LOGMETA | `rate_hz u32`, `vdiv_mv u16`, `probe_x10 u8`, `coupling_dc u8`, `zero_code i16`, `offset_pwm u16` |
| 0x04 | LOG | `first_index u32` (running sample counter since LOG start) + raw ADC codes |
| 0x05 | DROP | `first_index u32`, `count u32`: samples lost before index `first_index` |
| 0x06 | OK | ASCII echo of the accepted command |
| 0x07 | ERR | ASCII reason |

FRAME meta = the CSV metadata: `rate_hz u32, vdiv_mv u16, coupling_dc u8, probe_x10 u8, trig_mode u8,
edge u8, triggered u8, zero_code i16, trig_level_code i16, trig_index i32, offset_pwm u16`. Volts are
derived exactly as in the CSV (µV/code = vdiv_mv × 40 × probe factor; volts = (code − zero_code) × µV/code).

PC → scope: ASCII lines terminated by `\n` (max 32 chars): `INFO?`, `FRAMES`, `LOG <rate_hz>`, `STOP`.
Each command gets OK or ERR (unknown command, unsupported rate, or scope busy, e.g. mid auto-set).

### Frames mode

- The scope behaves exactly as without streaming (screen, keys, auto-set, CSV save).
- After `process_capture` accepts a capture and the port is open in frames mode, the record is queued as a
  FRAME. If the previous frame is still being sent, the new one is skipped; streaming never blocks or slows
  capture. The top bar shows `USB` while frames are streaming.

### Logger mode

- Rates (S/s): 1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 500000. Gap-free target up to
  200 kS/s; 500 kS/s is best effort.
- The ADC runs continuously into a DMA ring buffer (circular, half/full-transfer). Each completed half is
  sent as LOG packets. If USB cannot keep up, unsent data is discarded and a DROP packet records exactly how
  many samples were lost; gaps are never silent.
- Screen: status view with `LOGGING`, the rate, the samples sent, drops, and live min/max/mean volts.
- Keys while logging: Up/Down change V/div (a new LOGMETA is sent; the front-end settle wait applies), Run
  stops logging and returns to the scope, Power powers off. Trigger, auto-set and CSV save are unavailable.
- Logging stops on `STOP`, when the port is closed (DTR cleared), or when USB detaches.
- A CSV save with the drive mounted soft-disconnects USB (sub-project 2), which also drops the serial port;
  the host script reconnects and resumes.

### Python tool (`tools/`)

- `tools/dsoproto.py`: packet decoder (resync, CRC), FRAME/LOGMETA/LOG/DROP parsing, volts conversion, and
  CSV writers producing the same format as the scope's CSV. No GUI dependencies.
- `tools/scope_live.py`: pyqtgraph + PyQt5 GUI, pyserial. Finds the port by VID:PID (override `--port`).
  Plot: frames mode shows the latest frame (t = 0 at trigger); logger mode shows a scrolling window of the
  last N seconds with breaks at DROP gaps. Controls: mode (Frames/Logger), logger rate, Start/Stop,
  Record to CSV, window length. Status line: connection, frames/s or S/s, totals, drops. Auto-reconnects
  and resumes the previous mode.
- Recording: frames mode writes one CSV per frame (`frame_0001.csv`, …) in the scope's format; logger mode
  streams one CSV with a header from LOGMETA and `# gap=<n> samples at <index>` lines at drops.
- Dependencies: `pip install pyserial pyqtgraph PyQt5` (`tools/requirements.txt`).

### Testing

- Host (C): packet encoder + CRC, command parser, frame queue skip logic, logger ring/drop accounting,
  composite descriptors and CDC class requests.
- Python (pytest): decoder against shared golden test vectors (`tests/vectors/*.bin` produced by the C
  encoder test and committed), resync after corruption, DROP/gap handling, CSV output identical to the
  firmware CSV format.
- On device: drive + serial both enumerate; frames on the generator sine; logger at 10 kS/s and 200 kS/s for
  minutes with zero drops; 500 kS/s best effort; replug during streaming; drive copy while streaming.

### Out of scope for sub-project 4

Full remote control of scope settings (timebase, V/div, trigger) from the PC, streaming over anything but
USB, Windows/macOS-specific packaging.
