# DSO-510 replacement firmware — design

Date: 2026-11-13
Status: approved. Sub-project 1 (core scope) done and verified on hardware 2026-11-13 (merge 0c3fc14).

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
# cal=<gain>,<offset> (per-range constants used)
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
