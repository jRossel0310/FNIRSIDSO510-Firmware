# FNIRSI DSO-510 reverse engineering notes

## 2026-11-13 — first USB contact (normal mode)

USB device (normal power-on, plugged into PC):
- VID:PID `2e3c:5720` — Artery's stock "AT32 Mass Storage" example IDs (unchanged by FNIRSI)
- Manufacturer "Artery", Product "AT32 Mass Storage", serial `0CFB9ECD5805`
- USB 2.0 full speed, 1 interface: MSC / SCSI / Bulk-Only, EP 0x81 IN + 0x01 OUT, 64-byte packets
- SCSI vendor "AT32", model "Disk0", rev 2.00
- => MCU is an **Artery AT32** (Cortex-M4). Exact part TBD (open case / read chip marking).

Storage volume:
- 5 MiB, 1280 × 4096-byte blocks, FAT12, no partition table, no label, UUID `6252-DB82`
- Almost certainly backed by an external SPI NOR flash (4 KiB erase sector = block size)
- Contents (copied to `dumps/storage/`):
  - `LOGO/` — empty. Probably a custom boot logo goes here (format TBD)
  - `Screenshot file/N.bmp` — 320×240, 16 bpp, top-down (height -240), 54-byte header.
    Header says BI_RGB (would mean RGB555) but pixels are really **RGB565** (raw framebuffer dump).
  - `Screenshot simple file/N.bin` — 610 bytes saved waveform:
    - 0x000–0x259: 301 × (max u8, min u8) pairs, centered on ~0x80 (one pair per screen column?)
    - 0x25A: `00 00` ?, `01 2D` = 301 (big-endian sample-pair count)
    - 0x25E: `12 34 56 78` end magic
  - `System Volume Information/` — written by a Windows PC (factory), dated 2024-07-02

Screen: 320×240 (status bar on top, 10×8 div grid, measurements + settings bar on bottom).

## 2026-11-13 — IAP (update) mode

- Same VID:PID `2e3c:5720`, but SCSI model "Msc iap", 8 MiB, 4095 × 2048-byte blocks, FAT, label `DSO-510`
- Root contains only an empty `Ready.TXT` (dated 2008-04-18) — Artery SDK stock MSC-IAP demo behavior
  (copy a .bin → bootloader flashes it → status file changes). Emulated FAT; no readback of flash seen.
- Bootloader lives at 0x08000000–0x08006FFF (28 KiB); we do NOT have it.
- Entered by holding a button at power-on → bootloader checks button before jumping to app,
  so a broken custom app is recoverable by re-flashing the official .bin.

## Official firmware: `APP_DSO-510_V2.0.5_250114.bin`

- 283,776 bytes, sha256 `0afd2b1153f66920edfe763f714e3455751ee4d326d6dc852082a8e5c9394d28`
- **Unencrypted, raw ARM Cortex-M image. Load address 0x08007000** (verified: default handler
  0x08007324 = `b .`, reset handler at 0x080072F0 is Keil/ARMCC startup → SystemInit, __main)
- Initial SP 0x200375D0 → >192 KiB SRAM in use → likely AT32F403A/407 (224 KiB SRAM mode) or AT32F435/437
- Reset handler also reads the SP word at 0x08000000 (bootloader's vector table)
- FatFs volumes:
  - `2:/` = user volume exposed over USB (LOGO, Screenshot file, Screenshot simple file)
  - `3:/System file/` = hidden volume with UI graphics as `N.jpg` + `999.bin` (not exposed over USB)
- **Boot logo: `2:/LOGO/LOGODSO-510.jpg`** — dropping a JPG there likely gives a custom splash (easy first test)
- UI strings in English, German, Spanish, Portuguese (+ probably Chinese in a non-ASCII encoding)

## Hardware map (from static analysis of V2.0.5)

MCU: **AT32F403A/407** (STM32F1-style peripheral map: GPIOB @ 0x40010C00, DAC @ 0x40007400;
uses >192 KiB SRAM). Artery "flexible DMA mapping" regs (DMA_SRC_SEL @ DMAx+0xA0/0xA4) used.

Clocks (main init @ 0x080290F8): HEXT 8 MHz /2 × 60 = **240 MHz SYSCLK**, APB1 = APB2 = 120 MHz
(timer clocks 240 MHz). VTOR set to 0x08007000 at start of main (SystemInit sets 0x08000000 first).

Acquisition (FUN_0801AD00, FUN_0801A95C, DMA ISRs IRQ12/15/17):
- **External 8-bit ADC on PB0–PB7.** DMA reads GPIOB->IDR (0x40010C08) byte-wide into RAM.
- 4 DMA channels (DMA2 ch1, DMA2 ch2, DMA1 ch3, DMA1 ch5), circular, 7525 (0x1D65) bytes each,
  into one buffer (ptr @ 0x20000280) at offsets 0, 0x1D65, 0x3ACA, 0x582F → 4-way interleave,
  stored de-interleaved (one quarter per channel).
- Paced by TMR8/TMR5/TMR3/TMR1 (ARR=3, PSC=0) started staggered; TMR4 (ARR=4) master/gate.
  240 MHz / 5 = 48 MHz matches the advertised 48 MS/s (exact scheme TBD).
- Alternate mode: single DMA1 ch7 (0x40020080), 30100 (0x7594) bytes, TMR4 CH3-paced.
- DMA1 ch5 transfer-complete ISR (IRQ15) stops TMR4 and sets capture-done flag (0x20000284).

Display:
- **ST7789** 320×240, 16-bit 8080 bus on XMC bank 1 NE1: command @ 0x6001FFFE, data @ 0x60020000
  (RS = A16 = PD11). Pins: PD0,1,4(NOE),5(NWE),7(NE1),8,9,10,11(A16),14,15 + PE7–PE15. IOMUX 0x40010030 |= 0x08000000.
- XMC: BK1CTRL1=0x5010 (|1 enable), BK1TMG1=0x02020424, BK1TMGWR1=0x202, 0xA0000220 low byte = 8.
- LCD reset = PD6 (high 10 ms, low 20 ms, high 20 ms). Init: SLPOUT, 120 ms, 36=00, 3A=05, B2 0C 0C 00 33 33,
  B7 35, BB 28, C0 2C, C2 01, C3 0B, C4 20, C6 0F, D0 A4 A1, E0/E1 gamma, DISPON, then MADCTL 36=60 (landscape).
- Framebuffer → LCD via DMA1 ch2 memory-to-memory, 16-bit, from 0x20007470.
- **Backlight = PA2, TMR9 CH1 PWM, 1 kHz** (ARR 99), duty = brightness setting (byte @ 0x200003B5); 0 on shutdown.

Power / misc GPIO:
- **PD3 = power latch — CONFIRMED** (keytest: driving PD3 low turns the scope off).
  Note: hello app had no power-off → had to unplug the battery. Every app must implement power-off.
- Keys (active low, pull-up; stock scan FUN_0802EF00, tables @ 0x080349E4/ED/F8, 0x08034A1C),
  physical mapping confirmed with keytest:
  K1 PD2 Power · K2 PC9 Run/Stop · K3 PC7 Up · K4 PC8 Down · K5 PC6 Mode · K6 PD13 Auto ·
  K7 PB15 wheel right · K8 PB14 wheel press · K9 PB13 wheel left.
  Long-press codes: K1=10 (power off) K2=11 K5=12 K6=13 K8=14 (70 scan ticks).
- Battery: V = raw × 3.3/4095 × 2 (½ divider on PC3). Stock % thresholds: ≥4.06 V 100, 3.98 90, 3.92 80,
  3.87 70, 3.82 60, then 3.79/3.77/3.74/3.68. Keytest read raw 2763 (= 4.45 V, likely with USB charging).
- Power-on: with battery connected, USB alone only charges (green charger LED); press K1 to boot. PA8 read alongside battery measurement (charger status?).
- **PA3 = cal pad — CONFIRMED** (TMR9 CH2 1 kHz 50 % square, seen on probe).
- PB9 = TMR11 CH1 2 kHz PWM, duty toggled → **buzzer**.
- PA7 = TMR14 CH1 12-bit PWM (ARR 0xFFE), updated often → front-end offset / trigger level DAC.
- USART1 TX on PA9 @ 1.5 Mbaud (debug console? something on the board?).
- SPI flash: SPI3 remapped to PC10/11/12, CS = PD12.

Other:
- ADC1 ch13 (PC3) single conversion → **battery voltage** (FUN_08009408). PA8 = charge/power sense?
- **Generator = DAC1 / PA4 — CONFIRMED** (1 kHz sine seen on probe). Stock: TMR6 TRGO triggers DAC1,
  DMA2 ch3 (flex req 0x05) streams 100-point table (RAM @ 0x200002EA) circular; f = TMR6 rate / 100.
- DAC2 / PA5 held at 0xF46 (~3.15 V) by stock init — purpose unknown (bias?).
- **ADC capture CONFIRMED** (scopetest): TMR4 CH3 PWM on PB8 = ADC clock, DMA1 ch7 (flex req 0x4F) on TMR4
  overflow reads GPIOB->IDT; trace tracks PA7 offset PWM; cal square + generator sine visible.
- **SPI3** = external SPI NOR flash driver (FUN_0802D944) → FatFs volumes 2: and 3:.
- **USBD** (USB FS device, IRQ20) = MSC drive mode.
- Internal flash unlock (KEY 0x45670123) present → settings/calibration stored in internal flash?
- PD2 polled during init (button?).

## Tooling

- Ghidra 12.2 at `~/tools/ghidra_extract/ghidra_12.2_PUBLIC`, portable JDK 25 at `~/tools/jdk/`.
- Re-run analysis: see `tools/ghidra/README.md`. Output: `re/decomp.c`, `re/functions.tsv`.

## Open questions / next steps
- Confirm part number + external ADC chip + LCD controller from board photos
- Button GPIO map, front-end relays/gain control GPIOs, trigger implementation
- Exact 48 MS/s timer interleave scheme
- First custom app: `fw/hello` (color bars + bouncing square). **2026-11-13: flashed via IAP, WORKS** —
  bootloader accepted `APP_DSO-510_HELLO.bin` (no signature/name check seen), LCD init + backlight (PA2) confirmed.
  IAP sequence: copy → USB disconnect → re-enumerate ~1 s → app starts (USB gone, app has no USB).
- Open case: identify AT32 part number, SPI flash, ADC, FPGA (if any), LCD controller, SWD pads
- Check flash access protection (FAP) via SWD / Artery ROM bootloader (BOOT0 → USB DFU)

## 2026-11-13 — trigger stability

- 3PA1030 is pipelined; stopping its clock between captures makes the first samples of the next
  capture stale (previous capture's data) → trigger fired at random phase (video kept locally, not in the repo).
  Fix: discard first 16 samples (scopetest) / keep ADC clock running (real firmware).
- Verified stable with generator sine at 100 Hz, 333 Hz, 1 kHz, 1.33 kHz, 5 kHz, 7.27 kHz,
  10 kHz, 20 kHz (incl. frequencies not synchronous with the sample clock).
- Factory calibration/settings page: internal flash 0x08006000 (written by stock FUN_0801B218).
  Never write it.

## Factory calibration page (caldump, 2026-11-13)

`parse: VALID factory data` (photo kept locally, not in the repo). Words 0x00..0x26 at 0x08006000:

```
00: 00060055 0B000100 01000100 00000000
04: 1E320C01 000003E8 00056402 00050105
08: 05E805E7 05ED05ED 05E805ED 05F105EE
0C: 05F205F2 05EB05E8 05ED05EC 05E805EE
10: 05F005EF 05F305F1 05EC05E8 05EC05EC
14: 05E805EE 05F105EE 05F205F2 09F209F4
18: 09F409F3 09EE09F4 09F609F4 09F809F8
1C: 09F209EE 09F409F3 09EE09F3 09F709F4
20: 09F709F8 09F009EE 09F209F2 09EC09F2
24: 09F509F1 09F509F6 09F50995
```

- Marker 0x55 (saved by stock). All offset-PWM `lo` ≈ 0x5E7–0x5F3, all `hi` ≈ 0x9EE–0x9F8 (hi > lo:
  larger PWM moves the trace up). Span ≈ 0x406 PWM counts for the ±100-code offset range.
- Page survived every IAP flash so far (bootloader only rewrites ≥ 0x08007000).

## Hardware checks for sub-project 1 (2026-11-13)

- faulttest: uptime counts 1/s (240 MHz + SysTick OK); RUN → HARD FAULT screen, PC 0x0800722E = the
  `udf` in main, CFSR 0x00010000 (UNDEFINSTR), HFSR 0x40000000 (FORCED) (photo kept locally, not in the repo);
  Power switches off. `tools/flash.sh` works.
- Scope app (sub-project 1) verified: boot, timebase sweep 500 ns–50 ms/div, V/div 10 mV–10 V, AC/DC coupling
  direction correct (FE_PC5_FOR_DC=1), trigger Auto/Normal/Single + edges, run/stop, power off,
  auto-set (sine + open input), auto-zero, power-off during auto-zero.
- **Offset PWM (PA7) filter settles slowly: tau ≈ 0.33 s.** A 100-code offset move needs ~2 s to settle to
  < 0.5 code. Once settled, the trace position matches the ideal model exactly
  (code = 128 + offset + V / (V/div) × 25). Any code that measures after an offset change must wait
  `fe_settle_ms()` (diagnostic photos: auto-set debug overlay, `make EXTRA=-DAUTOSET_DEBUG`).
- Generator test sine sits on ~+1.6 V DC (0.44–2.72 V at 1 kHz).
- Cal pad reads ~3.0–3.1 Vpp with nominal gain (pad amplitude not yet checked with a meter).


## SPI NOR flash (flashprobe, 2026-11-13, photo docs/storage.jpg)
- SPI3 remap IOMUX_REMAP5[27:24]=3 -> SCK PC10, MISO PC11, MOSI PC12; CS PD12; mode 0 @ 15 MHz works.
- JEDEC ID 0x856017: Puya (0x85), type 0x60, 2^0x17 = **8 MiB**.
- Two FAT12 volumes, no partition table, OEM "MSDOS5.0", 4096 B/sector, 1 sector/cluster, 1 reserved:
  - 0x000000: 0x500 sectors = **5 MiB user volume** (stock `2:`, the USB drive)
  - 0x500000: 0x300 sectors = **3 MiB hidden system volume** (stock `3:`, UI graphics) — must never be written
    by our firmware, so recovery to stock keeps working.
- Hardware result (2026-11-13): status registers SR1=00, SR2=00, SR3=20 (no block protection). A sector write
  (erase+program+verify) takes about 31 ms. CS timing is mandatory: CS must rise only after SPI3 BSY is clear
  (last bit fully clocked) and stay high >= ~250 ns; without that the chip ignored every erase and program.

## Hardware checks for sub-project 2 (2026-11-13)

- flashdiag: SPI NOR erase/program work only with the CS fix (BSY-wait + CS-high hold); sector write ≈ 31 ms.
- usbdisk / scope app: enumerates first try as `2e3c:5720`, 1280 × 4096-byte removable disk, vfat UUID 6252-DB82
  (same volume as stock). Full 5 MiB backup taken with dd (`dumps/userdrive-backup.img`, not committed).
- Long-press Run saves: CAP0001–0004 unplugged (3 untriggered zero captures `triggered=0`, 1 sine `triggered=1`);
  each file 4800 rows + 10 header lines (~110 KB). Sine file: 999.8 Hz over 8 cycles, 0.38–2.66 V (2.28 Vpp),
  sample at t=0 = 1.500 V = trigger level, 600 kS/s spacing — matches the screen.
- Save while mounted (CAP0005–0007): soft-disconnect → kernel "USB disconnect" → re-enumeration ~2 s later, files
  complete. The desktop did not auto-remount after the re-plug (host policy; mount via file manager or
  `udisksctl mount -b /dev/sdX`).
- `fsck.vfat -n`: clean apart from the dirty bit (expected after unplugging while mounted / soft-disconnect).
- File timestamps are the fixed FatFs date 2026-11-13 00:00 (no RTC); hosts show it shifted by their time zone.
- Not run (user's choice): stock-firmware compatibility check and Windows.

## Hardware checks for sub-project 4 (2026-11-13)

- Composite device enumerates on Linux first try: `usb-storage` (drive) + `cdc_acm` (`/dev/ttyACM0`), bcdDevice 3.00.
- Serial port access: udev rule `/etc/udev/rules.d/70-dso510.rules` (uaccess + `ID_MM_DEVICE_IGNORE`). With CDC protocol
  0x01 ModemManager probed the port (second reader -> garbled stream); protocol 0x00 + the rule fixed it.
- CDC IN throughput (cdctest BENCH): 863 kB/s, 0 CRC errors, 0 sequence gaps over 4.3 MB.
- Frames mode: 13–50 frames/s (4800 samples each, depends on timebase), 0 bad CRC.
- Logger mode after the incremental-push fix: 10k / 100k / 200k / 500k S/s all gap-free over 8 s each
  (0 lost, continuous sample index, 0 bad CRC; 500k delivers 496 kS/s). Before the fix ≥ 100 kS/s lost 12.5 %
  (whole ring half pushed into the 8 KiB TX FIFO at once).
- OK/ERR replies arrive even mid-log at 500 kS/s.
- `tools/scope_live.py` (venv `~/.venvs/dso`): user checked the live window ("looks good for now").
- Not run: long (minutes) logger soak, replug/save-during-stream robustness steps, Windows/macOS.

## Hardware checks for sub-project 3 (2026-11-13)

- Generator driver (`gendac.c`): DAC1/PA4 + TMR6 + DMA2 ch3 works first boot with TEN/DMAEN set while the DAC is
  already enabled (differs from the order in `gen_sine_init`; no problem on the AT32F403A).
- User ran the full list and reported "everything passed": power-on 1 kHz sine ~2.0 Vpp with the `G` marker; all nine
  waveforms; square duty 30/70 %; DC level via Offset; Output Off/On; 10 Hz / 1 kHz / 10 kHz / 33.3 kHz / 50 kHz;
  0.1 / 1.0 / 3.0 Vpp; quick `Gen` mode (wheel = frequency, Up/Down = amplitude, wheel press = fine `~`); CSV save
  and USB drive with the generator running; output stops at power-off.
- Streaming with the generator running (default 1 kHz sine, 500 mV/div): Frames 150 frames in 3 s, 0 bad CRC, codes
  77..178 around zero 55 -> 0.44..2.46 V (2.02 Vpp, centre 1.45 V). Logger 100 kS/s for 10 s: 968 packets
  (~991 k samples), no DROP packets, 0 bad CRC.
- Parked for sub-project 5: planner search cost near 50 kHz (est. ~10 ms, not measured); DC level and wave offset
  share one setting; quick mode with DC selected changes an amplitude that has no effect; wheel rotation on the
  Generator menu row opens the page and a further tick toggles Output; amplitude accuracy not checked with a meter.

## Interleaved capture experiments (ilvtest, 2026-11-13)

Bench app `fw/apps/ilvtest` (PC-driven over the serial port: register peek/poke, generator, captures).
Probe on the generator output, range 6, generator as signal source.
- Stock scheme decoded from `FUN_0801AD00`: TMR4 reload 4 (48 MHz ADC clock, CH3 compare 3), TRGO = update;
  TMR8/5/3/1 gated slaves (SMS 5; ITR2, ITR2, ITR3, ITR3), reload 3, start counts 3, 1, 2, 0; overflow DMA requests
  0x6F / 0x57 / 0x47 / 0x37 on DMA2 ch1 / DMA2 ch2 / DMA1 ch3 / DMA1 ch5.
- 48 MS/s works: stream delays (TMR3, TMR5, TMR1, TMR8) = 0, 0.98, 2.02, 3.04 ADC clocks (std 0.07), so the
  record order is TMR3, TMR5, TMR1, TMR8. Per-phase mean residual on a 10 kHz sine < 0.04 code.
- Keeping the ADC clock running and freezing the slaves through TMR4's TRGO (master mode 000 <-> 010) works as
  well as the stock stop-TMR4 start.
- With either start, the TMR8 stream (start count 3 = reload) is sometimes one sample early: 3/400 and 1/212
  captures from firmware, 7/25 in a slow poke-driven run. With reload preload (first period 8, start counts
  6, 5, 4, 3, then reload 3): 0/40 whole-sample slips.
- Every armed DMA channel immediately serves one stale request (count 257 -> 256 before the release).
- ADC clocked at 24 MHz (TMR4 reload 9) with four streams: stream delays inconsistent (-0.9, 0.2, -1.3 clocks).
  24 MS/s should use two streams of the 48 MHz scheme instead.

## Hardware checks for sub-project 1b (2026-11-14)

Scope app, probe on the generator output (50 kHz, 3.0 Vpp), checked over USB frames.
- All 19 timebases draw (user); 200 ns/div and faster are hard to read on this screen, as expected.
- 24 MS/s (5 us/div): 2238 frames, both streams aligned in every frame, no spikes.
- 48 MS/s (2 us/div): stream order 0, 1, 2, 3 in every frame; no whole-sample slips in ~5000 frames.
- **Misreads on the DMA2 streams (1 and 3):** before the spike filter about 1 frame in 3 had an isolated sample
  ~20 codes off, always at major code transitions (0x3F/0x40, 0x7F/0x80, 0xBF/0xC0; e.g. 0x40 read as 0x58), only
  on the two DMA2 streams. Bench (`ilvtest`, DC level parked on a boundary): ~1e-3 of reads wrong on those
  streams, ~1e-4 on the DMA1 streams. ADC clock phase (TMR4 CH3 compare 1..4): compare 1 improves two boundaries
  but not mid-scale and has a 20 % duty; compare 4 and inverted polarity break the capture. Kept the stock compare 3.
  Likely cause: the two DMA controllers contend for the bus on back-to-back reads. `ilv_despike` removes the
  isolated spikes: 0 in 3190 frames (12 M samples) afterwards.
- Residual: in ~1 % of 48 MS/s frames stream 1 or 3 reads one ADC clock (20.8 ns) late for part of the frame.
- 12 MS/s single stream (10 us/div): 1958 frames, no spikes.
- Logger 100 kS/s started from 2 us/div: no drops, 0 bad CRC; scope returns to 48 MS/s frames afterwards.
- Top bar: `G` marker now col 29, `USB` cols 31..33, uncalibrated mark is a red `!` at col 35 (user: "looks better").
- Parked for sub-project 5: trace stretch / trigger jitter at 50 ns/div; auto-set measures frequency only at
  12 MS/s; baseline settle at the 10/5 us/div calibration boundary.
- User checks (reported "all three have passed"): CSV save at 2 us/div, auto-zero with the fast calibration set
  ("ZERO done"), frequency reading with 50 kHz at 5 us/div.
