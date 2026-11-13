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
- **8080 parallel LCD on XMC bank 1**: command @ 0x60000000, data @ 0x60020000 (RS = A16 on 16-bit bus).
- Framebuffer → LCD via DMA1 ch2 memory-to-memory, 16-bit, from 0x20007470.

Other:
- ADC1 ch13 (PC3) single conversion → **battery voltage** (FUN_08009408). PA8 = charge/power sense?
- **DAC** (+ TMR6 / TMR4) = DDS signal generator output (FUN_0800AF00, FUN_0801B80C, FUN_0801B8DC).
- **SPI3** = external SPI NOR flash driver (FUN_0802D944) → FatFs volumes 2: and 3:.
- **USBD** (USB FS device, IRQ20) = MSC drive mode.
- Internal flash unlock (KEY 0x45670123) present → settings/calibration stored in internal flash?
- PD2 polled during init (button?).

## Tooling

- Ghidra 12.2 at `~/tools/ghidra_extract/ghidra_12.2_PUBLIC`, portable JDK 25 at `~/tools/jdk/`.
- Re-run analysis: see `tools/ghidra/README.md`. Output: `re/decomp.c`, `re/functions.tsv`.

## Open questions / next steps
- Confirm part number + external ADC chip + LCD controller from board photos
- LCD controller init sequence (find writes to 0x60000000) → ST7789/ILI9341?
- Button GPIO map, front-end relays/gain control GPIOs, trigger implementation
- Exact 48 MS/s timer interleave scheme
- First custom app: blink/LCD fill at 0x08007000, flash via IAP (needs gcc-arm-none-eabi)
- Open case: identify AT32 part number, SPI flash, ADC, FPGA (if any), LCD controller, SWD pads
- Check flash access protection (FAP) via SWD / Artery ROM bootloader (BOOT0 → USB DFU)
