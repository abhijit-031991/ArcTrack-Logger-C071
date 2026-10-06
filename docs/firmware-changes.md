# ArcTrack Logger — Firmware Changes (AVR → STM32C0 Port)

**Target:** STM32C071K8 (Cortex-M0+, crystal-less USB, W25Q64 SPI flash)
**Last updated:** 2026-09-24

Reference for porting the AVR logger (`src/sample.cpp`) to the STM32C0. Tracks every
subsystem change, the original workflow, bugs to fix, open decisions, and build order.
Status: ✅ done · ⏳ pending · 🔲 decision needed.

---

## Current state
- **Toolchain / board / clock:** ✅ done — custom board JSON, project ldscript, **24 MHz core**
  (cold-boot-stable; see hardware-changes.md), USB CDC, DFU flow.
- **Test harness:** `src/test.cpp` (**v0.8**) — USB CDC console, DFU re-entry (`b`),
  charge status (`c`), baud benchmark (`r`), **SPI flash R/W/E test (`f`)**. `build_src_filter`
  currently scopes the build to `test.cpp`; real code goes into a new `main.cpp` later.
- **SPI flash driver:** ✅ done — see §2.

---

## Subsystem migration plan

### 1. Build / framework / clock — ✅ done
Custom board, ldscript, 24 MHz clock, USB CDC, DFU. Remaining: add `lib_deps`, retire
`build_src_filter = +<test.cpp>`, add real `main.cpp` (keep `test.cpp` as harness).

### 2. SPI flash driver (W25Q64) — ✅ done (replaces SPIMemory)
- **Driver:** `lib/LoRaE5_SPIFlash` (portable Arduino, no WL-specific code; already
  hardened incl. STM32 little-endian fix). **Bench-validated on hardware (v0.8):**
  auto-detects W25Q64 (8 MB), sector erase, page-aware `writeArray`, and
  `writeStruct`/`readStruct` all PASS.
- Construct with **SPI1** (`&flashSPI` on PB5/PB4/PB3) + **CS PA15** @ **8 MHz**.
- `.cpp.bak` deleted. ⚠️ `eraseChip` blocks up to 120 s → keep IWDG timeout above the
  longest erase; favor **sector erases**.

### 3. Metadata persistence — ⏳ pending (replaces EEPROM)
STM32C0 has **no EEPROM**. `meta` (writeAddr, readAddr, count, settings) must move to:
reserved **W25Q sector** *(recommended — good endurance, we own the flash)* with a rolling
record + sector erase when full, **or** MCU-flash emulation. → **decision #1**.

### 4. Timekeeping + wake — ⏳ pending (replaces RTC PIT + TimeLib)
Replace AVR 1 Hz RTC PIT / `rtcCounter` accumulator with **STM32RTC**: seed calendar from
GPS, timestamp records from RTC, **wake via an RTC alarm set to the full interval** (one
wake per cycle, not a 1 s tick). Removes `RTC_init()`, `ISR(RTC_PIT_vect)`, TimeLib. → **decision #5**.

### 5. Sleep mode — ⏳ pending (replaces AVR PWR_DOWN)
**STM32LowPower → STOP mode** between logs, woken by the RTC alarm. Re-init peripherals
gated by STOP on wake. Target ~µA in STOP.

### 6. Wake-into-command mode — 🔲 decision needed (new access model)
Today, command/download is **only** reachable in the 3-min boot window. Add a wake source
to enter command mode anytime: **VBUS-sense EXTI** (divide 5 V→GPIO) and/or **Hall/magnet**
input → wake from STOP → bring up USB CDC → run command handler → timeout back to logging.
→ **decision #2**.

### 7. App comms — ⏳ pending (UART → USB CDC; GPS on hardware UART)
App link = **USB CDC** (done). **GPS on a hardware UART** — USART1 (PB6/7) or USART2 (PA2/3).
Drop `SoftwareSerial`. → **decision #3**.

### 8. GPS parsing — ⏳ pending (TinyGPS++ ports unchanged)
Keep TinyGPS++; feed from the chosen UART. GPS power via **GPS_EN (PA4)** (was `GPS_PIN`).

### 9. ADC / battery + charge status — ⏳ pending
`analogRead(BAT_SNS=PB1)` at 12-bit; the 470 kΩ divider is high-impedance → **long ADC
sample time** (HAL sampling time / rely on C3). Gate divider with **SNS_EN (PA5)** during
measurement. Fold in **CHRG_STAT (PB2)** charge status (already built in harness). Verify
PB1 maps to an ADC channel.

### 10. Libraries / `lib_deps` — ⏳ pending
- **Add:** `TinyGPSPlus`, `ArduinoJson`, `STM32duino LowPower`, `STM32duino RTC`
  (flash = local `lib/LoRaE5_SPIFlash`).
- **Remove:** `SPIMemory`, `EEPROM`, `SoftwareSerial`, `avr/*`, `TimeLib` (if RTC replaces it),
  `elapsedMillis` (optional).

### 11. Watchdog — ⏳ pending (avr/wdt → IWDG)
STM32duino **IWatchdog**; kick in the main loop and during GPS acquisition. Timeout >
`gpsTimeout` and > longest flash erase.

### 12. Interrupts / ISRs — ⏳ pending
AVR `ISR()` / `sei()` → STM32 HAL/Arduino callbacks (RTC alarm callback, `attachInterrupt`
for VBUS/magnet EXTI).

### 13. Data record struct — ⏳ pending (freeze layout, fixed-width)
Pin every `data` field to fixed-width (`uint32_t/int32_t/float`); **freeze record size**
before writing the first record. Fix `FLASH_CAPACITY` (read from JEDEC — the driver already
reports 8 MB).

---

## Original firmware workflow (reference)

**Boot (`setup()`, once per power-up):**
1. Serial (9600 app) + Serial1 (115200 GPS) + SPI.
2. Pins; ADC 12-bit; 1 s delay; banner.
3. Flash init (power-cycle + `begin(MB(64))`); **halts forever if flash fails**.
4. `loadEepromSettings()` (or write defaults).
5. `deviceCalibration()` — flash + GPS self-test, JSON health summary.
6. `handleSerialCommands(3 min)` — the app command window.
7. `loadEepromSettings()` again; `RTC_init()`; enable sleep.

**Command window commands:** DATA_DOWNLOAD_ALL / _NEW, MEMORY_CLEAR / _STATUS,
REQUEST_SETTINGS, FINISH_CALIBRATION, plus a `settings` push (freq/timeout/hdop).

**Operational loop (`loop()`, forever):** 1 s RTC tick accumulator; every
`gpsFrequency × 60` s, wake → `recordGPSData()` (block for a fix up to `gpsTimeout`, build a
`data` record, append to flash, `dataCount++`, update EEPROM) → sleep again.

**Key constraint:** after boot the device **only logs and sleeps** — it never listens to
serial again. Downloading/settings requires a **reset + catch the 3-min window**. (This is
what the wake-into-command-mode change, §6, is meant to fix.)

---

## Bugs / issues in the original AVR code to fix during the port
- **Record portability:** `data.count` is `unsigned int` (2 B AVR / 4 B ARM) → changes the
  flash record layout across architectures. Use fixed-width types (§13).
- **`FLASH_CAPACITY` hard-coded 64 MB** but chip is **8 MB** (W25Q64) → read from JEDEC.
- **Integer-division storage %** in `deviceCalibration()`:
  `writeAddress/FLASH_CAPACITY * 100.0` = always 0. (`sendMemoryStatus()` does it right.)
- **`HDOP_AGE_THRESHOLD` is `uint8_t = 2000`** → overflows to 208 (unused, but wrong).
- **Serial protocol framing:** `handleSerialCommands` checks `>= sizeof(reqPing)` (3 B)
  before `sizeof(settings)` (larger) → the **settings branch is unreachable**. Needs a
  type/length-prefixed frame (redesign for USB CDC).
- **Flash CS:** original used `LCS`; on STM32 flash CS = **SPI1_NSS (PA15)**.
- **`getJEDECID()` into `uint16_t`** (SPIMemory) — moot; new driver reports full 32-bit ID
  + capacity.

---

## Open design decisions (resolve before further coding)
1. 🔲 **Metadata home:** reserved W25Q sector *(rec.)* vs MCU-flash emulation.
2. 🔲 **Wake-into-command model:** VBUS-sense EXTI, Hall/magnet, or both — and which pin(s).
3. 🔲 **GPS UART:** USART1 (PB6/7) or USART2 (PA2/3); what is the other reserved for (radio?).
4. ✅ **Flash driver:** RESOLVED → `LoRaE5_SPIFlash`.
5. 🔲 **Clock/time:** STM32RTC as sole clock (drop TimeLib)?

Also to scope (hinted but unimplemented in the sample): **accelerometer** (`x/y/z`,
`AINT1/2`, `AccelMetrics` lib), **radio/LoRa** (`radioFrq`, `longPing.mortality`),
**scheduling** (`startHour/endHour/scheduled`), **capacitance sensor** (`CAPACITANCE_*`,
`MAX_ELECTRODES`). Which are in scope now vs later?

---

## Build & validation order (each validated via the test harness first)
1. ✅ **Flash driver** — read/write/erase (done, v0.8).
2. **Metadata** in a W25Q sector — persistence across power cycles.
3. **STM32RTC** time + alarm — wake timing.
4. **STOP sleep** + wake — current draw + wake source.
5. **GPS** on the UART — real fix + record write.
6. **Command mode** (USB) + wake-to-command — download/settings.
7. **Watchdog + battery/charge**, then integrate into `main.cpp`.
