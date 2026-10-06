# ArcTrack Logger — Firmware Validation Test Plan

Exhaustive checklist to validate every subsystem before release. Nothing left to chance.

**Status legend:** ✅ passed · ⏳ pending · ⚠️ partial / needs re-test · ❌ failed
**Tools:** `tools/cmd.py <PORT> <code|set …>` (binary protocol), `reader.cpp` (standalone flash dump), `pio device monitor`, STM32CubeProgrammer (DFU/SWD), a DMM/µA meter, a bench PSU or battery, open-sky access.
**Device tag:** 10606. Reminder: the device only accepts commands during the **command window** (after boot, before `FINISH`/5-min idle); it drops USB in STOP, so re-read requires a **power-cycle**.

---

## A. Boot, clock, USB

| ID | Test | Method | Expected | Status |
|----|------|--------|----------|--------|
| A1 | Cold-boot enumeration | Power on 10× from cold | USB CDC (COM) enumerates 10/10 | ✅ |
| A2 | Boot with no host | Power on battery, no USB | Runs; no hang; logs after window | ✅ |
| A3 | 3 s startup delay | Open monitor immediately after boot | First output appears ~3 s in, nothing lost | ✅ |
| A4 | Clock = 24 MHz + HSI48/CRS | Confirm USB stable over a long session | No dropouts / re-enumeration | ✅ |
| A5 | Brown-out / low-batt boot | Boot at low battery voltage | Defined behavior (boots or clean halt, no lock-up) | ⏳ |

## B. Flash driver & record storage (W25Q)

| ID | Test | Method | Expected | Status |
|----|------|--------|----------|--------|
| B1 | Flash detected | Boot; watch `FLASH_SUCCESS(24)` | 24 emitted; JEDEC valid | ✅ |
| B2 | Flash absent handling | Boot with flash unpopulated | Halts cleanly (no false logging) | ⏳ |
| B3 | Record write + read back | Log records, download | Bytes match, fields sane | ✅ |
| B4 | Erase-on-demand at sector 0 | Fresh log, write records | Sector 0 erased once; writes verify | ✅ |
| B5 | Record spanning a sector boundary | Log > 110 records (4096/37) | Record crossing 4096 reads back intact | ⏳ |
| B6 | Log-full behaviour | Fill to `capacity − sector` | `MEMORY_FULL(61)`, no corruption, no wrap | ⏳ |
| B7 | Multi-sector growth | Log into sectors 1,2,3 | Each sector erased on entry; all records read | ⏳ |

## C. Metadata & persistence

| ID | Test | Method | Expected | Status |
|----|------|--------|----------|--------|
| C1 | Fresh chip → defaults | Erase chip, boot, `STATUS`+`REQUEST_SETTINGS` | freq3/timeout60/hdop5/minSat4, count0 | ✅ |
| C2 | Magic rejects stale/foreign meta | Flash with bumped magic | Settings reset to defaults | ✅ |
| C3 | Settings persist across power cycle | `set`, power-cycle, re-read | Same settings after reboot | ✅ |
| C4 | Write pointer = scan (no desync) | Log N, power-cycle, `STATUS` | `Used == Count × 37` every time | ✅ |
| C5 | Pointer recovery after power loss mid-log | Cut power during a log cycle, reboot | writeAddress recovered from data; no loss/desync | ⚠️ (scan validated; explicit mid-write cut pending) |
| C6 | Download pointer (readAddress) persists | `download new`, power-cycle, `download new` | Only records since last NEW; survives reboot | ⏳ |
| C7 | meta wear (long-run) | Extended logging | No meta-sector failure over expected life | ⏳ |

## D. Command session & protocol

| ID | Test | Method | Expected | Status |
|----|------|--------|----------|--------|
| D1 | Calibration handshake sequence | Boot, capture stream | `1→21→24/23→22→71/72→10→2` in order | ✅ |
| D2 | Non-blocking window | Send commands while GPS searching | Serviced immediately, not blocked | ✅ |
| D3 | GPS_CALIBRATING heartbeat | Watch during search | `14` every ~3 s until fix | ✅ |
| D4 | Auto summary on first fix | Acquire a fix in window | `13` + summary emitted | ⏳ |
| D5 | On-demand summary | `cmd 5` in window | Live summary (Fix/Sats/…) returned | ⏳ |
| D6 | Idle timeout closes window | Idle 5 min | Window closes, logging starts | ⏳ |
| D7 | Activity resets idle timer | Send a cmd every ~4 min for 15 min | Window stays open | ⏳ |
| D8 | `FINISH` closes window | `cmd 3` | Logging starts immediately | ✅ |
| D9 | Tag guard | Send frame with wrong tag | Ignored (settings → `42`) | ⏳ |
| D10 | Length dispatch (ping vs settings) | Send 3-byte and 31-byte frames | Correct branch each; settings reachable | ✅ |
| D11 | Malformed / partial frame | Send 1–2 stray bytes, or junk | No crash; no false action | ⏳ |
| D12 | CRC/robustness (N/A now) | — | Protocol is binary+tag-guard, no CRC | n/a |

## E. GPS

| ID | Test | Method | Expected | Status |
|----|------|--------|----------|--------|
| E1 | Position fix (open sky) | Log outdoors | Valid lat/lng, HDOP < threshold | ✅ (13.0397N/77.5617E) |
| E2 | Warm-start lock time | Fix after recent power | Locks in a few s (VBKUP) | ✅ (~3 s) |
| E3 | Cold-start lock time | First fix after long power-off / battery pull | Locks within `gpsTimeout` (may need 60–90 s) | ⏳ |
| E4 | `minSat` gate | Set minSat high (e.g. 12), log | Fix only when sats ≥ minSat | ⏳ |
| E5 | `hdop` gate | Set hdop strict (e.g. 2), log | Fix only when HDOP < threshold | ⏳ |
| E6 | `gpsTimeout` on no-fix | Cover antenna, log | Gives up at timeout, stores zeros, `LCKTM==timeout` | ✅ (15 s / 60 s seen) |
| E7 | Time-valid but no position | Poor sky | `DT` valid, lat/lng/sats = 0 | ✅ |
| E8 | Baud correctness | — | 9600 confirmed; parser reads GN/GP | ✅ |
| E9 | GPS power gating | Measure GPS rail | On only during acquire; off in sleep | ⏳ |

## F. Logging loop & cadence

| ID | Test | Method | Expected | Status |
|----|------|--------|----------|--------|
| F1 | True cadence | Log run, check `DT` spacing | Records ~`gpsFrequency` apart (not freq+acquire) | ✅ (181–182 s @ 3 min) |
| F2 | Cadence at other intervals | Set freq=1 and freq=5 | Spacing tracks the setting | ⚠️ (1 min ✅; others ⏳) |
| F3 | First record after window | Boot→window→log | First record ~immediately after window | ✅ |
| F4 | No-fix record stored | No-sky cycle | Zero record written, count advances | ✅ |
| F5 | Cadence accuracy over hours | Long run, compare `DT` to real time | Drift within LSI spec (~±5%), no creep | ⏳ |
| F6 | RTC seeded from GPS (if used) | — | Timestamps from GPS time | ✅ (DT from GPS) |

## G. Download

| ID | Test | Method | Expected | Status |
|----|------|--------|----------|--------|
| G1 | `DATA_DOWNLOAD_ALL` | `cmd 51` | `54` → all records → `52`; count matches | ✅ |
| G2 | `DATA_DOWNLOAD_NEW` | `cmd 50` twice | 1st returns new; 2nd returns none; pointer advances | ⏳ |
| G3 | Download count integrity | Compare records received vs `Count` | Equal, sequential CNT, no gaps | ✅ |
| G4 | Large download | Log 100s of records, download | Completes; watchdog fed; no truncation | ⏳ |
| G5 | Field formatting | Inspect JSON | 6-dp lat/lng, 2-dp hdop, SATS present | ✅ |

## H. Settings

| ID | Test | Method | Expected | Status |
|----|------|--------|----------|--------|
| H1 | Apply settings frame | `set 3 60 5 4` → `41` | Applied + persisted | ✅ |
| H2 | Clamp freq | `set 0 …` / `set 9999 …` | Clamped to 1 / 1440 | ⏳ |
| H3 | Clamp timeout | `set _ 1 _ _` / `set _ 999 _ _` | Clamped to 10 / 300 | ⏳ |
| H4 | Clamp hdop | out-of-range hdop | Clamped to 1 / 20 | ⏳ |
| H5 | Clamp minSat | out-of-range minSat | Clamped to 3 / 32 | ⏳ |
| H6 | minSat affects acquire | H5 then log | Uses new minSat in fix criteria | ⏳ |
| H7 | REQUEST_SETTINGS reply | `cmd 40` | Returns current settings (⚠ currently only `41`) | ❌ (stub — needs real reply) |

## I. Sleep, power, watchdog

| ID | Test | Method | Expected | Status |
|----|------|--------|----------|--------|
| I1 | STOP entered between logs | µA meter during sleep | Drops to STOP-mode current (~µA) | ⏳ |
| I2 | STOP wake on RTC alarm | Observe it wakes each cycle | Wakes reliably every second; logs on schedule | ✅ (implied by cadence) |
| I3 | Sleep current figure | Measure STOP current | Meets battery-life budget (record value) | ⏳ |
| I4 | Active current (acquire) | Measure during GPS on | Within expectation | ⏳ |
| I5 | IWDG kicks (normal) | Long run | No spurious resets | ✅ (no resets seen) |
| I6 | IWDG recovers a hang | Inject an infinite loop / stuck acquire | Resets within ~25 s, reboots clean | ⏳ |
| I7 | IWDG through STOP | Long sleep interval | No reset during legitimate sleep | ✅ (implied) |
| I8 | Clock restore after STOP | Post-wake USB/GPS work | Clock re-applied; peripherals OK | ✅ |

## J. Sensors

| ID | Test | Method | Expected | Status |
|----|------|--------|----------|--------|
| J1 | Accel WHO_AM_I | Fit LIS3DHTR, boot | `ACCELEROMETER_OK(71)` | ⏳ (no sensor fitted) |
| J2 | Accel x/y/z read | Log with sensor, tilt board | Sane g values, ~1 g magnitude at rest | ⏳ |
| J3 | Accel I2C address autodetect | Test 0x18 and 0x19 wiring | Detects either | ⏳ |
| J4 | Accel absent handling | No sensor | `72`, x/y/z = 0, no hang | ✅ |
| J5 | Battery voltage read | `set`→cal summary `BatteryV` | Matches DMM at BAT_SNS ×2 | ⏳ |
| J6 | SNS_EN gating | Measure divider node | Enabled only during read | ⏳ |
| J7 | Battery in record (`vbatt`) | Log, download | Non-zero mV, tracks battery | ⏳ |

## K. DFU & recovery

| ID | Test | Method | Expected | Status |
|----|------|--------|----------|--------|
| K1 | DFU via `REQ_DFU` | `cmd 200` in window | Reboots to ROM DFU (0483:DF11) | ⏳ |
| K2 | DFU via BOOT0 jumper | PA14 high + power-cycle | Enters DFU | ✅ |
| K3 | DFU via erase (SWD) | CubeProgrammer erase | Blank chip → DFU | ✅ |
| K4 | Reflash cycle | Upload via DFU | Verifies, runs new image | ✅ |

## L. End-to-end / soak

| ID | Test | Method | Expected | Status |
|----|------|--------|----------|--------|
| L1 | Full field run | Deploy on battery, several hours, open sky | Continuous fixes at cadence, no gaps/resets | ⏳ |
| L2 | Download after soak | Read the full run | Complete, sequential, timestamps monotonic | ⏳ |
| L3 | Battery-life estimate | Log current profile × duty cycle | Meets target runtime | ⏳ |
| L4 | Power-cycle stress | Many power-cycles during logging | Never loses/desyncs the log pointer | ⚠️ (several done ✅; formal stress ⏳) |
| L5 | dataCount > 65535 | (design) Log past 65535 records | Known uint16 cap — decide wrap/behaviour | ❌ (known limitation) |

---

## Priority gaps to close first
1. **J1–J3, J5, J7** — accelerometer + battery once the LIS3DHTR is fitted (only untested subsystems).
2. **I1/I3/I6** — STOP-mode current (battery life) + a real watchdog-hang recovery test.
3. **G2/C6** — download-NEW pointer behaviour across reboots.
4. **B6/L5** — log-full and dataCount-overflow policy (need a decision, not just a test).
5. **H7** — `REQUEST_SETTINGS` real reply (currently a stub).
6. **E3/E4/E5** — cold-start TTFF and the minSat/hdop gates.
