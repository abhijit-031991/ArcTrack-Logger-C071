# ArcTrack Logger — Hardware Changes & Fixes

**Board:** STM32C071K8 (crystal-less USB CDC) + TP4057 charger
**Last updated:** 2026-09-23

Running list of hardware issues found during USB bring-up and the fixes for them.
Grouped by priority. Check items off as they're implemented on the board / next spin.

---

## 🔴 Critical — blocking USB (fix first)

- [ ] **Add USB-C CC resistors** — `5.1 kΩ CC1 → GND` **and** `5.1 kΩ CC2 → GND`
  (two separate resistors, one per CC pin). Without them, no C-to-C host
  (phone, modern laptops) supplies VBUS or enumerates the device.
  *Root cause of all C-to-C failures.*
- [ ] **TP4057 PROG resistor (R1): 2.4 kΩ → 10 kΩ** (~500 mA → ~100 mA charge).
  Fixes PC rail-sag / "device malfunctioned", phone-OTG cutoff, and weak-host
  failures. (R1 also currently has no value set on the schematic.)
- [ ] **Add VBUS / +5 V input cap at TP4057 Vcc** — ~4.7–10 µF bulk.
  Missing on schematic; softens charge inrush.

## 🟠 Stability / decoupling (crystal-less USB needs a clean rail)

- [ ] **VDDA decoupling** — 100 nF + bulk on VDDA, cleanly tied to VDD.
  Directly stabilizes HSI48, the crystal-less USB clock.
- [ ] **VDD decoupling** — 100 nF per VDD pin + bulk near the MCU.
- [ ] **BAT-node bulk cap** — helps the load-on-BAT rail ride out transients.

## 🟡 Boot / programming access

- [ ] **Tie PA14 → GND (10 kΩ)** for deterministic app boot.
  (PA14 = BOOT0 = SWCLK; floating → random app-vs-DFU boot.)
- [ ] **Add a BOOT jumper** PA14 → 3.3 V for deliberate DFU entry.
- [ ] **Break out NRST** (currently inaccessible) — enables reliable
  connect-under-reset; important once firmware uses sleep/STOP.

## 🟢 Schematic placeholder values & verification

- [ ] **Set R2 (CHRG LED resistor)** — currently "R" (~1–4.7 kΩ).
- [ ] **Set BAT_SNS divider values (R6, R7)** — currently "R"; ratio so a full
  battery (4.2 V) maps below 3.3 V at the ADC.
- [ ] **Verify LDO EN (U3 MIC5504 pin 3) tied to VIN** (enable-high, not floating).
- [ ] **Verify TP4057 symbol pinout** against the datasheet (community symbols
  are often wrong).
- [ ] **Confirm BAT_SNS = PB1 is ADC-capable** on the C071K8.
  (Fixed the PB1/PB2 conflict: `BAT_SNS = PB1`, `CHRG_STAT = PB2`.)
- [ ] **Confirm USB data wiring** = `PA12 = D+`, `PA11 = D−`
  (first board had a USB fault here).

## ⚪ Build / mechanical (addressed on the current unit)

- [x] **Resolder the loose wire.**
- [x] **Replace the flaky slide switch.**

## 🔵 Design-level / future (optional, next spin)

- [ ] **Power-path charger** (MCP73871 / BQ2407x) — input-current-limit for
  USB/phone, regulated SYS rail, reliable charge status. Solves the
  load-on-BAT quirks wholesale.
- [ ] **Route TP4057 CHRG to a spare GPIO** — enables reliable 3-state charge
  status (Charging / Full / No-input); today only STDBY reaches the MCU.
- [ ] **VBUS-sense wake input** — VBUS → divider (5 V!) → EXTI/WKUP GPIO
  (e.g., PA0 / WKUP1) to wake from sleep on USB plug-in.

---

## Background notes

- **Why crystal-less USB is sensitive to the rail:** the C071 clocks USB from
  the internal HSI48 RC trimmed by CRS. Its frequency drifts with VDD, so a
  charge-current rail sag transiently detunes it → USB enumeration fails
  ("device malfunctioned"). A crystal-clocked part (e.g., STM32WB55) is immune
  to this, which is why the same charger layout worked there.
- **Why C-to-C fails but A-to-C sometimes works:** USB-A always sources 5 V
  VBUS (no CC needed), so A-to-C bypasses the missing CC resistors — it then
  succeeds only on hosts with stiff enough VBUS to survive the 500 mA charge
  draw. C-to-C requires the device Rd (5.1 kΩ) resistors to get VBUS at all.
- **The 2.4 kΩ / 500 mA charge current is the common thread** behind the PC
  instability, the phone non-enumeration, and the intermittent malfunction.
