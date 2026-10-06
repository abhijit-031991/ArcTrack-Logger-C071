# ArcTrack Logger — Communication API

**Link:** USB CDC (virtual COM). Baud is nominal (USB FS sets the real rate).
**Device tag:** `10606` (`tag` in `definitions.h`). Every message carries it as `ID`.
**Message codes:** `src/codes.h`.

## Directions & encoding

- **Device → App: JSON lines.** Every message is one newline-terminated JSON object, most carrying a numeric `Msg` code + `ID`: `{"Msg":<code>,"ID":<tag>, ...}`. Data points and the summary omit `Msg` (identified by their fields). Floats are printed fixed-point (no `Serial.print(float)`), so there is no double math on the wire.
- **App → Device: binary structs, tag-guarded.** Two frames, disambiguated by length (the larger `setttings` is checked first):
  - `reqPing` — **3 bytes**: `{ uint16_t tag; uint8_t request; }`
  - `setttings` — **31 bytes** (ARM widths): `{ uint16_t tag; int gpsFrq; int gpsTout; int hdop; int minSat; int radioFrq; int startHour; int endHour; bool scheduled; }`
  A frame whose `tag` ≠ device tag is ignored (settings → `SETTINGS_UPDATE_ERROR`). `int` is 32-bit little-endian on the STM32; the app must pack to match. Firmware applies only `gpsFrq / gpsTout / hdop / minSat`; the rest are reserved/ignored.

## Boot / command session

The session runs once at boot (after a 3 s startup delay for USB enumeration):

```
CALIBRATION_BEGIN(1)
FLASH_DIAGNOSTICS(21) → FLASH_SUCCESS(24) | FLASH_ERROR(23) → FLASH_DIAGNOSTICS_END(22)
ACCELEROMETER_OK(71) | ACCELEROMETER_ERROR(72)
GPS_CALIBRATION(10)                 ← GPS powered, acquiring in the background
CALIBRATION_END(2)                  ← handshake done; interactive window opens
── interactive window (5-min inactivity) ──
   • GPS_CALIBRATING(14) every 3 s while searching
   • on first valid fix: GPS_SUCCESS(13) + GPS_CALIBRATION_END(11) + summary
   • app commands serviced anytime (see below); each resets the 5-min idle timer
   • closes on FINISH_CALIBRATION(3) or 5 min of silence
```

After the window the device logs and sleeps; it does **not** listen again until the next boot.

## App → Device commands (`reqPing.request`)

| Code | Name | Device does / replies |
|---|---|---|
| 51 | `DATA_DOWNLOAD_ALL` | `DATA_DOWNLOAD_BEGIN(54)` → data points → `DATA_DOWNLOAD_END(52)` |
| 50 | `DATA_DOWNLOAD_NEW` | same, from the download pointer; advances + persists it |
| 66 | `MEMORY_STATUS` | memory-status reply (below) |
| 40 | `REQUEST_SETTINGS` | reloads settings, replies `SETTINGS_UPDATED(41)` |
| 63 | `MEMORY_CLEAR` | resets pointers (lazy erase) → `MEMORY_CLEARED(64)` |
| 5  | `CALIBRATION_SUMMARY` | replies with the summary (below) — request any time |
| 3  | `FINISH_CALIBRATION` | closes the window, starts logging |
| 200| `REQ_DFU` | reboots into the ROM DFU bootloader (dev only) |

**Settings write:** send the 31-byte `setttings` frame → firmware clamps and applies, replies `SETTINGS_UPDATED(41)` (or `SETTINGS_UPDATE_ERROR(42)` on tag mismatch).

| Setting | Field | Range | Default |
|---|---|---|---|
| Fix interval (min) | `gpsFrq` | 1–1440 | 3 |
| Fix timeout (s) | `gpsTout` | 10–300 | 60 |
| Max HDOP | `hdop` | 1–20 | 5 |
| Min satellites | `minSat` | 3–32 | 4 |

## Device → App payloads

**Bare status:** `{"Msg":<code>,"ID":<tag>}`

**Memory status** (`Msg 66`): `{"Msg":66,"ID":tag,"Used":<bytes>,"Free":<bytes>,"Percent":<0-100>,"Count":<records>}`

**Calibration summary** (auto on first fix, and on `CALIBRATION_SUMMARY` request):
```
{"ID":tag,"Fix":0|1,"Sats":n,"LAT":d.dddddd,"LNG":d.dddddd,"HDOP":d.dd,
 "DataPoints":n,"StorageUsed":pct,"StorageFree":pct,"BatteryV":v.vv}
```
Reflects the **current** GPS state — poll it to watch the fix come in.

**Data point** (one per record during a download):
```
{"ID":tag,"DT":<epoch>,"LAT":d.dddddd,"LNG":d.dddddd,"HDOP":d.dd,
 "SATS":n,"LCKTM":<s>,"CNT":<index>,"X":g.ggg,"Y":g.ggg,"Z":g.ggg}
```
- `DT` UTC epoch of the fix · `SATS` satellites used · `LCKTM` seconds to lock · `CNT` record index · `X/Y/Z` accel in g (0 if no LIS3DHTR).

## Fix criteria

A fix is accepted only when: `satellites ≥ minSat`, `HDOP < hdop`, position age < 1 s, and ≥ 3 s since power-on. Same criteria for the background calibration acquire and the per-interval logging acquire.

## Message codes (from `codes.h`)

Device→App: `CALIBRATION_BEGIN 1`, `CALIBRATION_END 2`, `GPS_CALIBRATION 10`, `GPS_CALIBRATION_END 11`, `GPS_ERROR 12`, `GPS_SUCCESS 13`, `GPS_CALIBRATING 14`, `FLASH_DIAGNOSTICS 21/22`, `FLASH_ERROR 23`, `FLASH_SUCCESS 24`, `SETTINGS_UPDATED 41`, `SETTINGS_UPDATE_ERROR 42`, `DATA_DOWNLOAD_END 52`, `DATA_DOWNLOAD_ERROR 53`, `DATA_DOWNLOAD_BEGIN 54`, `MEMORY_FULL 61`, `MEMORY_CLEARED 64`, `MEMORY_STATUS 66`, `ACCELEROMETER_OK 71`, `ACCELEROMETER_ERROR 72`.
App→Device: `FINISH_CALIBRATION 3`, `CALIBRATION_SUMMARY 5`, `REQUEST_SETTINGS 40`, `DATA_DOWNLOAD_NEW 50`, `DATA_DOWNLOAD_ALL 51`, `MEMORY_CLEAR 63`, `MEMORY_STATUS 66`, `REQ_DFU 200`.

## Notes / gotchas

- **On-flash record is 37 bytes** (`data` struct); changing it invalidates stored records — clear memory after any record-layout change.
- **Wire widths:** `setttings` uses 32-bit `int` (ARM); confirm the app packs identically. `reqPing` is 3 bytes.
- **`REQUEST_SETTINGS` reply is just `SETTINGS_UPDATED`** (no dedicated settings-report payload) — legacy behavior carried over.
- Enter **DFU**: `REQ_DFU(200)` command, or BOOT0 (PA14) jumper + power-cycle, or erase via SWD.
