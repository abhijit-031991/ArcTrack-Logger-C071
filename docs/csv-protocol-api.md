# ArcTrack Logger - CSV Communication Protocol Specification (v4.0)

This document specifies the line-delimited **CSV + CRC-8 communication protocol** implemented in `src/main-ag.cpp` for the ArcTrack Logger (`STM32C071K8`).

---

## 1. Protocol Architecture & Rationale

Prior firmware revisions utilized mixed binary C structs (`struct reqPing`, `struct setttings`) and raw JSON payloads. This introduced critical operational vulnerabilities:
- **Packet framing misalignment**: A single dropped or noise byte permanently desynchronized the binary parser.
- **Architecture & padding hazards**: Variable compiler struct packing (`__packed__`) and endianness discrepancies caused corrupted parameter writes.
- **High transmission overhead**: Verbose JSON keys consumed excessive USB CDC bandwidth and flash read time during bulk telemetry offloading.

The **CSV + CRC-8 Protocol** resolves these issues by adopting an NMEA-inspired, streaming ASCII standard:
- **Resynchronization guarantee**: Every message begins with `$` and terminates with CRLF (`\r\n`). Any corrupted or partial frame is dropped without stalling future frames.
- **Integrity verification**: Every payload is verified by an 8-bit cyclic redundancy check (CRC-8) before dispatch or execution.
- **Bandwidth reduction**: Strips key-name overhead, reducing telemetry payload size by ~45% compared to JSON.
- **Human-readable & machine-parseable**: Can be monitored directly in any standard serial terminal or parsed by Python/web client apps.

---

## 2. Physical & Transport Layer

| Parameter | Specification |
| :--- | :--- |
| **Physical Interface** | USB 2.0 Full-Speed (12 Mbps) Virtual COM Port (CDC-ACM) |
| **Baud Rate** | 115,200 baud |
| **Data Framing** | 8 Data Bits, No Parity, 1 Stop Bit (`8-N-1`) |
| **Line Termination** | Carriage Return + Line Feed (`\r\n`, ASCII `0x0D 0x0A`) |
| **Max Frame Length** | 96 bytes (including delimiters and checksum) |
| **Default Tag ID** | `10606` |

---

## 3. Frame Syntax & CRC-8 Algorithm

### Frame Format
```text
$<VERB>,<FIELD_1>,<FIELD_2>,...,<FIELD_N>*<CRC8>\r\n
```

- **`$`** (ASCII `0x24`): Frame start delimiter. Instantly resets the receiver parser buffer.
- **`<VERB>`**: 3-character uppercase command identifier (`SET`, `CMD`, `CLR`, `DFU`, `MSG`, `SUM`, `MEM`, `DAT`).
- **`,`** (ASCII `0x2C`): Comma separator between fields.
- **`*`** (ASCII `0x2A`): Checksum prefix. Marks the end of payload data.
- **`<CRC8>`**: 2-digit uppercase hexadecimal representation of the calculated CRC-8.
- **`\r\n`** (ASCII `0x0D 0x0A`): End-of-frame terminator.

### CRC-8 Algorithm Specification
- **Polynomial**: `0x31` ($x^8 + x^5 + x^4 + 1$ — Dallas/Maxim 1-Wire)
- **Initial Value**: `0x00`
- **Calculation Span**: Calculated across all characters strictly **between `$` and `*`** (exclusive of `$` and `*`).

#### Python Reference Implementation
```python
def crc8(data: str) -> int:
    crc = 0
    for byte in data.encode("ascii"):
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x31) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def format_frame(payload: str) -> str:
    c = crc8(payload)
    return f"${payload}*{c:02X}\r\n"
```

#### C/C++ Reference Implementation
```c
uint8_t crc8(const char* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint8_t)data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80) crc = (crc << 1) ^ 0x31;
      else            crc <<= 1;
    }
  }
  return crc;
}
```

---

## 4. Inbound Commands (Host → Device)

### 4.1 Update Settings (`$SET`)
Configures the device acquisition parameters and persists them to SPI flash metadata.

```text
$SET,<tag>,<gpsFrequency>,<gpsTimeout>,<gpsHdop>,<minSat>*<CRC8>\r\n
```

| Field | Type | Safe Bounds | Description |
| :--- | :--- | :--- | :--- |
| `tag` | `uint16` | Device Tag ID | Must match device tag (`10606`). Rejects mismatch. |
| `gpsFrequency` | `int` | `1` to `1440` | Fix interval in minutes (`1` min to `24` hours). |
| `gpsTimeout` | `int` | `30` to `300` | Max seconds to wait for a GPS lock before timing out. |
| `gpsHdop` | `int` | `2` to `20` | Maximum acceptable HDOP (lower = stricter precision). |
| `minSat` | `int` | `3` to `12` | Minimum satellites required to validate fix. |

- **Success Response**: Device echoes the confirmed settings: `$SET,<tag>,<frq>,<tout>,<hdop>,<minsat>*<CRC8>`
- **Error Response**: `$MSG,<tag>,42*...` (`SETTINGS_UPDATE_ERROR`)

---

### 4.2 General Commands (`$CMD`)
Issues action opcodes to the device during the calibration window.

```text
$CMD,<tag>,<code>*<CRC8>\r\n
```

| Opcode (`code`) | Name | Description | Response / Action |
| :---: | :--- | :--- | :--- |
| `3` | `FINISH_CALIBRATION` | Conclude bench session and enter field logging loop. | Shuts down USB clocks and enters sleep. |
| `5` | `CALIBRATION_SUMMARY`| Request current fix, battery, and storage metrics. | `$SUM,...` |
| `30` | `ABORT_CALIBRATION` | Abort bench session immediately. | `$MSG,<tag>,31*...` (`CALIBRATION_ABORTED`) |
| `40` | `REQUEST_SETTINGS` | Request active persisted logging configuration. | `$SET,...` |
| `43` | `SETTINGS_RESET` | Reset settings to factory defaults (`3, 60, 5, 4`).| Echoes restored `$SET,...` |
| `50` | `DATA_DOWNLOAD_NEW` | Stream records logged since previous download. | Streams `$DAT,...` surrounded by `$MSG` flags. |
| `51` | `DATA_DOWNLOAD_ALL` | Stream all historical records logged in flash. | Streams `$DAT,...` surrounded by `$MSG` flags. |
| `66` | `MEMORY_STATUS` | Query flash capacity, used/free bytes, and count. | `$MEM,...` |
| `201`| `REQ_SUMMARY` | On-demand query for calibration summary & hardware health. | `$SUM,...` (alias for opcode `5`) |

---

### 4.3 Memory Clear (`$CLR` — Guarded)
Clears storage log pointers and erases Sector 0 and Sector 1 to eliminate ghost record resurrection. Requires an explicit `CONFIRM` token.

```text
$CLR,<tag>,CONFIRM*<CRC8>\r\n
```

- **Success Response**: `$MSG,<tag>,64*...` (`MEMORY_CLEARED`)
- **Invalid Token / Error**: `$MSG,<tag>,65*...` (`MEMORY_CLEAR_ERROR`)

---

### 4.4 DFU Bootloader Reboot (`$DFU` — Guarded)
Commands the MCU to reboot directly into the internal STM32 ROM DFU USB bootloader for firmware updates. Requires an explicit `REBOOT` token.

```text
$DFU,<tag>,REBOOT*<CRC8>\r\n
```

- **Behavior**: Sets internal RAM retention flag `0xB00710AD`, dispatches `NVIC_SystemReset()`, and boots into System Memory bootloader.

---

## 5. Outbound Frames (Device → Host)

### 5.1 Status Message (`$MSG`)
Reports lifecycle events, self-test results, and operation flags.

```text
$MSG,<tag>,<code>*<CRC8>\r\n
```

| Code | Constant | Meaning |
| :---: | :--- | :--- |
| `1` | `CALIBRATION_BEGIN` | Calibration session opened (5-minute window active). |
| `2` | `CALIBRATION_END` | Initial hardware tests complete; waiting for host commands. |
| `10` | `GPS_CALIBRATION` | GPS module powered and attempting fix. |
| `11` | `GPS_CALIBRATION_END` | GPS calibration fix cycle completed. |
| `13` | `GPS_SUCCESS` | GPS fix successfully acquired with valid coordinates. |
| `14` | `GPS_CALIBRATING` | Heartbeat pulse sent every 3s while waiting for GPS lock. |
| `21` | `FLASH_DIAGNOSTICS` | SPI Flash JEDEC identification started. |
| `22` | `FLASH_DIAGNOSTICS_END`| Flash self-test finished. |
| `23` | `FLASH_ERROR` | SPI Flash did not respond or unknown chip ID. |
| `24` | `FLASH_SUCCESS` | SPI Flash verified (Winbond/Macronix JEDEC recognized). |
| `31` | `CALIBRATION_ABORTED` | Host requested calibration abort. |
| `41` | `SETTINGS_UPDATED` | Settings successfully written to flash metadata. |
| `42` | `SETTINGS_UPDATE_ERROR`| Settings payload rejected (invalid format or bounds). |
| `52` | `DATA_DOWNLOAD_END` | Data telemetry stream completed. |
| `54` | `DATA_DOWNLOAD_BEGIN` | Data telemetry stream starting. |
| `64` | `MEMORY_CLEARED` | Flash sectors invalidated; memory reset to empty. |
| `65` | `MEMORY_CLEAR_ERROR` | Clear aborted (missing `CONFIRM` token). |
| `71` | `ACCELEROMETER_OK` | LIS3DHTR accelerometer identified on I2C2 (`WHO_AM_I = 0x33`). |
| `72` | `ACCELEROMETER_ERROR` | LIS3DHTR accelerometer not responding on I2C2. |

---

### 5.2 Calibration Summary (`$SUM`)
Provides an on-demand snapshot of fix accuracy, battery voltage, storage consumption, charging status, and live hardware health.

```text
$SUM,<tag>,<haveFix>,<satellites>,<latitude>,<longitude>,<hdop>,<dataCount>,<usedPct>,<freePct>,<battVolts>,<chrgStat>,<flashOk>,<accelOk>*<CRC8>\r\n
```

**Field Descriptions**:
1. `tag`: Device ID (e.g. `10606`)
2. `haveFix`: `1` if fix is valid and fresh (<5s age), `0` otherwise
3. `satellites`: Number of locked satellites (e.g. `8`)
4. `latitude`: Latitude in decimal degrees, 6 decimal places (e.g. `37.774929`)
5. `longitude`: Longitude in decimal degrees, 6 decimal places (e.g. `-122.419416`)
6. `hdop`: Horizontal Dilution of Precision, 2 decimal places (e.g. `1.15`)
7. `dataCount`: Total historical log records written to device (e.g. `142`)
8. `usedPct`: Percentage of flash memory used (e.g. `1.12`)
9. `freePct`: Percentage of flash memory available (e.g. `98.88`)
10. `battVolts`: Settled battery voltage in Volts across 470k divider, 2 decimal places (e.g. `4.08`)
11. `chrgStat`: Charging complete status (`1` = Fully charged / Standby [TP4057 STDBY pin LOW], `0` = Charging in progress [TP4057 STDBY pin HIGH])
12. `flashOk`: SPI Flash diagnostic status (`1` = JEDEC ID verified & responding, `0` = Flash communication error)
13. `accelOk`: Accelerometer diagnostic status (`1` = LIS3DHTR verified on I2C2 [WHO_AM_I = 0x33], `0` = Accelerometer error)

---

### 5.3 Memory Status (`$MEM`)
Reports exact byte-level utilization of the circular storage log.

```text
$MEM,<tag>,<usedBytes>,<freeBytes>,<pctUsed>,<availableSlots>,<totalCount>*<CRC8>\r\n
```

**Field Descriptions**:
1. `tag`: Device ID (`10606`)
2. `usedBytes`: Total byte count of stored records (e.g. `4686`)
3. `freeBytes`: Remaining byte capacity in log ring (e.g. `4185522`)
4. `pctUsed`: Log space percentage utilized (e.g. `0.11`)
5. `availableSlots`: Records currently stored and readable (e.g. `142`)
6. `totalCount`: Monotonic lifetime record counter (e.g. `142`)

---

### 5.4 Data Record (`$DAT`)
Streamed sequentially during data offload (`DATA_DOWNLOAD_NEW` or `DATA_DOWNLOAD_ALL`).

```text
$DAT,<id>,<datetime>,<lat>,<lng>,<hdop>,<sats>,<locktime>,<count>,<x>,<y>,<z>*<CRC8>\r\n
```

| Field | Type | Format | Description |
| :--- | :--- | :--- | :--- |
| `id` | `uint16` | Integer | Device Tag ID (`10606`). |
| `datetime` | `uint32` | Unix Epoch | UTC timestamp in seconds (from GPS date/time). |
| `lat` | `float` | `%.6f` | Latitude in decimal degrees (e.g. `37.774929`). |
| `lng` | `float` | `%.6f` | Longitude in decimal degrees (e.g. `-122.419416`). |
| `hdop` | `float` | `%.2f` | Precision metric (`99.99` if no fix). |
| `sats` | `uint8` | Integer | Number of satellites used for fix (`0` if no fix). |
| `locktime` | `uint16`| Integer | Seconds elapsed to acquire fix. |
| `count` | `uint32`| Integer | Sequential record sequence number. |
| `x` | `float` | `%.3f` | LIS3DHTR X-axis acceleration in *g* (e.g. `0.012`). |
| `y` | `float` | `%.3f` | LIS3DHTR Y-axis acceleration in *g* (e.g. `-0.045`). |
| `z` | `float` | `%.3f` | LIS3DHTR Z-axis acceleration in *g* (e.g. `0.985`). |

---

## 6. Pre-computed Command Reference Table (Tag 10606)

These exact strings can be copied and pasted directly into the serial terminal at **115200 baud**:

| ASCII Command String | Command Name | Description |
| :--- | :--- | :--- |
| `$CMD,10606,5*2E` | **CALIBRATION_SUMMARY (5)** | Request fix, battery voltage, and memory usage (`$SUM`) |
| `$CMD,10606,201*14`| **REQ_SUMMARY (201)** | On-demand query for summary, battery, and hardware health (`$SUM`) |
| `$CMD,10606,40*A8` | **REQUEST_SETTINGS (40)** | Query current active configuration (`$SET`) |
| `$CMD,10606,66*D7` | **MEMORY_STATUS (66)** | Query memory capacity and record count (`$MEM`) |
| `$CMD,10606,50*5C` | **DATA_DOWNLOAD_NEW (50)** | Offload new records since last download (`$DAT`) |
| `$CMD,10606,51*6D` | **DATA_DOWNLOAD_ALL (51)** | Offload all historical records in memory (`$DAT`) |
| `$CMD,10606,43*FB` | **SETTINGS_RESET (43)** | Reset settings to factory defaults (`3, 60, 5, 4`) |
| `$SET,10606,3,60,5,4*9B` | **SET Default** | Set 3-min interval, 60s timeout, HDOP 5, 4 sats |
| `$SET,10606,1,45,5,4*6B` | **SET High-Cadence** | Set 1-min interval, 45s timeout, HDOP 5, 4 sats |
| `$SET,10606,15,60,3,6*02` | **SET High-Precision** | Set 15-min interval, 60s timeout, HDOP 3, 6 sats |
| `$CLR,10606,CONFIRM*A2` | **MEMORY_CLEAR (63)** | Erase log memory and reset pointers (Guarded) |
| `$DFU,10606,REBOOT*54` | **REQ_DFU (200)** | Reboot into STM32 ROM DFU Bootloader (Guarded) |
| `$CMD,10606,3*88` | **FINISH_CALIBRATION (3)** | Finish bench test and start field logging sleep |
| `$CMD,10606,30*06` | **ABORT_CALIBRATION (30)** | Abort calibration and begin logging immediately |

---

## 7. Example Session Traces

### 7.1 Boot & Calibration Phase
```text
(Device boots, USB enumerates)
Device -> $MSG,10606,1*A0       (CALIBRATION_BEGIN)
Device -> $MSG,10606,21*90      (FLASH_DIAGNOSTICS)
Device -> $MSG,10606,24*95      (FLASH_SUCCESS)
Device -> $MSG,10606,22*93      (FLASH_DIAGNOSTICS_END)
Device -> $MSG,10606,71*AF      (ACCELEROMETER_OK - I2C2 verified)
Device -> $MSG,10606,10*96      (GPS_CALIBRATION - GPS module powered)
Device -> $MSG,10606,2*A3       (CALIBRATION_END - Ready for commands)
Device -> $MSG,10606,14*92      (GPS_CALIBRATING heartbeat)
Device -> $MSG,10606,14*92      (GPS_CALIBRATING heartbeat)
Device -> $MSG,10606,13*95      (GPS_SUCCESS - Fix acquired)
Device -> $MSG,10606,11*97      (GPS_CALIBRATION_END)
Device -> $SUM,10606,1,7,37.774929,-122.419416,1.40,0,0.00,100.00,4.12,0,1,1*95 (Fix, battery, chrg, flash & acc)
```

### 7.2 Configuration Update
```text
Host   -> $SET,10606,5,45,4,5*8D
Device -> $SET,10606,5,45,4,5*8D (Echo confirms settings saved to flash metadata)
```

### 7.3 Data Download
```text
Host   -> $CMD,10606,50*5C
Device -> $MSG,10606,54*9A       (DATA_DOWNLOAD_BEGIN)
Device -> $DAT,10606,1727438400,37.774929,-122.419416,1.40,7,22,0,0.012,-0.045,0.985*2F
Device -> $DAT,10606,1727438700,37.774945,-122.419401,1.35,8,18,1,0.015,-0.041,0.988*1B
Device -> $MSG,10606,52*9C       (DATA_DOWNLOAD_END)
```

### 7.4 Transition to Field Deployment
```text
Host   -> $CMD,10606,3*88        (FINISH_CALIBRATION)
(Device cuts GPS power, disables USB clocks & HSI48, and enters 0.5 µA STOP mode)
```
