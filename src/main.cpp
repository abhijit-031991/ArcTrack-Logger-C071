/*
 * main.cpp - ArcTrack Logger (STM32C071K8).
 *
 * A straight port of the AVR logger (sample.cpp) to the STM32C0, kept as a
 * single file with the same shape and the same app protocol. Only the pieces the
 * chip change forces are different from the original:
 *   - SystemClock_Config: crystal-less USB (HSI48 + CRS), 24 MHz core.
 *   - Flash: LoRaE5_SPIFlash driver (W25Q) instead of SPIMemory.
 *   - Settings/metadata: reserved top flash sector (no EEPROM on the C0).
 *   - Sleep/wake: STOP mode + RTC alarm 1 Hz tick (no AVR PWR_DOWN / RTC PIT).
 *   - Console: USB CDC (Serial) instead of a hardware UART.
 * Protocol matches sample.cpp: JSON out ({"Msg","ID",...}), small binary structs
 * in (reqPing / setttings), tag-guarded. JSON is hand-rolled (no ArduinoJson).
 */
#include <Arduino.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <IWatchdog.h>
#include "definitions.h"
#include "codes.h"
#include "LoRaE5_SPIFlash.h"

// ---- Library objects -------------------------------------------------------
TinyGPSPlus    gps;
SPIClass       flashSPI(SPI1_MOSI, SPI1_MISO, SPI1_SCK);
LoRaE5_SPIFlash flash(SPI1_NSS, &flashSPI, 8000000);      // W25Q @ 8 MHz
HardwareSerial gpsSerial(USART1_RX, USART1_TX);           // GPS on USART1 (PB6/PB7)

// ---- Configurable settings (loaded from flash metadata) --------------------
int gpsFrequency = 3;     // minutes between fixes
int gpsTimeout   = 60;    // seconds to wait for a fix
int gpsHdop      = 5;     // max HDOP (lower is better)
int minSat       = 4;     // minimum satellites required to accept a fix

// ---- Device state ----------------------------------------------------------
uint32_t writeAddress = 0;        // ring head (byte offset of next write)
uint32_t dlCount      = 0;        // records with count < dlCount already downloaded (NEW)
uint32_t dataCount    = 0;        // total records ever logged (monotonic CNT; never wraps in practice)
uint32_t metaAddr     = 0;        // reserved top sector (set in setup)
bool     sleeping     = false;
volatile uint32_t rtcCounter = 0; // 1 Hz tick accumulator
uint16_t lastLockS    = 0;        // seconds the last acquire took (-> record LCKTM)

// ---- LIS3DHTR accelerometer (I2C1: SDA=PB9, SCL=PB8) -----------------------
// Switch to I2C2_SDA/I2C2_SCL if the sensor is wired to I2C2 (PA6/PA7).
#define ACCEL_SDA I2C1_SDA
#define ACCEL_SCL I2C1_SCL
uint8_t accelAddr    = 0x18;       // resolved at init (0x18 or 0x19 by WHO_AM_I)
bool    accelPresent = false;

// ---- Constants -------------------------------------------------------------
const uint32_t CMD_IDLE_TIMEOUT_MS = 300000; // command window closes after 5 min of inactivity
const uint32_t GPS_BAUD            = 9600;   // GPS module baud
const uint32_t META_MAGIC          = 0x41544B32UL;  // "ATK2" - bumped to force settings back to defaults
const uint32_t WDG_TIMEOUT_US    = 25000000; // ~25 s independent watchdog (< 32 s max)

// ============================================================================
//  Clock (crystal-less USB): 24 MHz SYSCLK + HSI48/CRS
// ============================================================================
extern "C" void SystemClock_Config(void) {
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};
  RCC_PeriphCLKInitTypeDef pk = {0};

  osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
  osc.HSIState            = RCC_HSI_ON;
  osc.HSIDiv              = RCC_HSI_DIV2;          // 24 MHz
  osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  osc.HSI48State          = RCC_HSI48_ON;
  HAL_RCC_OscConfig(&osc);

  clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1;
  clk.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
  clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0);

  pk.PeriphClockSelection = RCC_PERIPHCLK_USB;
  pk.UsbClockSelection    = RCC_USBCLKSOURCE_HSI48;
  HAL_RCCEx_PeriphCLKConfig(&pk);

  __HAL_RCC_CRS_CLK_ENABLE();
  RCC_CRSInitTypeDef crs = {0};
  crs.Prescaler             = RCC_CRS_SYNC_DIV1;
  crs.Source                = RCC_CRS_SYNC_SOURCE_USB;
  crs.Polarity              = RCC_CRS_SYNC_POLARITY_RISING;
  crs.ReloadValue           = RCC_CRS_RELOADVALUE_DEFAULT;
  crs.ErrorLimitValue       = RCC_CRS_ERRORLIMIT_DEFAULT;
  crs.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;
  HAL_RCCEx_CRSConfig(&crs);
}

// ============================================================================
//  DFU re-entry (STM32 reflash path; the AVR used an ISP programmer)
// ============================================================================
#define DFU_BOOT_MAGIC 0xB00710ADUL
#define REQ_DFU        (byte)200      // dev-only reqPing.request: reboot to ROM DFU
static uint32_t bootFlag __attribute__((section(".noinit"), used));

static void jumpToBootloader() {
  __disable_irq(); HAL_RCC_DeInit(); HAL_DeInit();
  SysTick->CTRL = 0; SysTick->LOAD = 0; SysTick->VAL = 0;
  SCB->VTOR = 0x1FFF0000UL;
  __set_MSP(*(volatile uint32_t *)0x1FFF0000UL);
  ((void (*)(void)) * (volatile uint32_t *)0x1FFF0004UL)();
  while (1) { }
}

// ============================================================================
//  RTC 1 Hz tick + STOP sleep (replaces AVR RTC PIT + PWR_DOWN)
// ============================================================================
RTC_HandleTypeDef hrtc;

extern "C" void RTC_IRQHandler(void) {
  if (__HAL_RTC_ALARM_GET_FLAG(&hrtc, RTC_FLAG_ALRAF)) {
    __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);
    rtcCounter++;
  }
}

void RTC_init(void) {
  __HAL_RCC_LSI_ENABLE();
  while (!__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY)) { }
  RCC_PeriphCLKInitTypeDef pk = {0};
  pk.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  pk.RTCClockSelection    = RCC_RTCCLKSOURCE_LSI;
  HAL_RCCEx_PeriphCLKConfig(&pk);
  __HAL_RCC_RTC_ENABLE();
  __HAL_RCC_RTCAPB_CLK_ENABLE();

  hrtc.Instance          = RTC;                 // LSI ~32 kHz: 128*250 = 32000 -> 1 Hz
  hrtc.Init.HourFormat   = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv  = 249;
  hrtc.Init.OutPut       = RTC_OUTPUT_DISABLE;
  HAL_RTC_Init(&hrtc);

  RTC_AlarmTypeDef a = {0};                      // Alarm A masked -> matches every second
  a.Alarm              = RTC_ALARM_A;
  a.AlarmMask          = RTC_ALARMMASK_ALL;
  a.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  a.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  a.AlarmDateWeekDay    = 1;
  HAL_NVIC_SetPriority(RTC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(RTC_IRQn);
  HAL_RTC_SetAlarm_IT(&hrtc, &a, RTC_FORMAT_BIN);
  __HAL_RTC_ALARM_EXTI_ENABLE_IT();
}

// Sleep in STOP until the next 1 Hz alarm (USB drops for the duration).
void sleepOneTick(void) {
  NVIC_DisableIRQ(USB_DRD_FS_IRQn);
  HAL_SuspendTick();
  uint32_t before = rtcCounter;
  do { HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI); } while (rtcCounter == before);
  SystemClock_Config();          // STOP halted HSI/HSI48 -> restore
  HAL_ResumeTick();
}

// ============================================================================
//  Utility (settings persistence, battery, JSON out)
// ============================================================================
// Simple metadata: erase the reserved top sector and write one `meta` record at
// its base (mirrors EEPROM.put). Erased flash reads 0xFF -> gfrq == -1 flags a
// fresh chip. (One sector erase per save; fine for now, revisit for flash wear.)
// Metadata now stores ONLY settings + the download pointer. writeAddress and
// dataCount are NOT trusted from here -- they are reconstructed from the actual
// log by recoverLogPointers(), so the write pointer can never desync from the
// data (the old per-record meta save was the source of the pointer-lag bug).
void updateEeprom() {
  meta m;
  m.magic = META_MAGIC;
  m.gfrq = gpsFrequency; m.gto = gpsTimeout; m.hdop = gpsHdop;
  m.count = (uint16_t)dataCount; m.wa = writeAddress; m.ra = dlCount;  // wa/count informational; ra = download marker
  m.msat = minSat;
  flash.eraseSector(metaAddr);
  flash.writeStruct(metaAddr, m);
}

// Reconstruct the log write pointer by scanning records from address 0 until the
// first blank/foreign slot. Authoritative -- immune to metadata pointer lag.
void recoverLogPointers() {
  uint32_t logEnd = flash.getCapacity() - flash.getSectorSize();
  uint32_t nslots = logEnd / sizeof(data);
  data d;
  uint32_t i = 0, prev = 0, maxc = 0;
  bool any = false;
  for (; i < nslots; i++) {
    if (flash.readStruct(i * sizeof(data), d) != FLASH_OK) break;
    if (d.id != tag) break;                     // blank slot -> head (log not yet full)
    if (any && d.count != prev + 1) break;      // count drops -> ring boundary (wrapped)
    prev = d.count; maxc = d.count; any = true;
  }
  writeAddress = i * sizeof(data);              // ring head (wraps on the next write)
  dataCount = any ? maxc + 1 : 0;               // next monotonic count
}

bool loadEepromSettings() {
  meta m;
  flash.readStruct(metaAddr, m);
  if (m.magic != META_MAGIC) {            // fresh / foreign / old-format metadata
    gpsFrequency = 3; gpsTimeout = 60; gpsHdop = 5; minSat = 4;
    dlCount = 0;
    updateEeprom();                       // write a valid meta
    return false;
  }
  gpsFrequency = m.gfrq; gpsTimeout = m.gto; gpsHdop = m.hdop;
  minSat = m.msat;
  if (minSat < 3 || minSat > 32) minSat = 4;
  dlCount = m.ra;                         // meta.ra repurposed as the download count marker
  if (dlCount > dataCount) dlCount = dataCount;   // clamp a stale marker
  return true;
}

float readBatteryVoltage() {
  digitalWrite(SNS_EN, HIGH);            // enable the divider (STM32: gated by SNS_EN)
  delayMicroseconds(300);
  uint16_t raw = analogRead(BAT_SNS);
  digitalWrite(SNS_EN, LOW);
  return (raw / 4095.0f) * 3.3f * 2.0f;  // 2:1 divider
}

// ---- LIS3DHTR: bit-banged I2C (avoids the ~10 KB Wire/HAL-I2C stack) --------
// Open-drain: release = INPUT_PULLUP (line pulled high), assert 0 = drive LOW.
// The board's I2C pull-ups do the rest. ~100 kHz; only a few bytes, rarely.
static inline void i2cDelay()   { delayMicroseconds(4); }
static inline void sdaHi()      { pinMode(ACCEL_SDA, INPUT_PULLUP); }
static inline void sdaLo()      { pinMode(ACCEL_SDA, OUTPUT); digitalWrite(ACCEL_SDA, LOW); }
static inline void sclHi()      { pinMode(ACCEL_SCL, INPUT_PULLUP); }
static inline void sclLo()      { pinMode(ACCEL_SCL, OUTPUT); digitalWrite(ACCEL_SCL, LOW); }
static inline uint8_t sdaGet()  { pinMode(ACCEL_SDA, INPUT_PULLUP); return (uint8_t)digitalRead(ACCEL_SDA); }

static void i2cStart() { sdaHi(); sclHi(); i2cDelay(); sdaLo(); i2cDelay(); sclLo(); i2cDelay(); }
static void i2cStop()  { sdaLo(); i2cDelay(); sclHi(); i2cDelay(); sdaHi(); i2cDelay(); }

static bool i2cWrite(uint8_t b) {                 // returns true on ACK
  for (uint8_t i = 0; i < 8; i++) {
    if (b & 0x80) sdaHi(); else sdaLo();
    b <<= 1; i2cDelay(); sclHi(); i2cDelay(); sclLo(); i2cDelay();
  }
  sdaHi(); i2cDelay(); sclHi(); i2cDelay();
  bool ack = (sdaGet() == 0);
  sclLo(); i2cDelay();
  return ack;
}
static uint8_t i2cRead(bool ack) {
  uint8_t b = 0; sdaHi();
  for (uint8_t i = 0; i < 8; i++) {
    i2cDelay(); sclHi(); i2cDelay(); b = (uint8_t)((b << 1) | (sdaGet() & 1)); sclLo();
  }
  if (ack) sdaLo(); else sdaHi();
  i2cDelay(); sclHi(); i2cDelay(); sclLo(); sdaHi(); i2cDelay();
  return b;
}

void accelWriteReg(uint8_t reg, uint8_t val) {
  i2cStart(); i2cWrite(accelAddr << 1); i2cWrite(reg); i2cWrite(val); i2cStop();
}
uint8_t accelReadReg(uint8_t reg) {
  i2cStart(); i2cWrite(accelAddr << 1); i2cWrite(reg);
  i2cStart(); i2cWrite((accelAddr << 1) | 1); uint8_t v = i2cRead(false); i2cStop();
  return v;
}

bool accelInit() {
  accelAddr = 0x18;                               // SA0 low
  if (accelReadReg(0x0F) != 0x33) {               // WHO_AM_I
    accelAddr = 0x19;                             // SA0 high
    if (accelReadReg(0x0F) != 0x33) return false;
  }
  accelWriteReg(0x20, 0x57);   // CTRL_REG1: 100 Hz, X/Y/Z enabled
  accelWriteReg(0x23, 0x88);   // CTRL_REG4: BDU + high-resolution, +/-2 g
  return true;
}

// Read x/y/z in g. +/-2 g high-res = 1 mg/LSB on the 12-bit (left-justified) sample.
void accelRead(float& x, float& y, float& z) {
  i2cStart(); i2cWrite(accelAddr << 1); i2cWrite(0x28 | 0x80);   // OUT_X_L, auto-increment
  i2cStart(); i2cWrite((accelAddr << 1) | 1);
  uint8_t xl = i2cRead(true), xh = i2cRead(true);
  uint8_t yl = i2cRead(true), yh = i2cRead(true);
  uint8_t zl = i2cRead(true), zh = i2cRead(false);
  i2cStop();
  x = (int16_t)((xh << 8) | xl) / 16 * 0.001f;
  y = (int16_t)((yh << 8) | yl) / 16 * 0.001f;
  z = (int16_t)((zh << 8) | zl) / 16 * 0.001f;
}

// Civil date/time -> Unix epoch (replaces AVR TimeLib).
uint32_t toEpoch(uint16_t y, uint8_t mo, uint8_t d, uint8_t h, uint8_t mi, uint8_t s) {
  int yy = (int)y - (mo <= 2), era = (yy >= 0 ? yy : yy - 399) / 400;
  unsigned yoe = (unsigned)(yy - era * 400);
  unsigned doy = (153u * (mo + (mo > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  long days = (long)era * 146097 + (long)doe - 719468;
  return (uint32_t)(days * 86400L + h * 3600L + mi * 60L + s);
}

// Print a float with `decimals` fixed places using integer/float math only.
// Avoids Serial.print(float) -> Arduino printFloat, whose rounding is done in
// double (~pulls the 64-bit soft-float routines). Single-precision only here.
void printFixed(float v, uint8_t decimals) {
  if (v < 0) { Serial.write('-'); v = -v; }
  uint32_t mult = 1; for (uint8_t i = 0; i < decimals; i++) mult *= 10;
  uint32_t scaled = (uint32_t)(v * (float)mult + 0.5f);
  Serial.print(scaled / mult);                 // integer part (no float print)
  if (decimals) {
    Serial.write('.');
    uint32_t fp = scaled % mult;
    for (uint32_t d = mult / 10; d > 0; d /= 10) Serial.write((char)('0' + (fp / d) % 10));
  }
}

// TinyGPS++ raw (integer) degrees -> float, bypassing its double lat()/lng().
float rawToDeg(const RawDegrees &r) {
  float d = (float)r.deg + (float)r.billionths * 1e-9f;
  return r.negative ? -d : d;
}

void sendSerialMessage(byte code) {
  Serial.print(F("{\"Msg\":")); Serial.print(code);
  Serial.print(F(",\"ID\":"));  Serial.print(tag);
  Serial.println(F("}"));
  Serial.flush();
}

void sendMemoryStatus() {
  uint32_t logBytes = flash.getCapacity() - flash.getSectorSize();
  uint32_t nslots   = logBytes / sizeof(data);
  uint32_t avail    = (dataCount < nslots) ? dataCount : nslots;   // records available to download
  uint32_t used     = avail * sizeof(data);
  Serial.print(F("{\"Msg\":"));     Serial.print(MEMORY_STATUS);
  Serial.print(F(",\"ID\":"));      Serial.print(tag);
  Serial.print(F(",\"Used\":"));    Serial.print(used);
  Serial.print(F(",\"Free\":"));    Serial.print(logBytes - used);
  Serial.print(F(",\"Percent\":")); printFixed((float)used / (float)logBytes * 100.0f, 2);
  Serial.print(F(",\"Count\":"));   Serial.print(avail);            // downloadable records
  Serial.print(F(",\"Total\":"));   Serial.print(dataCount);        // lifetime records logged
  Serial.println(F("}"));
  Serial.flush();
}

// ============================================================================
//  GPS
// ============================================================================
bool acquireGPSFix(uint32_t timeoutSeconds) {
  uint32_t start = millis();
  int32_t hdopX100 = 999900;              // HDOP x100 (integer; avoids double)
  digitalWrite(GPS_EN, HIGH);
  delay(100);
  gpsSerial.begin(GPS_BAUD);

  while (millis() - start < timeoutSeconds * 1000UL) {
    IWatchdog.reload();                       // acquire can outlast the watchdog window
    while (gpsSerial.available()) gps.encode((char)gpsSerial.read());
    if (gps.hdop.isValid() && gps.hdop.value() > 0) hdopX100 = gps.hdop.value();

    if (hdopX100 < (int32_t)gpsHdop * 100 &&
        gps.satellites.value() >= (uint32_t)minSat &&
        gps.location.age() < 1000 &&
        millis() - start > 3000) {
      digitalWrite(GPS_EN, LOW);
      lastLockS = (uint16_t)((millis() - start) / 1000);   // time to acquire the fix
      return true;
    }
  }
  digitalWrite(GPS_EN, LOW);
  lastLockS = (uint16_t)((millis() - start) / 1000);       // = timeout when no fix
  return false;
}

bool recordGPSData() {
  bool ok = acquireGPSFix(gpsTimeout);

  data dat;
  memset(&dat, 0, sizeof(dat));
  if (ok && gps.location.age() < 2000) {
    dat.lat = rawToDeg(gps.location.rawLat());
    dat.lng = rawToDeg(gps.location.rawLng());
  }
  if (gps.time.isValid() && gps.date.isValid())
    dat.datetime = toEpoch(gps.date.year(), gps.date.month(), gps.date.day(),
                           gps.time.hour(), gps.time.minute(), gps.time.second());
  dat.locktime = lastLockS;              // seconds the acquire actually took
  dat.hdop = gps.hdop.value() / 100.0f;   // raw int (HDOP x100) -> float; no double
  dat.count = dataCount;
  dat.id = tag;
  dat.sats = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;
  if (accelPresent) {                    // packed struct: read to locals, then store
    float ax, ay, az;
    accelRead(ax, ay, az);
    dat.x = ax; dat.y = ay; dat.z = az;
  }

  // Circular log: when the head reaches the end of the log region, wrap to 0 and
  // start overwriting the oldest data. metaAddr is the log end (top sector reserved).
  if (writeAddress + sizeof(data) > metaAddr) writeAddress = 0;

  // Erase-on-demand: erase each sector the moment the write pointer reaches its
  // base (handles record straddle, and wipes the oldest sector on wrap).
  uint32_t ss = flash.getSectorSize();
  uint32_t first = writeAddress / ss, last = (writeAddress + sizeof(data) - 1) / ss;
  for (uint32_t s = first; s <= last; s++) { uint32_t base = s * ss; if (writeAddress <= base) flash.eraseSector(base); }

  if (flash.writeStruct(writeAddress, dat) == FLASH_OK) {
    writeAddress += sizeof(data);
    dataCount++;
    // No meta write here: writeAddress/dataCount are recovered by scanning the
    // log on boot, so a power loss mid-cycle can't desync the pointer. This also
    // removes a meta-sector erase from every record (far less flash wear).
    return true;
  }
  return false;
}

// ============================================================================
//  Data download
// ============================================================================
void sendDataRecord(const data& dat) {
  Serial.print(F("{\"ID\":"));    Serial.print(dat.id);
  Serial.print(F(",\"DT\":"));    Serial.print(dat.datetime);
  Serial.print(F(",\"LAT\":"));   printFixed(dat.lat, 6);
  Serial.print(F(",\"LNG\":"));   printFixed(dat.lng, 6);
  Serial.print(F(",\"HDOP\":"));  printFixed(dat.hdop, 2);
  Serial.print(F(",\"SATS\":"));  Serial.print(dat.sats);
  Serial.print(F(",\"LCKTM\":")); Serial.print(dat.locktime);
  Serial.print(F(",\"CNT\":"));   Serial.print(dat.count);
  Serial.print(F(",\"X\":"));     printFixed(dat.x, 3);
  Serial.print(F(",\"Y\":"));     printFixed(dat.y, 3);
  Serial.print(F(",\"Z\":"));     printFixed(dat.z, 3);
  Serial.println(F("}"));
  Serial.flush();
}

// Iterate the ring oldest -> newest. Not wrapped: records 0..dataCount-1 from slot 0.
// Wrapped: nslots records starting at the head (the oldest, next to be overwritten).
void downloadAllData() {
  sendSerialMessage(DATA_DOWNLOAD_BEGIN);
  uint32_t nslots = (flash.getCapacity() - flash.getSectorSize()) / sizeof(data);
  uint32_t n     = (dataCount < nslots) ? dataCount : nslots;
  uint32_t start = (dataCount <= nslots) ? 0 : (writeAddress / sizeof(data));
  for (uint32_t k = 0; k < n; k++) {
    data d;
    uint32_t slot = (start + k) % nslots;
    if (flash.readStruct(slot * sizeof(data), d) == FLASH_OK && d.id == tag) sendDataRecord(d);
    IWatchdog.reload();
  }
  sendSerialMessage(DATA_DOWNLOAD_END);
}

void downloadNewData() {
  sendSerialMessage(DATA_DOWNLOAD_BEGIN);
  uint32_t nslots = (flash.getCapacity() - flash.getSectorSize()) / sizeof(data);
  uint32_t n     = (dataCount < nslots) ? dataCount : nslots;
  uint32_t start = (dataCount <= nslots) ? 0 : (writeAddress / sizeof(data));
  for (uint32_t k = 0; k < n; k++) {
    data d;
    uint32_t slot = (start + k) % nslots;
    if (flash.readStruct(slot * sizeof(data), d) == FLASH_OK && d.id == tag && d.count >= dlCount)
      sendDataRecord(d);
    IWatchdog.reload();
  }
  dlCount = dataCount;          // everything up to now is now downloaded
  updateEeprom();              // persist the marker
  sendSerialMessage(DATA_DOWNLOAD_END);
}

// ============================================================================
//  Command session (non-blocking calibration + responsive command window)
// ============================================================================
// GPS acquires a fix in the BACKGROUND while the app can download data, change
// settings, or request the summary at any time. Replaces the old blocking
// deviceCalibration() + handleSerialCommands().

// Fix criteria, integer-only (no float/double). HDOP is compared as value()x100.
bool gpsFixOk() {
  return gps.location.isValid() && gps.location.age() < 1000 &&
         gps.satellites.value() >= (uint32_t)minSat &&
         gps.hdop.isValid() && gps.hdop.value() > 0 &&
         gps.hdop.value() < (int32_t)gpsHdop * 100;
}

// Current calibration/health snapshot. Sent automatically on first fix and on
// demand when the app sends CALIBRATION_SUMMARY.
void sendCalibrationSummary() {
  uint32_t cap = flash.getCapacity();
  bool haveFix = gps.location.isValid() && gps.location.age() < 5000;
  Serial.print(F("{\"ID\":"));          Serial.print(tag);
  Serial.print(F(",\"Fix\":"));         Serial.print(haveFix ? 1 : 0);
  Serial.print(F(",\"Sats\":"));        Serial.print(gps.satellites.isValid() ? gps.satellites.value() : 0);
  Serial.print(F(",\"LAT\":"));         printFixed(haveFix ? rawToDeg(gps.location.rawLat()) : 0.0f, 6);
  Serial.print(F(",\"LNG\":"));         printFixed(haveFix ? rawToDeg(gps.location.rawLng()) : 0.0f, 6);
  Serial.print(F(",\"HDOP\":"));        printFixed(gps.hdop.isValid() ? gps.hdop.value() / 100.0f : 0.0f, 2);
  Serial.print(F(",\"DataPoints\":"));  Serial.print(dataCount);
  Serial.print(F(",\"StorageUsed\":")); printFixed((float)writeAddress / (float)cap * 100.0f, 2);
  Serial.print(F(",\"StorageFree\":")); printFixed((float)(cap - writeAddress) / (float)cap * 100.0f, 2);
  Serial.print(F(",\"BatteryV\":"));    printFixed(readBatteryVoltage(), 2);
  Serial.println(F("}"));
  Serial.flush();
}

void commandSession() {
  // --- calibration handshake (fast; GPS then acquires in the background) ---
  sendSerialMessage(CALIBRATION_BEGIN);
  sendSerialMessage(FLASH_DIAGNOSTICS);
  uint32_t jedec = flash.getJEDECID();
  sendSerialMessage((jedec != 0 && jedec != 0xFFFFFFFF) ? FLASH_SUCCESS : FLASH_ERROR);
  sendSerialMessage(FLASH_DIAGNOSTICS_END);
  sendSerialMessage(accelPresent ? ACCELEROMETER_OK : ACCELEROMETER_ERROR);
  sendSerialMessage(GPS_CALIBRATION);
  digitalWrite(GPS_EN, HIGH);
  delay(100);
  gpsSerial.begin(GPS_BAUD);
  sendSerialMessage(CALIBRATION_END);          // handshake done -> interactive window opens

  // --- interactive command window --------------------------------------------
  // Stays open as long as the app is active: any received frame resets the idle
  // timer. Closes on FINISH_CALIBRATION or CMD_IDLE_TIMEOUT_MS of silence. GPS
  // keeps acquiring in the background (GPS_CALIBRATING heartbeat; on first fix ->
  // GPS_SUCCESS + GPS_CALIBRATION_END + summary).
  bool     gpsFixed = false, finish = false;
  uint32_t lastActivity = millis(), lastBeat = 0;

  while (!finish && millis() - lastActivity < CMD_IDLE_TIMEOUT_MS) {
    IWatchdog.reload();

    while (gpsSerial.available()) gps.encode((char)gpsSerial.read());   // feed parser, no blocking

    if (!gpsFixed && gpsFixOk()) {                       // first fix -> announce + summary
      gpsFixed = true;
      sendSerialMessage(GPS_SUCCESS);
      sendSerialMessage(GPS_CALIBRATION_END);
      sendCalibrationSummary();
    } else if (!gpsFixed && millis() - lastBeat >= 3000) {
      lastBeat = millis();
      sendSerialMessage(GPS_CALIBRATING);               // "still searching" heartbeat
    }

    // service one command frame if present (binary, tag-guarded; length-dispatched)
    if (Serial.available() >= (int)sizeof(reqPing)) {
      lastActivity = millis();                           // app activity -> keep window open
      delay(20);                                          // let the whole frame arrive
      if (Serial.available() >= (int)sizeof(setttings)) {
        setttings s;
        Serial.readBytes((char*)&s, sizeof(s));
        if (s.tag == tag) {
          gpsFrequency = s.gpsFrq; gpsTimeout = s.gpsTout; gpsHdop = s.hdop; minSat = s.minSat;
          if (gpsFrequency < 1) gpsFrequency = 1; if (gpsFrequency > 1440) gpsFrequency = 1440;
          if (gpsTimeout < 10)  gpsTimeout = 10;  if (gpsTimeout > 300)   gpsTimeout = 300;
          if (gpsHdop < 1)      gpsHdop = 1;      if (gpsHdop > 20)       gpsHdop = 20;
          if (minSat < 3)       minSat = 3;       if (minSat > 32)        minSat = 32;
          updateEeprom();
          sendSerialMessage(SETTINGS_UPDATED);
        } else sendSerialMessage(SETTINGS_UPDATE_ERROR);
      } else {
        reqPing p;
        Serial.readBytes((char*)&p, sizeof(p));
        if (p.tag == tag) {
          switch (p.request) {
            case FINISH_CALIBRATION:  finish = true; break;
            case DATA_DOWNLOAD_ALL:   downloadAllData(); break;
            case DATA_DOWNLOAD_NEW:   downloadNewData(); break;
            case MEMORY_CLEAR:
              flash.eraseSector(0);            // wipe sector 0 so the boot scan sees an empty log
              writeAddress = 0; dlCount = 0; dataCount = 0;
              updateEeprom();
              sendSerialMessage(MEMORY_CLEARED);
              break;
            case MEMORY_STATUS:       sendMemoryStatus(); break;
            case REQUEST_SETTINGS:    loadEepromSettings(); sendSerialMessage(SETTINGS_UPDATED); break;
            case CALIBRATION_SUMMARY: sendCalibrationSummary(); break;   // on-demand summary
            case REQ_DFU:             bootFlag = DFU_BOOT_MAGIC; NVIC_SystemReset(); break;  // dev
            default: break;
          }
        }
      }
    }
    delay(5);
  }

  digitalWrite(GPS_EN, LOW);                      // window closed; logging loop re-powers per fix
}

// ============================================================================
//  Setup & loop
// ============================================================================
void setup() {
  if (bootFlag == DFU_BOOT_MAGIC) { bootFlag = 0; jumpToBootloader(); }

  Serial.begin(115200);                 // USB CDC
  delay(3000);                          // let USB enumerate / host open the port before we print
  pinMode(GPS_EN, OUTPUT); digitalWrite(GPS_EN, LOW);   // GPS off through boot
  pinMode(SNS_EN, OUTPUT); digitalWrite(SNS_EN, LOW);
  analogReadResolution(12);

  if (!flash.begin()) { while (1) { delay(1000); } }    // flash mandatory -> halt
  metaAddr = flash.getCapacity() - flash.getSectorSize();

  recoverLogPointers();                 // derive writeAddress/dataCount from the actual log

  IWatchdog.begin(WDG_TIMEOUT_US);      // arm after flash; kicked in every long-running loop

  accelPresent = accelInit();           // LIS3DHTR on I2C (raw x/y/z)

  loadEepromSettings();                 // settings + download pointer (writeAddress from scan)
  commandSession();                     // non-blocking calibration + responsive commands
  loadEepromSettings();

  RTC_init();
}

void loop() {
  IWatchdog.reload();                     // IWDG runs through STOP (LSI); kick each wake
  if (sleeping) {
    if (rtcCounter >= (uint32_t)gpsFrequency * 60UL) sleeping = false;
  } else {
    rtcCounter = 0;                      // start the interval at the cycle start -> true cadence
    recordGPSData();                     // (acquire time now counts within gpsFrequency)
    sleeping = true;
  }
  sleepOneTick();                        // STOP ~1 s, woken by the RTC alarm
}
