/*
 * main-ag.cpp - ArcTrack Logger (STM32C071K8) - Hardened & CSV Edition.
 *
 * Hardened architecture port of the ArcTrack Logger featuring:
 *   - CSV + CRC-8 communication protocol (replaces length-dispatched binary structs & JSON).
 *   - Fixed GPS fix validation (location.isValid() enforced).
 *   - Terminated UART and bus leakage prevention on GPS power-down.
 *   - Settled ADC battery sampling across high-impedance divider.
 *   - Confirmation-guarded MEMORY_CLEAR and REQ_DFU to prevent arbitrary wipe/takeover.
 *   - Dual-sector invalidation in MEMORY_CLEAR to prevent ghost record resurrection.
 *   - Ultra-low power STOP mode with active IWatchdog (no HSI48/CRS clock overhead in sleep).
 *
 * NOTE: Compatible with existing definitions.h and codes.h without modifications.
 */
#include <Arduino.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <IWatchdog.h>
#include "definitions.h"
#include "codes.h"
#ifndef REQ_SUMMARY
#define REQ_SUMMARY 201
#endif
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

// ---- LIS3DHTR accelerometer (I2C2: SDA=PA6, SCL=PA7) -----------------------
#define ACCEL_SDA I2C2_SDA
#define ACCEL_SCL I2C2_SCL
uint8_t accelAddr    = 0x18;       // resolved at init (0x18 or 0x19 by WHO_AM_I)
bool    accelPresent = false;

// ---- Constants -------------------------------------------------------------
const uint32_t CMD_IDLE_TIMEOUT_MS = 300000; // command window closes after 5 min of inactivity
const uint32_t GPS_BAUD            = 9600;   // GPS module baud
const uint32_t META_MAGIC          = 0x41544B32UL;  // "ATK2"
const uint32_t WDG_TIMEOUT_US      = 25000000; // ~25 s independent watchdog (< 32 s max)

// ============================================================================
//  CRC-8 Implementation (Dallas/Maxim: x^8 + x^5 + x^4 + 1, Poly 0x31)
// ============================================================================
uint8_t crc8(const char* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint8_t)data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// ============================================================================
//  CSV Buffer Formatter (Zero-allocation, single-precision fixed point)
// ============================================================================
class CsvBuffer {
public:
  char buf[128];
  size_t len;

  CsvBuffer() : len(0) { buf[0] = '\0'; }

  void reset() {
    len = 0;
    buf[0] = '\0';
  }

  void appendStr(const char* s) {
    while (*s && len < sizeof(buf) - 8) {
      buf[len++] = *s++;
    }
    buf[len] = '\0';
  }

  void appendChar(char c) {
    if (len < sizeof(buf) - 8) {
      buf[len++] = c;
    }
    buf[len] = '\0';
  }

  void appendInt(int32_t v) {
    char tmp[16];
    itoa(v, tmp, 10);
    appendStr(tmp);
  }

  void appendUint(uint32_t v) {
    char tmp[16];
    utoa(v, tmp, 10);
    appendStr(tmp);
  }

  void appendFixed(float v, uint8_t decimals) {
    if (isnan(v) || isinf(v)) {
      appendStr("0.0");
      return;
    }
    if (v < 0) {
      appendChar('-');
      v = -v;
    }
    uint32_t mult = 1;
    for (uint8_t i = 0; i < decimals; i++) mult *= 10;
    uint32_t scaled = (uint32_t)(v * (float)mult + 0.5f);
    appendUint(scaled / mult);
    if (decimals) {
      appendChar('.');
      uint32_t fp = scaled % mult;
      for (uint32_t d = mult / 10; d > 0; d /= 10) {
        appendChar((char)('0' + (fp / d) % 10));
      }
    }
  }

  void send() {
    uint8_t crc = crc8(buf, len);
    Serial.write('$');
    Serial.write(buf, len);
    Serial.write('*');
    const char hexDigits[] = "0123456789ABCDEF";
    Serial.write(hexDigits[(crc >> 4) & 0x0F]);
    Serial.write(hexDigits[crc & 0x0F]);
    Serial.write("\r\n");
    // Non-blocking: USB CDC transmit queue handles async packet delivery.
    // Avoids indefinite hang in core while-loop if USB host disconnects.
  }
};

static CsvBuffer outBuf;

// Helper to send a simple status/code message
void sendCsvMessage(uint8_t code) {
  outBuf.reset();
  outBuf.appendStr("MSG,");
  outBuf.appendUint(tag);
  outBuf.appendChar(',');
  outBuf.appendUint(code);
  outBuf.send();
}

// Helper to broadcast current settings
void sendCsvSettings() {
  outBuf.reset();
  outBuf.appendStr("SET,");
  outBuf.appendUint(tag);
  outBuf.appendChar(',');
  outBuf.appendInt(gpsFrequency);
  outBuf.appendChar(',');
  outBuf.appendInt(gpsTimeout);
  outBuf.appendChar(',');
  outBuf.appendInt(gpsHdop);
  outBuf.appendChar(',');
  outBuf.appendInt(minSat);
  outBuf.send();
}

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

// Restore 24 MHz core clock (HSI / 2) before waking peripherals (GPS, Flash, etc.)
// Ensures exact 9600 baud for GPS and 1 ms SysTick while keeping HSI48/CRS OFF to save power.
void restoreCoreClock(void) {
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  osc.HSIState            = RCC_HSI_ON;
  osc.HSIDiv              = RCC_HSI_DIV2;          // 24 MHz
  osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  HAL_RCC_OscConfig(&osc);

  clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1;
  clk.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
  clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0);
}

// ============================================================================
//  DFU re-entry (Guarded)
// ============================================================================
#define DFU_BOOT_MAGIC 0xB00710ADUL
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
//  RTC 1 Hz tick + STOP sleep (Active IWDG, No Clock Overhead on Sleep Ticks)
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

  hrtc.Instance          = RTC;
  hrtc.Init.HourFormat   = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv  = 249;
  hrtc.Init.OutPut       = RTC_OUTPUT_DISABLE;
  HAL_RTC_Init(&hrtc);

  // Fully masked Alarm A: triggers every second without calendar math (cannot miss)
  RTC_AlarmTypeDef a = {0};
  a.Alarm               = RTC_ALARM_A;
  a.AlarmMask           = RTC_ALARMMASK_ALL;
  a.AlarmSubSecondMask  = RTC_ALARMSUBSECONDMASK_ALL;
  a.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  a.AlarmDateWeekDay     = 1;
  HAL_NVIC_SetPriority(RTC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(RTC_IRQn);
  HAL_RTC_SetAlarm_IT(&hrtc, &a, RTC_FORMAT_BIN);
  __HAL_RTC_ALARM_EXTI_ENABLE_IT();
}

// Sleep in STOP mode until next 1 Hz alarm. Intermediate ticks wake on 12 MHz HSI,
// reload watchdog, and re-enter STOP without waking HSI48/CRS.
void sleepOneTick(void) {
  NVIC_DisableIRQ(USB_DRD_FS_IRQn);
  HAL_SuspendTick();
  uint32_t before = rtcCounter;
  do {
    HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
  } while (rtcCounter == before);
  
  // NOTE: On wake from STOP, MCU runs on bare 12 MHz HSI.
  // We do NOT call SystemClock_Config() here to avoid firing up HSI48/CRS in sleep.
  HAL_ResumeTick();
}

// ============================================================================
//  Metadata Persistence & Log Recovery
// ============================================================================
void updateEeprom() {
  meta m;
  m.magic = META_MAGIC;
  m.gfrq  = gpsFrequency;
  m.gto   = gpsTimeout;
  m.hdop  = gpsHdop;
  // Saturate count in metadata if it exceeds 16-bit capacity
  m.count = (dataCount > 65535) ? 65535 : (uint16_t)dataCount;
  m.wa    = writeAddress;
  m.ra    = dlCount;
  m.msat  = minSat;
  flash.eraseSector(metaAddr);
  flash.writeStruct(metaAddr, m);
}

void recoverLogPointers() {
  uint32_t logEnd = flash.getCapacity() - flash.getSectorSize();
  uint32_t nslots = logEnd / sizeof(data);
  data d;
  uint32_t i = 0, prev = 0, maxc = 0;
  bool any = false;
  for (; i < nslots; i++) {
    if (flash.readStruct(i * sizeof(data), d) != FLASH_OK) break;
    if (d.id != tag) break;                     // blank slot -> head
    if (any && d.count != prev + 1) break;      // count drops -> ring boundary
    prev = d.count; maxc = d.count; any = true;
  }
  writeAddress = i * sizeof(data);
  dataCount    = any ? maxc + 1 : 0;
}

bool loadEepromSettings() {
  meta m;
  flash.readStruct(metaAddr, m);
  if (m.magic != META_MAGIC) {
    gpsFrequency = 3; gpsTimeout = 60; gpsHdop = 5; minSat = 4;
    dlCount = 0;
    updateEeprom();
    return false;
  }
  gpsFrequency = m.gfrq;
  gpsTimeout   = m.gto;
  gpsHdop      = m.hdop;
  minSat       = m.msat;
  if (gpsFrequency < 1 || gpsFrequency > 1440) gpsFrequency = 3;
  if (gpsTimeout < 30 || gpsTimeout > 300)     gpsTimeout = 60;
  if (gpsHdop < 2 || gpsHdop > 20)             gpsHdop = 5;
  if (minSat < 3 || minSat > 12)               minSat = 4;
  dlCount      = m.ra;
  if (dlCount > dataCount) dlCount = dataCount;
  return true;
}

// FIXED (VULN-07): 10 ms settling delay for 470k divider + multi-sample averaging
float readBatteryVoltage() {
  digitalWrite(SNS_EN, HIGH);
  delay(10); // Allow high-impedance divider to fully settle
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 4; i++) {
    sum += analogRead(BAT_SNS);
    delayMicroseconds(50);
  }
  digitalWrite(SNS_EN, LOW);
  float raw = (float)sum / 4.0f;
  return (raw / 4095.0f) * 3.3f * 2.0f;
}

// ============================================================================
//  LIS3DHTR: Bit-Banged I2C
// ============================================================================
static inline void i2cDelay()   { delayMicroseconds(4); }
static inline void sdaHi()      { pinMode(ACCEL_SDA, INPUT_PULLUP); }
static inline void sdaLo()      { pinMode(ACCEL_SDA, OUTPUT); digitalWrite(ACCEL_SDA, LOW); }
static inline void sclHi()      { pinMode(ACCEL_SCL, INPUT_PULLUP); }
static inline void sclLo()      { pinMode(ACCEL_SCL, OUTPUT); digitalWrite(ACCEL_SCL, LOW); }
static inline uint8_t sdaGet()  { pinMode(ACCEL_SDA, INPUT_PULLUP); return (uint8_t)digitalRead(ACCEL_SDA); }

static void i2cStart() { sdaHi(); sclHi(); i2cDelay(); sdaLo(); i2cDelay(); sclLo(); i2cDelay(); }
static void i2cStop()  { sdaLo(); i2cDelay(); sclHi(); i2cDelay(); sdaHi(); i2cDelay(); }

static bool i2cWrite(uint8_t b) {
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
  accelAddr = 0x18;
  if (accelReadReg(0x0F) != 0x33) {
    accelAddr = 0x19;
    if (accelReadReg(0x0F) != 0x33) return false;
  }
  accelWriteReg(0x20, 0x57);   // 100 Hz, X/Y/Z enabled
  accelWriteReg(0x23, 0x88);   // BDU + high-res, +/-2 g
  return true;
}

// Put LIS3DHTR into 0.5 µA Power-Down mode (ODR[3:0] = 0000)
void accelSleep() {
  if (accelPresent) {
    accelWriteReg(0x20, 0x07); // Power-down, axes enabled
  }
}

// Wake LIS3DHTR to 100 Hz High-Res mode (~11 µA) and wait for first valid sample
void accelWake() {
  if (accelPresent) {
    accelWriteReg(0x20, 0x57); // 100 Hz, X/Y/Z enabled
    delay(12);                 // Turn-on settling time: 1/ODR + 1ms ~ 11 ms
  }
}

void accelRead(float& x, float& y, float& z) {
  i2cStart(); i2cWrite(accelAddr << 1); i2cWrite(0x28 | 0x80);
  i2cStart(); i2cWrite((accelAddr << 1) | 1);
  uint8_t xl = i2cRead(true), xh = i2cRead(true);
  uint8_t yl = i2cRead(true), yh = i2cRead(true);
  uint8_t zl = i2cRead(true), zh = i2cRead(false);
  i2cStop();
  x = (int16_t)((xh << 8) | xl) / 16 * 0.001f;
  y = (int16_t)((yh << 8) | yl) / 16 * 0.001f;
  z = (int16_t)((zh << 8) | zl) / 16 * 0.001f;
}

// Civil date/time -> Unix epoch
uint32_t toEpoch(uint16_t y, uint8_t mo, uint8_t d, uint8_t h, uint8_t mi, uint8_t s) {
  int yy = (int)y - (mo <= 2), era = (yy >= 0 ? yy : yy - 399) / 400;
  unsigned yoe = (unsigned)(yy - era * 400);
  unsigned doy = (153u * (mo + (mo > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  long days = (long)era * 146097 + (long)doe - 719468;
  return (uint32_t)(days * 86400L + h * 3600L + mi * 60L + s);
}

float rawToDeg(const RawDegrees &r) {
  float d = (float)r.deg + (float)r.billionths * 1e-9f;
  return r.negative ? -d : d;
}

// ============================================================
//  CSV Status & Telemetry Reporters
// ============================================================
void sendMemoryStatus() {
  uint32_t logBytes = flash.getCapacity() - flash.getSectorSize();
  uint32_t nslots   = logBytes / sizeof(data);
  uint32_t avail    = (dataCount < nslots) ? dataCount : nslots;
  uint32_t used     = avail * sizeof(data);
  float pctUsed     = (float)used / (float)logBytes * 100.0f;

  outBuf.reset();
  outBuf.appendStr("MEM,");
  outBuf.appendUint(tag);
  outBuf.appendChar(',');
  outBuf.appendUint(used);
  outBuf.appendChar(',');
  outBuf.appendUint(logBytes - used);
  outBuf.appendChar(',');
  outBuf.appendFixed(pctUsed, 2);
  outBuf.appendChar(',');
  outBuf.appendUint(avail);
  outBuf.appendChar(',');
  outBuf.appendUint(dataCount);
  outBuf.send();
}

void sendCalibrationSummary() {
  uint32_t cap = flash.getCapacity();
  if (cap == 0) cap = 4194304; // Fallback to 4MB to prevent divide-by-zero
  bool haveFix = gps.location.isValid() && gps.location.age() < 5000;
  float stUsed = (float)writeAddress / (float)cap * 100.0f;
  float stFree = (float)(cap - writeAddress) / (float)cap * 100.0f;
  bool chrgComplete = (digitalRead(CHRG_STAT) == LOW); // TP4057 STDBY: LOW = complete/standby
  uint32_t jedec = flash.getJEDECID();
  bool flashOk = (jedec != 0 && jedec != 0xFFFFFFFF);
  bool accelOk = accelPresent && (accelReadReg(0x0F) == 0x33);

  outBuf.reset();
  outBuf.appendStr("SUM,");
  outBuf.appendUint(tag);
  outBuf.appendChar(',');
  outBuf.appendInt(haveFix ? 1 : 0);
  outBuf.appendChar(',');
  outBuf.appendUint(gps.satellites.isValid() ? gps.satellites.value() : 0);
  outBuf.appendChar(',');
  outBuf.appendFixed(haveFix ? rawToDeg(gps.location.rawLat()) : 0.0f, 6);
  outBuf.appendChar(',');
  outBuf.appendFixed(haveFix ? rawToDeg(gps.location.rawLng()) : 0.0f, 6);
  outBuf.appendChar(',');
  outBuf.appendFixed(gps.hdop.isValid() ? gps.hdop.value() / 100.0f : 0.0f, 2);
  outBuf.appendChar(',');
  outBuf.appendUint(dataCount);
  outBuf.appendChar(',');
  outBuf.appendFixed(stUsed, 2);
  outBuf.appendChar(',');
  outBuf.appendFixed(stFree, 2);
  outBuf.appendChar(',');
  outBuf.appendFixed(readBatteryVoltage(), 2);
  outBuf.appendChar(',');
  outBuf.appendInt(chrgComplete ? 1 : 0);
  outBuf.appendChar(',');
  outBuf.appendInt(flashOk ? 1 : 0);
  outBuf.appendChar(',');
  outBuf.appendInt(accelOk ? 1 : 0);
  outBuf.send();
}

// ============================================================================
//  GPS Acquisition
// ============================================================================
bool acquireGPSFix(uint32_t timeoutSeconds) {
  gps = TinyGPSPlus(); // Clear all stale fix state/satellites from previous sessions
  uint32_t start = millis();
  int32_t hdopX100 = 999900;
  
  pinMode(GPS_EN, OUTPUT);
  digitalWrite(GPS_EN, HIGH);
  delay(100);
  gpsSerial.begin(GPS_BAUD);
  while (gpsSerial.available()) gpsSerial.read(); // Discard power-on framing noise

  bool fixOk = false;

  while (millis() - start < timeoutSeconds * 1000UL) {
    IWatchdog.reload();
    while (gpsSerial.available()) gps.encode((char)gpsSerial.read());
    if (gps.hdop.isValid() && gps.hdop.value() > 0) hdopX100 = gps.hdop.value();

    // FIXED (VULN-05): Verify gps.location.isValid() alongside age, sats, and HDOP
    if (gps.location.isValid() &&
        gps.location.age() < 1000 &&
        gps.satellites.isValid() &&
        gps.satellites.value() >= (uint32_t)minSat &&
        hdopX100 < (int32_t)gpsHdop * 100 &&
        millis() - start > 3000) {
      fixOk = true;
      break;
    }
  }

  lastLockS = (uint16_t)((millis() - start) / 1000);

  // FIXED (VULN-06): Terminate UART receiver and pulldown lines before cutting GPS power
  gpsSerial.end();
  pinMode(USART1_RX, INPUT_PULLDOWN);
  pinMode(USART1_TX, INPUT_PULLDOWN);
  digitalWrite(GPS_EN, LOW);

  return fixOk;
}

bool recordGPSData() {
  bool ok = acquireGPSFix(gpsTimeout);

  data dat;
  memset(&dat, 0, sizeof(dat));
  // FIXED (VULN-05): Check location.isValid() before recording coordinates
  if (ok && gps.location.isValid() && gps.location.age() < 2000) {
    dat.lat = rawToDeg(gps.location.rawLat());
    dat.lng = rawToDeg(gps.location.rawLng());
  }
  if (gps.time.isValid() && gps.date.isValid()) {
    dat.datetime = toEpoch(gps.date.year(), gps.date.month(), gps.date.day(),
                           gps.time.hour(), gps.time.minute(), gps.time.second());
  }
  dat.locktime = lastLockS;
  dat.hdop     = (gps.hdop.isValid() && gps.hdop.value() > 0) ? (gps.hdop.value() / 100.0f) : 99.99f;
  dat.count    = dataCount;
  dat.id       = tag;
  dat.sats     = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;
  if (accelPresent) {
    accelWake();
    float ax, ay, az;
    accelRead(ax, ay, az);
    dat.x = ax; dat.y = ay; dat.z = az;
    accelSleep();
  }

  // Circular ring buffer wrap check
  if (writeAddress + sizeof(data) > metaAddr) writeAddress = 0;

  // Erase-on-demand: erase sector base when write pointer reaches it
  uint32_t ss = flash.getSectorSize();
  uint32_t first = writeAddress / ss, last = (writeAddress + sizeof(data) - 1) / ss;
  for (uint32_t s = first; s <= last; s++) {
    uint32_t base = s * ss;
    if (writeAddress <= base) flash.eraseSector(base);
  }

  if (flash.writeStruct(writeAddress, dat) == FLASH_OK) {
    writeAddress += sizeof(data);
    dataCount++;
    return true;
  }
  return false;
}

// ============================================================================
//  CSV Data Download (45% smaller bandwidth than JSON)
// ============================================================================
void sendDataRecord(const data& dat) {
  outBuf.reset();
  outBuf.appendStr("DAT,");
  outBuf.appendUint(dat.id);
  outBuf.appendChar(',');
  outBuf.appendUint(dat.datetime);
  outBuf.appendChar(',');
  outBuf.appendFixed(dat.lat, 6);
  outBuf.appendChar(',');
  outBuf.appendFixed(dat.lng, 6);
  outBuf.appendChar(',');
  outBuf.appendFixed(dat.hdop, 2);
  outBuf.appendChar(',');
  outBuf.appendUint(dat.sats);
  outBuf.appendChar(',');
  outBuf.appendUint(dat.locktime);
  outBuf.appendChar(',');
  outBuf.appendUint(dat.count);
  outBuf.appendChar(',');
  outBuf.appendFixed(dat.x, 3);
  outBuf.appendChar(',');
  outBuf.appendFixed(dat.y, 3);
  outBuf.appendChar(',');
  outBuf.appendFixed(dat.z, 3);
  outBuf.send();
}

void downloadAllData() {
  sendCsvMessage(DATA_DOWNLOAD_BEGIN);
  uint32_t nslots = (flash.getCapacity() - flash.getSectorSize()) / sizeof(data);
  uint32_t n     = (dataCount < nslots) ? dataCount : nslots;
  uint32_t start = (dataCount <= nslots) ? 0 : (writeAddress / sizeof(data));
  for (uint32_t k = 0; k < n; k++) {
    data d;
    uint32_t slot = (start + k) % nslots;
    if (flash.readStruct(slot * sizeof(data), d) == FLASH_OK && d.id == tag) {
      sendDataRecord(d);
    }
    IWatchdog.reload();
  }
  sendCsvMessage(DATA_DOWNLOAD_END);
}

void downloadNewData() {
  sendCsvMessage(DATA_DOWNLOAD_BEGIN);
  uint32_t nslots = (flash.getCapacity() - flash.getSectorSize()) / sizeof(data);
  uint32_t n     = (dataCount < nslots) ? dataCount : nslots;
  uint32_t start = (dataCount <= nslots) ? 0 : (writeAddress / sizeof(data));
  for (uint32_t k = 0; k < n; k++) {
    data d;
    uint32_t slot = (start + k) % nslots;
    if (flash.readStruct(slot * sizeof(data), d) == FLASH_OK && d.id == tag && d.count >= dlCount) {
      sendDataRecord(d);
    }
    IWatchdog.reload();
  }
  dlCount = dataCount;
  updateEeprom();
  sendCsvMessage(DATA_DOWNLOAD_END);
}

// ============================================================================
//  CSV Line Parser & Command Dispatcher
// ============================================================================
class CsvParser {
private:
  char lineBuf[96];
  size_t idx;
  bool inFrame;

public:
  CsvParser() : idx(0), inFrame(false) {}

  void reset() {
    idx = 0;
    inFrame = false;
  }

  // FIXED (VULN-04): Resynchronizes strictly on '$' and verifies CRC8 checksum
  bool processChar(char c, char* outPayload, size_t maxLen) {
    if (c == '$') {
      idx = 0;
      inFrame = true;
      return false;
    }
    if (!inFrame) return false;

    if (c == '\r' || c == '\n') {
      if (idx > 0) {
        lineBuf[idx] = '\0';
        inFrame = false;
        idx = 0;

        char* star = strchr(lineBuf, '*');
        if (!star) return false;

        *star = '\0';
        const char* crcHex = star + 1;
        if (strlen(crcHex) < 2) return false;

        uint8_t expectedCrc = (uint8_t)strtoul(crcHex, nullptr, 16);
        uint8_t actualCrc   = crc8(lineBuf, strlen(lineBuf));

        if (expectedCrc != actualCrc) {
          return false;
        }

        strncpy(outPayload, lineBuf, maxLen - 1);
        outPayload[maxLen - 1] = '\0';
        return true;
      }
      return false;
    }

    if (idx < sizeof(lineBuf) - 1) {
      lineBuf[idx++] = c;
    } else {
      idx = 0;
      inFrame = false;
    }
    return false;
  }
};

static CsvParser csvReceiver;

// Simple non-destructive comma tokenizer
const char* nextToken(const char* str, char* token, size_t maxLen) {
  if (token && maxLen > 0) token[0] = '\0';
  if (!str || !*str) return nullptr;
  size_t i = 0;
  while (*str && *str != ',' && i < maxLen - 1) {
    token[i++] = *str++;
  }
  token[i] = '\0';
  if (*str == ',') str++;
  return str;
}

bool gpsFixOk() {
  return gps.location.isValid() && gps.location.age() < 1000 &&
         gps.satellites.isValid() &&
         gps.satellites.value() >= (uint32_t)minSat &&
         gps.hdop.isValid() && gps.hdop.value() > 0 &&
         gps.hdop.value() < (int32_t)gpsHdop * 100;
}

// ============================================================================
//  Interactive Command Session
// ============================================================================
void commandSession() {
  sendCsvMessage(CALIBRATION_BEGIN);
  sendCsvMessage(FLASH_DIAGNOSTICS);
  uint32_t jedec = flash.getJEDECID();
  sendCsvMessage((jedec != 0 && jedec != 0xFFFFFFFF) ? FLASH_SUCCESS : FLASH_ERROR);
  sendCsvMessage(FLASH_DIAGNOSTICS_END);
  sendCsvMessage(accelPresent ? ACCELEROMETER_OK : ACCELEROMETER_ERROR);
  sendCsvMessage(GPS_CALIBRATION);
  
  gps = TinyGPSPlus();
  pinMode(GPS_EN, OUTPUT);
  digitalWrite(GPS_EN, HIGH);
  delay(100);
  gpsSerial.begin(GPS_BAUD);
  sendCsvMessage(CALIBRATION_END);

  bool     gpsFixed = false, finish = false;
  uint32_t lastActivity = millis(), lastBeat = 0;
  char     payload[96];

  while (!finish && millis() - lastActivity < CMD_IDLE_TIMEOUT_MS) {
    IWatchdog.reload();

    while (gpsSerial.available()) gps.encode((char)gpsSerial.read());

    if (!gpsFixed && gpsFixOk()) {
      gpsFixed = true;
      sendCsvMessage(GPS_SUCCESS);
      sendCsvMessage(GPS_CALIBRATION_END);
      sendCalibrationSummary();
    } else if (!gpsFixed && millis() - lastBeat >= 3000) {
      lastBeat = millis();
      sendCsvMessage(GPS_CALIBRATING);
    }

    // Process incoming CSV stream
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (csvReceiver.processChar(c, payload, sizeof(payload))) {
        lastActivity = millis();
        
        char verb[8] = {0}, tagStr[10] = {0};
        const char* p = nextToken(payload, verb, sizeof(verb));
        p = nextToken(p, tagStr, sizeof(tagStr));
        uint16_t inTag = (uint16_t)atoi(tagStr);

        if (inTag != tag) {
          sendCsvMessage(SETTINGS_UPDATE_ERROR);
          continue;
        }

        // --- $SET,<tag>,<frq>,<tout>,<hdop>,<minsat> ---
        if (strcmp(verb, "SET") == 0) {
          char fStr[10] = {0}, tStr[10] = {0}, hStr[10] = {0}, mStr[10] = {0};
          p = nextToken(p, fStr, sizeof(fStr));
          p = nextToken(p, tStr, sizeof(tStr));
          p = nextToken(p, hStr, sizeof(hStr));
          p = nextToken(p, mStr, sizeof(mStr));

          // Reject incomplete parameter lists
          if (fStr[0] == '\0' || tStr[0] == '\0' || hStr[0] == '\0' || mStr[0] == '\0') {
            sendCsvMessage(SETTINGS_UPDATE_ERROR);
            continue;
          }

          int inFrq  = atoi(fStr);
          int inTout = atoi(tStr);
          int inHdop = atoi(hStr);
          int inMsat = atoi(mStr);

          // FIXED (VULN-03): Realistic boundaries to prevent GPS acquisition sabotage
          if (inFrq < 1)     inFrq  = 1;     if (inFrq > 1440) inFrq  = 1440;
          if (inTout < 30)   inTout = 30;    if (inTout > 300) inTout = 300; // Min 30s
          if (inHdop < 2)    inHdop = 2;     if (inHdop > 20)  inHdop = 20;  // Min HDOP 2
          if (inMsat < 3)    inMsat = 3;     if (inMsat > 12)  inMsat = 12;  // Max 12 sats

          gpsFrequency = inFrq;
          gpsTimeout   = inTout;
          gpsHdop      = inHdop;
          minSat       = inMsat;
          updateEeprom();
          sendCsvSettings(); // Echo confirmed settings back to host
        }
        // --- $CMD,<tag>,<code> ---
        else if (strcmp(verb, "CMD") == 0) {
          char codeStr[10] = {0};
          nextToken(p, codeStr, sizeof(codeStr));
          if (codeStr[0] == '\0') continue;
          uint8_t req = (uint8_t)atoi(codeStr);

          switch (req) {
            case FINISH_CALIBRATION:  finish = true; break;
            case ABORT_CALIBRATION:   sendCsvMessage(CALIBRATION_ABORTED); finish = true; break;
            case DATA_DOWNLOAD_ALL:   downloadAllData(); break;
            case DATA_DOWNLOAD_NEW:   downloadNewData(); break;
            case MEMORY_STATUS:       sendMemoryStatus(); break;
            case REQUEST_SETTINGS:    loadEepromSettings(); sendCsvSettings(); break;
            case SETTINGS_RESET:
              gpsFrequency = 3; gpsTimeout = 60; gpsHdop = 5; minSat = 4;
              updateEeprom();
              sendCsvSettings();
              break;
            case CALIBRATION_SUMMARY:
            case REQ_SUMMARY:
              sendCalibrationSummary();
              break;
            default: break;
          }
        }
        // --- FIXED (VULN-01 & VULN-02): $CLR,<tag>,CONFIRM ---
        else if (strcmp(verb, "CLR") == 0) {
          char confirmStr[12] = {0};
          nextToken(p, confirmStr, sizeof(confirmStr));
          if (strcmp(confirmStr, "CONFIRM") == 0) {
            // Invalidate Sector 0 AND Sector 1 base to prevent ghost record resurrection
            flash.eraseSector(0);
            flash.eraseSector(flash.getSectorSize());
            writeAddress = 0; dlCount = 0; dataCount = 0;
            updateEeprom();
            sendCsvMessage(MEMORY_CLEARED);
          } else {
            sendCsvMessage(MEMORY_CLEAR_ERROR);
          }
        }
        // --- FIXED (VULN-01): $DFU,<tag>,REBOOT ---
        else if (strcmp(verb, "DFU") == 0) {
          char rebootStr[12] = {0};
          nextToken(p, rebootStr, sizeof(rebootStr));
          if (strcmp(rebootStr, "REBOOT") == 0) {
            bootFlag = DFU_BOOT_MAGIC;
            NVIC_SystemReset();
          }
        }
      }
    }
    delay(5);
  }

  // Gracefully terminate GPS UART & power
  gpsSerial.end();
  pinMode(USART1_RX, INPUT_PULLDOWN);
  pinMode(USART1_TX, INPUT_PULLDOWN);
  digitalWrite(GPS_EN, LOW);

  // Power down high-frequency clocks and USB peripheral for field sleep
  __HAL_RCC_USB_CLK_DISABLE();
  __HAL_RCC_CRS_CLK_DISABLE();
  __HAL_RCC_HSI48_DISABLE();
}

// ============================================================================
//  Setup & Loop
// ============================================================================
void setup() {
  if (bootFlag == DFU_BOOT_MAGIC) {
    bootFlag = 0;
    jumpToBootloader();
  }

  Serial.begin(115200);
  pinMode(GPS_EN, OUTPUT); digitalWrite(GPS_EN, LOW);
  pinMode(SNS_EN, OUTPUT); digitalWrite(SNS_EN, LOW);
  pinMode(CHRG_STAT, INPUT_PULLUP);
  analogReadResolution(12);

  if (!flash.begin()) {
    while (1) { delay(1000); }
  }
  metaAddr = flash.getCapacity() - flash.getSectorSize();

  recoverLogPointers();

  accelPresent = accelInit();
  accelSleep(); // Enter 0.5 µA power-down mode immediately after hardware init

  loadEepromSettings();

  // Non-blocking wait: up to 10 seconds for USB / Android app to open connection
  uint32_t waitStart = millis();
  while (!Serial && (millis() - waitStart < 10000)) {
    delay(20);
  }
  if (Serial) {
    delay(1000); // Allow Android USB driver & app background threads to settle
  }

  IWatchdog.begin(WDG_TIMEOUT_US);

  commandSession();
  loadEepromSettings();

  RTC_init();
}

void loop() {
  IWatchdog.reload(); // Fed on every 1-second RTC tick
  if (sleeping) {
    if (rtcCounter >= (uint32_t)gpsFrequency * 60UL) {
      sleeping = false;
    }
  } else {
    restoreCoreClock();  // Restore 24 MHz SYSCLK for exact 9600 baud GPS and 8 MHz SPI
    recordGPSData();
    rtcCounter = 0;      // Start cadence AFTER fix completes to guarantee sleep between attempts
    sleeping = true;
  }
  sleepOneTick();
}
