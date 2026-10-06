/*
 * reader.cpp - standalone W25Q flash reader / diagnostic (not the app).
 *
 * Type a number in the serial monitor and press Enter; the device reads that many
 * records from flash starting at address 0 (ignores writeAddress) and prints them,
 * plus the raw metadata record. Lets us see exactly what's stored, independent of
 * the app's pointers.
 *
 * Build: set  build_src_filter = +<reader.cpp>  in platformio.ini, then upload.
 * Restore  +<main.cpp>  afterwards.
 */
#include <Arduino.h>
#include <SPI.h>
#include "definitions.h"          // data / meta structs, pins
#include "LoRaE5_SPIFlash.h"

SPIClass        flashSPI(SPI1_MOSI, SPI1_MISO, SPI1_SCK);
LoRaE5_SPIFlash flash(SPI1_NSS, &flashSPI, 8000000);

// ---- Clock: 24 MHz + HSI48/CRS for USB CDC ---------------------------------
extern "C" void SystemClock_Config(void) {
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};
  RCC_PeriphCLKInitTypeDef pk = {0};
  osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
  osc.HSIState            = RCC_HSI_ON;
  osc.HSIDiv              = RCC_HSI_DIV2;
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

static uint32_t metaAddr = 0;

static void printMeta() {
  meta m;
  flash.readStruct(metaAddr, m);
  Serial.print(F("meta @0x")); Serial.print(metaAddr, HEX);
  Serial.print(F("  gfrq=")); Serial.print(m.gfrq);
  Serial.print(F(" gto="));   Serial.print(m.gto);
  Serial.print(F(" hdop="));  Serial.print(m.hdop);
  Serial.print(F(" minSat="));Serial.print(m.msat);
  Serial.print(F(" count=")); Serial.print(m.count);
  Serial.print(F(" wa="));    Serial.print(m.wa);
  Serial.print(F(" ra="));    Serial.println(m.ra);
  Serial.print(F("sizeof(data)=")); Serial.print((uint32_t)sizeof(data));
  Serial.print(F("  sizeof(meta)=")); Serial.println((uint32_t)sizeof(meta));
}

static void readRecords(uint32_t n) {
  Serial.print(F("--- reading ")); Serial.print(n); Serial.println(F(" records (from addr 0) ---"));
  for (uint32_t i = 0; i < n; i++) {
    uint32_t addr = i * (uint32_t)sizeof(data);
    data d;
    if (flash.readStruct(addr, d) != FLASH_OK) {
      Serial.print(F("read FAIL @")); Serial.println(addr);
      break;
    }
    Serial.print(F("[")); Serial.print(i); Serial.print(F("] @")); Serial.print(addr);
    Serial.print(F(" cnt="));  Serial.print(d.count);
    Serial.print(F(" id="));   Serial.print(d.id);
    Serial.print(F(" dt="));   Serial.print(d.datetime);
    Serial.print(F(" lat="));  Serial.print(d.lat, 6);
    Serial.print(F(" lng="));  Serial.print(d.lng, 6);
    Serial.print(F(" hdop=")); Serial.print(d.hdop, 2);
    Serial.print(F(" sats=")); Serial.print(d.sats);
    Serial.print(F(" lck="));  Serial.print(d.locktime);
    Serial.println();
  }
  Serial.println(F("--- done ---"));
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println();
  Serial.println(F("=== ArcTrack Flash Reader ==="));
  if (!flash.begin()) { Serial.println(F("FLASH INIT FAILED")); while (1) delay(1000); }
  metaAddr = flash.getCapacity() - flash.getSectorSize();
  Serial.print(F("Flash: ")); Serial.print(flash.getManufacturer());
  Serial.print(' ');          Serial.print(flash.getModel());
  Serial.print(F("  cap="));  Serial.print(flash.getCapacity());
  Serial.print(F("  sector=")); Serial.println(flash.getSectorSize());
  printMeta();
  Serial.print(F("\nrecords to read> "));
}

void loop() {
  static char buf[16];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (len) {
        buf[len] = 0; len = 0;
        readRecords((uint32_t)atoi(buf));
        Serial.print(F("\nrecords to read> "));
      }
    } else if (len < sizeof(buf) - 1 && c >= ' ') {
      buf[len++] = c;
    }
  }
}
