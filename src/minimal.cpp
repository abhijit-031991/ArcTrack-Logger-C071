/*
 * minimal.cpp — bare USB CDC bring-up, nothing else.
 *
 * Purpose: isolate board B's non-enumeration. This has the SAME crystal-less
 * clock config as the test harness, but NO flash library, NO RTC, NO global
 * objects, NO SPI. If this enumerates on board B, the problem is something in
 * the fuller firmware; if it does NOT, it's the clock/USB bring-up itself on
 * that board.
 */

#include <Arduino.h>

extern "C" void SystemClock_Config(void) {
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};
  RCC_PeriphCLKInitTypeDef periph = {0};

  osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
  osc.HSIState            = RCC_HSI_ON;
  osc.HSIDiv              = RCC_HSI_DIV2;               // 24 MHz
  osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  osc.HSI48State          = RCC_HSI48_ON;
  HAL_RCC_OscConfig(&osc);

  clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1;
  clk.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
  clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0);

  periph.PeriphClockSelection = RCC_PERIPHCLK_USB;
  periph.UsbClockSelection    = RCC_USBCLKSOURCE_HSI48;
  HAL_RCCEx_PeriphCLKConfig(&periph);

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

static uint32_t tick = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println();
  Serial.println(F("==== MINIMAL USB sketch ===="));
  Serial.println(F("No flash lib, no RTC, no globals, no SPI."));
  Serial.print  (F("SYSCLK: "));
  Serial.print  (SystemCoreClock / 1000000UL);
  Serial.println(F(" MHz, USB=HSI48+CRS"));
  Serial.println(F("If you can read this, a bare USB CDC enumerates on this board."));
}

void loop() {
  Serial.print(F("[min] tick "));
  Serial.println(tick++);
  delay(1000);
}
