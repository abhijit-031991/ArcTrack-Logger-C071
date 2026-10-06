/*
 * test.cpp — USB CDC serial bring-up test / on-target harness seed
 *
 * Target: STM32C071K8 (Cortex-M0+, USB 2.0 FS device, crystal-less).
 *
 * Clocking (crystal-less):
 *   The generic C071K variant ships an EMPTY SystemClock_Config(), so the
 *   part would otherwise run at the ~12 MHz reset default with USB unclocked
 *   and CRS off -> USB out of spec ("device malfunctioned"). We override
 *   SystemClock_Config() below to run SYSCLK at 24 MHz from HSI, clock USB
 *   from HSIUSB48, and enable CRS to trim HSI48 against USB SOF. (24 MHz, not
 *   48: at 48 MHz rail ripple detunes the crystal-less HSI48; 24 MHz is
 *   cold-boot-stable 10/10.)
 *
 * USB console:
 *   With PIO_FRAMEWORK_ARDUINO_ENABLE_CDC, `Serial` is the USB virtual COM
 *   port. This sketch prints a banner + 1 Hz heartbeat (TX) and echoes /
 *   dispatches commands (RX). Command 'b' reboots into the ROM DFU
 *   bootloader over USB, so we can reflash without the BOOT0 jumper.
 *
 * On BOOT0: on the C071, BOOT0 is PA14 = SWCLK. Flash-empty always enters
 * DFU; with an app present, PA14 high at reset enters DFU.
 */

#include <Arduino.h>
#include <SPI.h>
#include <TinyGPS++.h>     // NMEA parser for the GPS on USART1
#include "LoRaE5_SPIFlash.h"   // W25Q driver (chosen for the logger port)
#include "definitions.h"   // board pin map (CHRG_STAT, SPI1_*, etc.)

// ---- Configuration ---------------------------------------------------------
static const uint32_t HEARTBEAT_MS  = 1000;  // heartbeat cadence
static const uint32_t HOST_WAIT_MS  = 5000;  // max wait for host to open port
static const uint32_t FIRST_PRINT_DELAY_MS = 3000;  // quiet time before first output

// ---- Baud benchmark --------------------------------------------------------
// NOTE: Serial here is USB CDC, so the baud value is nominal - the CDC link
// runs at USB Full-Speed regardless. This benchmark demonstrates exactly that:
// times should be ~equal across all baud rates (a real UART would scale with baud).
static const uint32_t kBauds[] = {
  1200, 2400, 4800, 9600, 19200, 38400, 57600,
  115200, 230400, 460800, 921600, 1000000, 2000000
};
static const uint8_t  kNumBauds   = sizeof(kBauds) / sizeof(kBauds[0]);
static const uint16_t kPrintCount = 100;

// ---- DFU re-entry ----------------------------------------------------------
// Magic left in a no-init RAM word survives a warm reset; setup() checks it
// before USB comes up and jumps to the system bootloader if set.
#define DFU_BOOT_MAGIC 0xB00710ADUL
static uint32_t bootFlag __attribute__((section(".noinit"), used));
static const uint32_t SYSMEM_BASE = 0x1FFF0000UL;  // STM32C0 system memory

// ---- State -----------------------------------------------------------------
static uint32_t lastBeat  = 0;
static uint32_t beatCount = 0;

// ---- Clock: 48 MHz SYSCLK + HSI48/CRS for USB ------------------------------
extern "C" void SystemClock_Config(void) {
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};
  RCC_PeriphCLKInitTypeDef periph = {0};

  // Run the core at 24 MHz (HSI / 2). This is the validated cold-boot-stable
  // config (10/10 enumeration): 48 MHz detunes HSI48 via rail ripple on this
  // crystal-less design. NOTE: the board-B non-enumeration was NOT a clock
  // issue after all -- it was GPS-backup switch inrush sagging the rail at the
  // reset-vector fetch (fixed in HW: MIC94071 -> MIC94073 soft-start). See
  // memory: board-b-gps-backup-inrush.
  osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
  osc.HSIState            = RCC_HSI_ON;
  osc.HSIDiv              = RCC_HSI_DIV2;               // HSISYS = 24 MHz
  osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  osc.HSI48State          = RCC_HSI48_ON;              // USB source RC (48 MHz)
  HAL_RCC_OscConfig(&osc);

  // SYSCLK = HSISYS (24 MHz) -> 0 wait states.
  clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1;
  clk.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
  clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0);

  // USB kernel clock from HSIUSB48 (48 MHz).
  periph.PeriphClockSelection = RCC_PERIPHCLK_USB;
  periph.UsbClockSelection    = RCC_USBCLKSOURCE_HSI48;
  HAL_RCCEx_PeriphCLKConfig(&periph);

  // CRS: auto-trim HSI48 against USB start-of-frame so USB stays in spec
  // across temperature/voltage without a crystal.
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

// ---- Jump to ROM bootloader (for DFU) --------------------------------------
static void jumpToBootloader() {
  __disable_irq();
  HAL_RCC_DeInit();
  HAL_DeInit();
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL  = 0;
  SCB->VTOR = SYSMEM_BASE;
  uint32_t sp = *(volatile uint32_t *)SYSMEM_BASE;
  uint32_t pc = *(volatile uint32_t *)(SYSMEM_BASE + 4U);
  __set_MSP(sp);
  void (*bootJump)(void) = (void (*)(void))pc;
  bootJump();
  while (1) { /* never returns */ }
}

// ---- Baud benchmark --------------------------------------------------------
// For each common baud rate: set it, print kPrintCount lines, and time how long
// the 100 prints take (including flush, so the timing reflects real delivery).
static void runBaudBenchmark() {
  Serial.println();
  Serial.println(F("==== BAUD BENCHMARK (Serial = USB CDC) ===="));
  Serial.print  (F("Printing "));
  Serial.print  (kPrintCount);
  Serial.println(F("x \"Hello from Arcturus Inc.\" per baud rate."));
  Serial.println(F("Over USB CDC the baud is nominal - USB FS sets the real rate."));
  Serial.println(F("-------------------------------------------"));
  Serial.flush();

  for (uint8_t b = 0; b < kNumBauds; b++) {
    Serial.flush();
    Serial.begin(kBauds[b]);   // nominal for USB CDC; would matter on a UART
    delay(20);

    uint32_t t0 = micros();
    for (uint16_t i = 0; i < kPrintCount; i++) {
      Serial.println(F("Hello from Arcturus Inc."));
    }
    Serial.flush();            // wait until actually delivered to the host
    uint32_t dtUs = micros() - t0;

    Serial.print(F("RESULT baud="));
    Serial.print(kBauds[b]);
    Serial.print(F(" : "));
    Serial.print(kPrintCount);
    Serial.print(F(" prints in "));
    Serial.print(dtUs);
    Serial.print(F(" us ("));
    Serial.print((float)dtUs / 1000.0f, 2);
    Serial.println(F(" ms)"));
    Serial.flush();
    delay(50);
  }

  Serial.begin(115200);        // restore a sane nominal baud
  Serial.println(F("==== BENCHMARK DONE ===="));
  Serial.flush();
}

// ---- SPI flash (W25Q) ID test ----------------------------------------------
// SPI1 from definitions.h: MOSI=PB5, MISO=PB4, SCK=PB3, CS(NSS)=PA15.
static SPIClass flashSPI(SPI1_MOSI, SPI1_MISO, SPI1_SCK);

// A stand-in for the kind of fixed-width record the logger will store, used to
// exercise the library's writeStruct/readStruct template path.
struct FlashTestRec {
  uint32_t magic;
  uint32_t datetime;
  float    lat;
  float    lng;
  uint16_t count;
} __attribute__((packed));

// Full bring-up test of the LoRaE5_SPIFlash driver on our SPI1 + CS PA15:
// identify -> erase a scratch sector -> array write/read -> struct write/read.
// Uses the LAST sector as scratch so low addresses stay untouched.
static void flashTest() {
  Serial.println(F("---- SPI FLASH (LoRaE5_SPIFlash) ----"));
  LoRaE5_SPIFlash flash(SPI1_NSS, &flashSPI, 8000000);  // 8 MHz

  if (!flash.begin()) {
    Serial.println(F("[flash] begin() FAILED - check CS/wiring/power"));
    return;
  }
  Serial.print(F("[flash] "));
  Serial.print(flash.getManufacturer()); Serial.print(' ');
  Serial.print(flash.getModel());
  Serial.print(F("  JEDEC=0x")); Serial.print(flash.getJEDECID(), HEX);
  Serial.print(F("  ")); Serial.print(flash.getCapacity()); Serial.println(F(" bytes"));

  const uint32_t testAddr = flash.getCapacity() - 2 * flash.getSectorSize();  // 2nd-last (last = metadata)
  Serial.print(F("[flash] scratch sector @ 0x")); Serial.println(testAddr, HEX);

  // 1) Erase + blank check
  if (flash.eraseSector(testAddr) != FLASH_OK) { Serial.println(F("[flash] ERASE: FAIL")); return; }
  bool blank = flash.isBlank(testAddr, 256);
  Serial.print(F("[flash] erase    : ")); Serial.println(blank ? F("PASS (blank)") : F("FAIL (not blank)"));

  // 2) Array write/read
  uint8_t wr[16], rd[16];
  for (uint8_t i = 0; i < 16; i++) wr[i] = (uint8_t)(i * 17 + 3);
  bool arrOk = (flash.writeArray(testAddr, wr, 16) == FLASH_OK) &&
               (flash.readArray(testAddr, rd, 16) == FLASH_OK) &&
               (memcmp(wr, rd, 16) == 0);
  Serial.print(F("[flash] array    : ")); Serial.println(arrOk ? F("PASS") : F("FAIL"));

  // 3) Struct write/read (the path the logger records will use)
  FlashTestRec recW = { 0xA5A5A5A5UL, 1719000000UL, 12.9716f, 77.5946f, 42 };
  FlashTestRec recR = {0, 0, 0, 0, 0};
  const uint32_t recAddr = testAddr + 64;
  bool recOk = (flash.writeStruct(recAddr, recW) == FLASH_OK) &&
               (flash.readStruct(recAddr, recR) == FLASH_OK) &&
               (recR.magic == recW.magic && recR.datetime == recW.datetime &&
                recR.lat == recW.lat && recR.lng == recW.lng && recR.count == recW.count);
  Serial.print(F("[flash] struct   : ")); Serial.println(recOk ? F("PASS") : F("FAIL"));
  if (recOk) {
    Serial.print(F("[flash]   readback lat=")); Serial.print(recR.lat, 4);
    Serial.print(F(" lng=")); Serial.print(recR.lng, 4);
    Serial.print(F(" count=")); Serial.println(recR.count);
  }

  Serial.print(F("[flash] RESULT   : "));
  Serial.println((blank && arrOk && recOk) ? F("ALL PASS") : F("FAIL"));
}

// ---- Metadata store (reserved top W25Q sector) -----------------------------
// Rolling-record scheme in the LAST 4 KB sector: each save writes a fresh record
// into the next free 64-byte slot; the current record is the valid one with the
// highest seq; when the sector fills it is erased and we start over. Wear-levels
// writes and survives power cycles. (Caveat: a power loss in the small window
// between the fill-erase and the next write loses metadata -> a two-sector
// ping-pong would remove that window; fine for bring-up.)
struct MetaRecord {
  uint32_t magic;
  uint32_t seq;          // monotonic; highest valid = current
  uint16_t version;      // MetaRecord format version (bump on any layout change)
  uint16_t minSat;       // setting: min satellites required to record a fix
  uint32_t writeAddr;    // log append pointer
  uint32_t readAddr;     // download pointer
  uint32_t dataCount;    // records stored
  int32_t  gpsFrequency; // --- settings ---
  int32_t  gpsTimeout;
  int32_t  gpsHdop;
  uint32_t checksum;     // sum of the 9 words above
};
#define META_MAGIC   0x41544D54UL // "ATMT"
#define META_VERSION 1            // current MetaRecord format version
#define META_SLOT    64           // bytes per slot (>= sizeof(MetaRecord)=40)

static uint32_t metaChecksum(const MetaRecord &r) {
  const uint32_t *w = (const uint32_t *)&r;
  uint32_t sum = 0;
  for (int i = 0; i < 9; i++) sum += w[i];   // magic..gpsHdop (all words before checksum)
  return sum;
}

// Scan the metadata sector: true if a valid record exists (fills `out` with the
// highest-seq one); `nextSlot` = next free slot index.
static bool metaScan(LoRaE5_SPIFlash &flash, MetaRecord &out, int &nextSlot) {
  uint32_t base  = flash.getCapacity() - flash.getSectorSize();
  int      slots = flash.getSectorSize() / META_SLOT;
  bool     found = false;
  uint32_t bestSeq = 0;
  int      lastUsed = -1;
  for (int i = 0; i < slots; i++) {
    MetaRecord r;
    if (flash.readStruct(base + (uint32_t)i * META_SLOT, r) != FLASH_OK) continue;
    if (r.magic == META_MAGIC && metaChecksum(r) == r.checksum) {
      lastUsed = i;
      if (!found || r.seq > bestSeq) { bestSeq = r.seq; out = r; found = true; }
    }
  }
  nextSlot = lastUsed + 1;
  return found;
}

// Save a new metadata record (magic/seq/checksum managed here).
static bool metaSave(LoRaE5_SPIFlash &flash, MetaRecord r) {
  uint32_t base  = flash.getCapacity() - flash.getSectorSize();
  int      slots = flash.getSectorSize() / META_SLOT;
  MetaRecord cur; int nextSlot;
  uint32_t newSeq = metaScan(flash, cur, nextSlot) ? cur.seq + 1 : 1;
  if (nextSlot < 0) nextSlot = 0;

  // Erase-before-write: erase the sector if it is full, OR if the target slot
  // is not already blank (handles a fresh/stale sector on first use -- flash
  // can only clear bits, so writing over non-0xFF fails verification).
  bool needErase = (nextSlot >= slots) ||
                   !flash.isBlank(base + (uint32_t)nextSlot * META_SLOT, sizeof(MetaRecord));
  if (needErase) {
    if (flash.eraseSector(base) != FLASH_OK) return false;
    nextSlot = 0;
  }

  r.magic    = META_MAGIC;
  r.version  = META_VERSION;
  r.seq      = newSeq;
  r.checksum = metaChecksum(r);
  return flash.writeStruct(base + (uint32_t)nextSlot * META_SLOT, r) == FLASH_OK;
}

// Read-only: print current metadata (called at boot to show persistence).
static void metaShow() {
  LoRaE5_SPIFlash flash(SPI1_NSS, &flashSPI, 8000000);
  if (!flash.begin()) { Serial.println(F("[meta] flash begin FAILED")); return; }
  MetaRecord cur; int nextSlot;
  if (metaScan(flash, cur, nextSlot)) {
    Serial.print(F("[meta] current: v")); Serial.print(cur.version);
    Serial.print(F(" seq=")); Serial.print(cur.seq);
    Serial.print(F(" count=")); Serial.print(cur.dataCount);
    Serial.print(F(" wa=")); Serial.print(cur.writeAddr);
    Serial.print(F(" ra=")); Serial.print(cur.readAddr);
    Serial.print(F(" freq=")); Serial.print(cur.gpsFrequency);
    Serial.print(F(" minSat=")); Serial.print(cur.minSat);
    Serial.print(F(" nextSlot=")); Serial.println(nextSlot);
  } else {
    Serial.println(F("[meta] no record yet (fresh sector)"));
  }
}

// 'm' command: load -> simulate one log write -> save -> reload -> verify.
static void metaTest() {
  Serial.println(F("---- METADATA (reserved top sector) ----"));
  LoRaE5_SPIFlash flash(SPI1_NSS, &flashSPI, 8000000);
  if (!flash.begin()) { Serial.println(F("[meta] flash begin FAILED")); return; }

  MetaRecord rec; int nextSlot;
  bool have = metaScan(flash, rec, nextSlot);
  if (!have) { rec = MetaRecord{}; rec.gpsFrequency = 3; rec.gpsTimeout = 60; rec.gpsHdop = 5; rec.minSat = 4; }
  Serial.print(F("[meta] before: "));
  if (have) { Serial.print(F("seq=")); Serial.print(rec.seq); Serial.print(F(" count=")); Serial.println(rec.dataCount); }
  else      { Serial.println(F("(fresh)")); }

  rec.dataCount += 1;       // pretend one record was logged
  rec.writeAddr += 36;      // pretend 36 bytes appended

  if (!metaSave(flash, rec)) { Serial.println(F("[meta] SAVE FAILED")); return; }

  MetaRecord back; int ns2;
  bool ok = metaScan(flash, back, ns2) &&
            back.dataCount == rec.dataCount &&
            back.writeAddr == rec.writeAddr &&
            back.gpsFrequency == rec.gpsFrequency;
  Serial.print(F("[meta] after : v")); Serial.print(back.version);
  Serial.print(F(" seq=")); Serial.print(back.seq);
  Serial.print(F(" count=")); Serial.print(back.dataCount);
  Serial.print(F(" wa=")); Serial.print(back.writeAddr);
  Serial.print(F(" minSat=")); Serial.print(back.minSat);
  Serial.print(F(" nextSlot=")); Serial.println(ns2);
  Serial.print(F("[meta] verify: ")); Serial.println(ok ? F("PASS") : F("FAIL"));
}

// ---- Wake/sleep scheduler (RTC masked-alarm 1 Hz tick) ---------------------
// The C0 RTC has NO wake-up timer and the chip has NO LPTIM, so the periodic
// wake source is Alarm A with ALL fields masked -> it matches every second.
// Each cycle we just clear the ALRAF flag (no re-arm math, no set-in-past race).
// Real firmware: this alarm (with _IT) wakes the MCU from STOP. Here we poll the
// flag in RUN mode so the USB console stays alive to watch the state machine.
static RTC_HandleTypeDef hrtc;
static bool rtcBaseReady = false;
static volatile uint32_t rtcTicks = 0;

// RTC alarm ISR. On the C0 the RTC uses RTC_IRQn (EXTI line 19); this strong
// definition overrides the weak default handler. Clearing ALRAF also clears
// the direct EXTI line, so the alarm re-fires next second with no re-arm.
extern "C" void RTC_IRQHandler(void) {
  if (__HAL_RTC_ALARM_GET_FLAG(&hrtc, RTC_FLAG_ALRAF)) {
    __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);
    rtcTicks++;
  }
}

static bool rtcBaseInit() {
  if (rtcBaseReady) return true;
  __HAL_RCC_LSI_ENABLE();                      // C0: no DBP backup-domain lock
  uint32_t t0 = millis();
  while (!__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY)) { if (millis() - t0 > 100) return false; }

  RCC_PeriphCLKInitTypeDef pk = {0};
  pk.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  pk.RTCClockSelection    = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&pk) != HAL_OK) return false;
  __HAL_RCC_RTC_ENABLE();
  __HAL_RCC_RTCAPB_CLK_ENABLE();

  hrtc.Instance            = RTC;              // LSI ~32 kHz: 128 * 250 = 32000 -> 1 Hz
  hrtc.Init.HourFormat     = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv   = 127;
  hrtc.Init.SynchPrediv    = 249;
  hrtc.Init.OutPut         = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap    = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp   = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK) return false;

  rtcBaseReady = true;
  return true;
}

// Alarm A masked to fire every second. withIT=true enables the interrupt + EXTI
// (so it can wake the MCU from STOP); false = flag-poll only (RUN-mode test).
static void rtcSetMaskedAlarm(bool withIT) {
  RTC_AlarmTypeDef a = {0};
  a.Alarm               = RTC_ALARM_A;
  a.AlarmMask           = RTC_ALARMMASK_ALL;          // match every second
  a.AlarmSubSecondMask  = RTC_ALARMSUBSECONDMASK_ALL;
  a.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  a.AlarmDateWeekDay    = 1;
  HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
  if (withIT) {
    HAL_NVIC_SetPriority(RTC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_IRQn);
    HAL_RTC_SetAlarm_IT(&hrtc, &a, RTC_FORMAT_BIN);
    __HAL_RTC_ALARM_EXTI_ENABLE_IT();
  } else {
    HAL_RTC_SetAlarm(&hrtc, &a, RTC_FORMAT_BIN);
  }
}

static bool rtcAlarmFired() { return __HAL_RTC_ALARM_GET_FLAG(&hrtc, RTC_FLAG_ALRAF) != 0U; }
static void rtcAlarmClear() { __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF); }

// 's' command: run the tick -> accumulate -> log state machine for 2 cycles.
static void sleepSchedTest() {
  Serial.println(F("---- WAKE/SLEEP SCHED (RTC masked-alarm 1 Hz, RUN mode) ----"));
  if (!rtcBaseInit()) { Serial.println(F("[sched] RTC init FAILED")); return; }
  rtcSetMaskedAlarm(false);        // poll mode (no IT) for the RUN-mode demo
  const uint32_t interval = 10;   // demo LOG interval (s); real = gpsFrequency*60
  Serial.print(F("[sched] 1 Hz tick, LOG every ")); Serial.print(interval);
  Serial.println(F(" s, 2 cycles (poll mode; real fw would STOP here)"));

  uint32_t accum = 0, ticks = 0; int logs = 0;
  rtcAlarmClear();
  while (logs < 2) {
    while (!rtcAlarmFired()) { /* STOP in real fw; poll in RUN so USB stays up */ }
    rtcAlarmClear();               // periodic via mask -> no re-arm, just clear
    ticks++; accum++;
    Serial.print(F("[tick] #")); Serial.print(ticks);
    Serial.print(F("  accum=")); Serial.print(accum); Serial.println(F("s  (iwdg kick)"));
    if (accum >= interval) {
      accum = 0; logs++;
      Serial.print(F("[LOG] trigger #")); Serial.print(logs);
      Serial.println(F("  -> would acquire GPS + write record + metaSave"));
    }
  }
  Serial.println(F("[sched] done"));
}

// 'S' command: REAL STOP mode. Enters STOP, woken by the 1 Hz masked alarm
// interrupt, for ~10 s, then restores clocks + USB. USB drops during STOP
// (measure current then); the COM port returns with the summary on wake.
static void realStopTest() {
  Serial.println(F("---- REAL STOP TEST (alarm IT, ~10 s) ----"));
  Serial.println(F("[stop] Entering STOP; USB drops now. Measure current; COM returns in ~10 s."));
  Serial.flush();
  delay(300);

  if (!rtcBaseInit()) { Serial.println(F("[stop] RTC init FAILED")); return; }
  rtcSetMaskedAlarm(true);          // masked alarm A + IT + EXTI (wakes from STOP)
  rtcTicks = 0;

  NVIC_DisableIRQ(USB_DRD_FS_IRQn); // don't let USB SOF/traffic wake STOP
  HAL_SuspendTick();                // don't let SysTick (1 ms) wake STOP

  while (rtcTicks < 10) {           // 10 x 1 s alarm wakes; spurious wakes just re-STOP
    HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
  }

  HAL_ResumeTick();                 // restore SysTick before any HAL timeouts
  SystemClock_Config();             // STOP stopped HSI48/CRS -> restore 24 MHz + USB clock
  NVIC_EnableIRQ(USB_DRD_FS_IRQn);
  Serial.begin(115200);             // re-init USB CDC (host re-enumerates)
  delay(1500);
  Serial.print(F("[stop] Back! woke after "));
  Serial.print(rtcTicks);
  Serial.println(F(" alarm ticks (~10 s in STOP)."));
  Serial.println(F("[stop] STOP + alarm-IT wake + USB recovery OK."));
}

// ---- Helpers ---------------------------------------------------------------
static void printBanner() {
  Serial.println();
  Serial.println(F("====================================="));
  Serial.println(F("  ArcTrack Logger - USB serial test"));
  Serial.println(F("  MCU: STM32C071K8  |  CDC over USB FS"));
  Serial.print  (F("  SYSCLK: "));
  Serial.print  (SystemCoreClock / 1000000UL);
  Serial.println(F(" MHz (HSI), USB=HSI48+CRS"));
  Serial.println(F("====================================="));
  Serial.print(F("Built: "));
  Serial.print(F(__DATE__));
  Serial.print(' ');
  Serial.println(F(__TIME__));
  Serial.println(F("Commands: '?' help | 'p' ping | 'v' version | 'c' charge | 'f' flash | 'm' meta | 'g' gps pwr | 'D' gps raw dump | 'G' gps fix | 'B' gps baud scan | 's' sched | 'S' STOP | 'r' baud bench | 'b' DFU"));
  Serial.println();
}

// TP4057 STDBY (CHRG_STAT) is open-drain, active-low: LOW = charge complete /
// standby; HIGH (via pull-up) = charging OR no charger input (can't tell which
// from STDBY alone). Read with the internal pull-up enabled.
static bool chargeComplete() {
  return digitalRead(CHRG_STAT) == LOW;
}

// ---- GPS power (MIC94073 high-side switch, EN = GPS_EN/PA4, active-high) -----
// The GPS backup rail is gated by a MIC94073 soft-start load switch. It is held
// OFF at boot and only enabled AFTER the rail has settled: on board B the
// switch's turn-on inrush, coinciding with the reset-vector fetch, browned out
// the LDO and locked the core (fixed in HW by the 94071->94073 soft-start swap;
// this sequencing keeps even the soft-started inrush clear of the boot window).
// See memory: board-b-gps-backup-inrush.
static bool gpsPowered = false;
static void gpsPower(bool on) {
  digitalWrite(GPS_EN, on ? HIGH : LOW);
  gpsPowered = on;
}

// ---- GPS: USART1 (TX=PB6, RX=PB7) + TinyGPS++ ------------------------------
// HardwareSerial on the USART1 pins from definitions.h. USB CDC is `Serial`, so
// USART1 is free for the GPS. GPS_BAUD default is the common u-blox 9600; the
// original AVR design ran the module at 115200 -- if 'G' shows no data, run the
// baud scan ('B') to find the module's actual rate, then set GPS_BAUD.
static const uint32_t GPS_BAUD = 9600;
HardwareSerial gpsSerial(USART1_RX, USART1_TX);   // (RX=PB7, TX=PB6) -> USART1
static TinyGPSPlus gps;

// 'G' command: power the GPS, stream NMEA into TinyGPS++ for ~30 s, and print a
// parsed status line every 2 s (sats / HDOP / fix / UTC). Any key stops early.
static void gpsTest() {
  Serial.println(F("---- GPS (USART1 TX=PB6 RX=PB7) + TinyGPS++ ----"));
  gpsPower(true);
  delay(100);                       // let the module rail settle
  gpsSerial.begin(GPS_BAUD);
  Serial.print(F("[gps] @ ")); Serial.print(GPS_BAUD);
  Serial.println(F(" baud; streaming ~30 s (send any key to stop)"));

  uint32_t t0 = millis(), lastReport = 0, chars = 0;
  while (millis() - t0 < 30000) {
    while (gpsSerial.available()) { gps.encode((char)gpsSerial.read()); chars++; }

    if (millis() - lastReport >= 2000) {
      lastReport = millis();
      Serial.print(F("[gps] chars=")); Serial.print(chars);
      Serial.print(F(" sats="));
      if (gps.satellites.isValid()) Serial.print(gps.satellites.value()); else Serial.print(F("--"));
      Serial.print(F(" hdop="));
      if (gps.hdop.isValid()) Serial.print(gps.hdop.hdop(), 2); else Serial.print(F("--"));
      Serial.print(F(" fix="));
      if (gps.location.isValid()) {
        Serial.print(gps.location.lat(), 6); Serial.print(',');
        Serial.print(gps.location.lng(), 6);
      } else {
        Serial.print(F("no"));
      }
      Serial.print(F(" utc="));
      if (gps.time.isValid() && gps.date.isValid()) {
        char buf[24];
        snprintf(buf, sizeof(buf), "%04u-%02u-%02u %02u:%02u:%02u",
                 gps.date.year(), gps.date.month(), gps.date.day(),
                 gps.time.hour(), gps.time.minute(), gps.time.second());
        Serial.print(buf);
      } else {
        Serial.print(F("--"));
      }
      Serial.println();
    }

    if (Serial.available()) { Serial.read(); break; }  // key press -> stop early
  }

  Serial.print(F("[gps] done. chars=")); Serial.print(chars);
  Serial.print(F(" withFix=")); Serial.print(gps.sentencesWithFix());
  Serial.print(F(" cksumFail=")); Serial.print(gps.failedChecksum());
  Serial.print(F(" passed=")); Serial.println(gps.passedChecksum());
  if (chars == 0)
    Serial.println(F("[gps] NO DATA - wrong baud? run 'B' scan, or check TX/RX swap / power"));
  else if (gps.passedChecksum() == 0)
    Serial.println(F("[gps] bytes but 0 valid sentences - baud mismatch likely; run 'B' scan"));
}

// 'D' command: raw passthrough. Power the GPS and dump USART1 bytes verbatim to
// the USB console so we can see exactly what the module emits. Readable NMEA
// ("$GPGGA,...") = baud correct; garbage = baud mismatch (run 'B'); nothing =
// no power / TX-RX swap / dead module. Runs until any key is pressed.
static void gpsRawDump() {
  Serial.println(F("---- GPS RAW DUMP (USART1 -> USB, verbatim) ----"));
  gpsPower(true);                    // GPS_EN HIGH
  delay(100);
  gpsSerial.begin(GPS_BAUD);
  Serial.print(F("[gps] GPS_EN HIGH, USART1 RX=PB7 @ ")); Serial.print(GPS_BAUD);
  Serial.println(F(" baud. Raw bytes for 60 s (send any key to stop early):"));
  Serial.println(F("--------------------------------------------------"));

  while (gpsSerial.available()) gpsSerial.read();   // drop a partial GPS line
  while (Serial.available())    Serial.read();      // drop the 'D'+Enter that launched us

  uint32_t chars = 0, t0 = millis();
  while (millis() - t0 < 60000) {
    while (gpsSerial.available()) { Serial.write((uint8_t)gpsSerial.read()); chars++; }
    if (Serial.available()) { Serial.read(); break; }   // real key press -> stop early
  }
  Serial.println();
  Serial.print(F("[gps] raw dump stopped. bytes seen=")); Serial.println(chars);
  if (chars == 0)
    Serial.println(F("[gps] NOTHING - check GPS power (g), TX/RX wiring, or baud (B)"));
}

// 'B' command: try common GPS baud rates and report which yields framed NMEA
// ('$' sentence starts). Use this to discover the module's configured rate.
static void gpsBaudScan() {
  static const uint32_t bauds[] = { 9600, 4800, 38400, 57600, 115200 };
  Serial.println(F("---- GPS BAUD SCAN ----"));
  gpsPower(true);
  delay(100);
  for (uint8_t i = 0; i < sizeof(bauds) / sizeof(bauds[0]); i++) {
    gpsSerial.begin(bauds[i]);
    delay(50);
    while (gpsSerial.available()) gpsSerial.read();   // flush partial line

    uint32_t t0 = millis(), total = 0, dollars = 0, printable = 0;
    while (millis() - t0 < 1500) {
      while (gpsSerial.available()) {
        char c = (char)gpsSerial.read();
        total++;
        if (c == '$') dollars++;
        if (c >= 32 && c < 127) printable++;
      }
    }
    gpsSerial.end();

    bool likely = (total > 20) && (dollars > 0) && (printable * 4 > total * 3);
    Serial.print(F("[scan] ")); Serial.print(bauds[i]);
    Serial.print(F(" baud: bytes=")); Serial.print(total);
    Serial.print(F(" '$'=")); Serial.print(dollars);
    Serial.println(likely ? F("  <-- LIKELY") : F(""));
  }
  Serial.println(F("[scan] done. Set GPS_BAUD to the LIKELY rate and rebuild."));
}

static void handleCommand(char c) {
  switch (c) {
    case '?':
    case 'h':
      Serial.println(F("[help] ? help | p ping | v version | c charge | f flash | m meta | g gps pwr | D gps raw dump | G gps fix | B gps baud scan | s sched | S STOP | r baud bench | b DFU"));
      break;
    case 'c':
      Serial.print(F("[charge] STDBY="));
      Serial.print(chargeComplete() ? F("LOW") : F("HIGH"));
      Serial.print(F(" -> "));
      Serial.println(chargeComplete() ? F("FULL / standby (charge complete)")
                                      : F("charging or no input power"));
      break;
    case 'p':
      Serial.println(F("[pong]"));
      break;
    case 'v':
      Serial.println(F("[version] USB serial test v0.14 (GPS USART1 + TinyGPS++)"));
      break;
    case 'f':
      flashTest();
      break;
    case 'm':
      metaTest();
      break;
    case 'g':
      gpsPower(!gpsPowered);
      Serial.print(F("[gps] GPS_EN "));
      Serial.print(gpsPowered ? F("HIGH") : F("LOW"));
      Serial.println(gpsPowered ? F(" -> backup rail ON") : F(" -> backup rail OFF"));
      break;
    case 'G':
      gpsTest();
      break;
    case 'D':
      gpsRawDump();
      break;
    case 'B':
      gpsBaudScan();
      break;
    case 's':
      sleepSchedTest();
      break;
    case 'S':
      realStopTest();
      break;
    case 'r':
      runBaudBenchmark();
      break;
    case 'b':
      Serial.println(F("[boot] rebooting into ROM DFU bootloader..."));
      Serial.flush();
      delay(50);
      bootFlag = DFU_BOOT_MAGIC;
      NVIC_SystemReset();
      break;  // not reached
    case '\r':
    case '\n':
      break;  // swallow line endings quietly
    default:
      Serial.print(F("[echo] "));
      Serial.println(c);
      break;
  }
}

// ---- Arduino entry points --------------------------------------------------
void setup() {
  // Software DFU entry: if the 'b' command asked for it, jump to the ROM
  // bootloader now, before USB is brought up.
  if (bootFlag == DFU_BOOT_MAGIC) {
    bootFlag = 0;
    jumpToBootloader();
  }

  // Hold the GPS-backup high-side switch OFF from the very start of boot, so its
  // turn-on inrush can never coincide with the reset-vector fetch. Enabled later,
  // after the rail has settled (see gpsPower()).
  pinMode(GPS_EN, OUTPUT);
  gpsPower(false);

  pinMode(CHRG_STAT, INPUT_PULLUP);  // TP4057 STDBY (open-drain, active-low)

  Serial.begin(115200);  // baud is nominal for USB CDC, kept for tooling

  // Requested: 3-second quiet period before the first serial output.
  delay(FIRST_PRINT_DELAY_MS);

  // Wait for the host to open the port (DTR), but time out so a headless
  // boot still proceeds instead of hanging here.
  uint32_t start = millis();
  while (!Serial && (millis() - start) < HOST_WAIT_MS) {
    delay(10);
  }

  printBanner();

  // Boot self-tests over the USB console (flash R/W/E + metadata persistence).
  flashTest();
  metaShow();

  // Rail is settled and we are well past the boot window: safe to power the GPS
  // backup rail now.
  gpsPower(true);
  Serial.println(F("[gps] GPS_EN HIGH -> backup rail powered (post-boot)"));

  lastBeat = millis();
}

void loop() {
  uint32_t nowMs = millis();
  if (nowMs - lastBeat >= HEARTBEAT_MS) {
    lastBeat = nowMs;
    Serial.print(F("[hb] #"));
    Serial.print(beatCount++);
    Serial.print(F("  uptime="));
    Serial.print(nowMs / 1000);
    Serial.print(F("s  chg="));
    Serial.println(chargeComplete() ? F("FULL") : F("CHG?"));
  }

  while (Serial.available() > 0) {
    handleCommand((char)Serial.read());
  }
}
