#include <Arduino.h>
#include <elapsedMillis.h>
#include <TinyGPS++.h>
#include <time.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SPIMemory.h>
#include <TimeLib.h>
#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <definitions.h>
#include <Wire.h>
#include <codes.h>
#include <ArduinoJson.h>

///// LIBRARY DECLARATIONS /////
TinyGPSPlus gps;
SPIFlash flash(LCS);
elapsedMillis mTime;

///// DEVICE DEFINITIONS /////

// GPS Storage Variables
float lastKnownLat = 0;
float lastKnownLng = 0;

// Flash Addresses
uint32_t writeAddress = 0;
uint32_t readAddress = 0;

// GPS Control Variables (User Configurable)
int gpsFrequency = 3;        // GPS Frequency in minutes
int gpsTimeout = 60;          // GPS Timeout in seconds
int gpsHdop = 5;              // GPS HDOP threshold (lower is better)

// Device Variables
uint16_t dataCount = 0;       // Number of data points stored

// System State Variables
bool sleeping = false;
volatile uint32_t rtcCounter = 0;
const uint32_t FLASH_CAPACITY = 67108864; // 64MB in bytes

// Constants
const uint32_t SERIAL_TIMEOUT = 180000;   // 3 minutes
const uint16_t GPS_SETUP_TIMEOUT = 600;   // 10 minutes for initial GPS test
const uint32_t GPS_AGE_THRESHOLD = 2000;  // GPS data must be < 2 seconds old
const uint8_t HDOP_AGE_THRESHOLD = 2000;  // HDOP data must be < 2 seconds old
const uint32_t MIN_GPS_RUNTIME = 5000;    // Minimum 5 seconds for GPS to stabilize

//...................................//
//           UTILITY FUNCTIONS       //
//...................................//

void updateEeprom() {
  meta metaData;
  metaData.count = dataCount;
  metaData.gfrq = gpsFrequency;
  metaData.gto = gpsTimeout;
  metaData.hdop = gpsHdop;
  metaData.ra = readAddress;
  metaData.wa = writeAddress;
  EEPROM.put(eepromAddress, metaData);
}

void sendSerialMessage(byte code) {
  StaticJsonDocument<256> doc;
  String data;
  doc["Msg"] = code;
  doc["ID"] = tag;
  serializeJson(doc, data);
  Serial.println(data);
  Serial.flush();
}

void sendMemoryStatus() {
  StaticJsonDocument<256> doc;
  String data;
  uint32_t usedSpace = writeAddress;
  uint32_t freeSpace = FLASH_CAPACITY - writeAddress;
  float percentUsed = (float)usedSpace / FLASH_CAPACITY * 100.0;
  
  doc["Msg"] = MEMORY_STATUS;
  doc["ID"] = tag;
  doc["Used"] = usedSpace;
  doc["Free"] = freeSpace;
  doc["Percent"] = percentUsed;
  doc["Count"] = dataCount;
  
  serializeJson(doc, data);
  Serial.println(data);
  Serial.flush();
}

bool loadEepromSettings() {
  meta metaData;
  EEPROM.get(eepromAddress, metaData);
  
  // Check if EEPROM has valid data (not factory default -1)
  if (metaData.gfrq == -1 || metaData.gto == -1 || metaData.hdop == -1) {
    Serial.println(F("No valid EEPROM settings found"));
    Serial.println(F("Initializing with defaults"));
    updateEeprom();
    return false;
  }
  
  // Load settings from EEPROM
  gpsFrequency = metaData.gfrq;
  gpsTimeout = metaData.gto;
  gpsHdop = metaData.hdop;
  readAddress = metaData.ra;
  writeAddress = metaData.wa;
  dataCount = metaData.count;
  
  Serial.println(F("#### LOADED SETTINGS ####"));
  Serial.print(F("GPS Frequency: ")); Serial.print(gpsFrequency); Serial.println(F(" min"));
  Serial.print(F("GPS Timeout: ")); Serial.print(gpsTimeout); Serial.println(F(" sec"));
  Serial.print(F("GPS HDOP: ")); Serial.println(gpsHdop);
  Serial.print(F("Data Points: ")); Serial.println(dataCount);
  Serial.print(F("Write Address: ")); Serial.println(writeAddress);
  Serial.print(F("Read Address: ")); Serial.println(readAddress);
  Serial.println();
  
  return true;
}

float readBatteryVoltage() {
    delayMicroseconds(200);   // allow S/H capacitor to charge
    uint16_t raw = analogRead(BAT_SNS);

    float v_adc = (raw / 4095.0) * 3.3;
    return v_adc * 2.0;
}



//...................................//
//           GPS FUNCTIONS           //
//...................................//

bool acquireGPSFix(uint32_t timeoutSeconds, bool isSetup = false) {
  mTime = 0;
  double hdopValue = 100.00;
  uint32_t timeoutMs = timeoutSeconds * 1000UL;
  uint32_t lastPrintTime = 0;
  
  digitalWrite(GPS_PIN, HIGH);
  delay(100);
  
  Serial.print(F("Acquiring GPS fix (timeout: "));
  Serial.print(timeoutSeconds);
  Serial.println(F(" sec)"));
  
  while (mTime < timeoutMs) {
    // Feed GPS parser one character at a time
    while (Serial1.available() > 0) {
      if (gps.encode(Serial1.read())) {
        // Successfully parsed a complete sentence
        if (gps.hdop.hdop() != 0.00) {
          hdopValue = gps.hdop.hdop();
        }
      }
    }
    
    // Print status every 5 seconds
    if (mTime - lastPrintTime >= 5000) {
      lastPrintTime = mTime;
      Serial.print(F("Sats: ")); 
      Serial.print(gps.satellites.value());
      Serial.print(F(" | HDOP: ")); 
      Serial.println(hdopValue, 2);
    }
    
    // Check if we meet fix criteria
    if (hdopValue < (double)gpsHdop && 
        gps.satellites.value() >= 4 &&
        gps.location.age() < 1000 && 
        mTime > 3000) {
      
      Serial.println(F("Valid GPS fix acquired!"));
      digitalWrite(GPS_PIN, LOW);
      return true;
    }
  }
  
  Serial.println(F("GPS timeout"));
  digitalWrite(GPS_PIN, LOW);
  return false;
}

bool recordGPSData(bool isSetup = false) {
  bool fixAcquired = acquireGPSFix(isSetup ? GPS_SETUP_TIMEOUT : gpsTimeout, isSetup);
  
  data dat;
  memset(&dat, 0, sizeof(dat)); // Initialize struct to zero
  
  // Populate data structure
  if (fixAcquired && gps.location.age() < GPS_AGE_THRESHOLD) {
    dat.lat = gps.location.lat();
    dat.lng = gps.location.lng();
    lastKnownLat = dat.lat;
    lastKnownLng = dat.lng;
    Serial.print(F("Location: "));
    Serial.print(dat.lat, 6);
    Serial.print(F(", "));
    Serial.println(dat.lng, 6);
  } else {
    // Use zeros to indicate no fix
    dat.lat = 0.0;
    dat.lng = 0.0;
    Serial.println(F("No valid location - storing zeros"));
  }
  
  // Set timestamp (use GPS time if available, otherwise system time)
  if (gps.time.isValid() && gps.date.isValid()) {
    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), 
            gps.date.day(), gps.date.month(), gps.date.year());
  }
  dat.datetime = (uint32_t)now();
  dat.locktime = mTime / 1000;
  dat.hdop = gps.hdop.hdop();
  dat.count = dataCount;
  dat.id = tag;
  dat.x = 0;
  dat.y = 0;
  dat.z = 0;
  
  Serial.print(F("Timestamp: ")); Serial.println(dat.datetime);
  Serial.print(F("Lock time: ")); Serial.print(dat.locktime); Serial.println(F(" sec"));
  Serial.print(F("HDOP: ")); Serial.println(dat.hdop, 2);
  
  // Don't write to flash during setup
  if (isSetup) {
    sendSerialMessage(fixAcquired ? GPS_SUCCESS : GPS_ERROR);
    return fixAcquired;
  }
  
  // Check if flash is full
  if (writeAddress + sizeof(data) >= FLASH_CAPACITY) {
    Serial.println(F("ERROR: Flash memory full!"));
    sendSerialMessage(MEMORY_FULL);
    return false;
  }
  
  // Write data to flash
  if (!flash.powerUp()) {
    Serial.println(F("ERROR: Flash power up failed"));
    return false;
  }
  
  delay(10); // Give flash time to power up
  
  uint32_t newWriteAddress = flash.getAddress(sizeof(data));
  Serial.print(F("Writing to address: ")); Serial.println(newWriteAddress);
  
  if (flash.writeAnything(newWriteAddress, dat)) {
    Serial.println(F("Write successful"));
    writeAddress = newWriteAddress;
    dataCount++;
    updateEeprom();
    flash.powerDown();
    return true;
  } else {
    Serial.println(F("ERROR: Flash write failed"));
    Serial.println(flash.error(VERBOSE));
    flash.powerDown();
    return false;
  }
}

//...................................//
//        DATA DOWNLOAD FUNCTIONS    //
//...................................//

bool sendDataPoint(uint32_t address) {
  data dat;
  StaticJsonDocument<256> doc;
  String databuf;
  
  if (!flash.readAnything(address, dat)) {
    Serial.println(F("ERROR: Flash read failed"));
    return false;
  }
  
  doc["ID"] = dat.id;
  doc["DT"] = dat.datetime;
  doc["LAT"] = dat.lat;
  doc["LNG"] = dat.lng;
  doc["HDOP"] = dat.hdop;
  doc["LCKTM"] = dat.locktime;
  doc["CNT"] = dat.count;
  doc["X"] = dat.x;
  doc["Y"] = dat.y;
  doc["Z"] = dat.z;
  
  serializeJson(doc, databuf);
  Serial.println(databuf);
  Serial.flush();
  
  return true;
}

void downloadAllData() {
  Serial.println(F("Starting full data download"));
  sendSerialMessage(DATA_DOWNLOAD_BEGIN);
  
  if (!flash.powerUp()) {
    Serial.println(F("ERROR: Flash power up failed"));
    sendSerialMessage(DATA_DOWNLOAD_ERROR);
    return;
  }
  
  delay(10);
  
  uint32_t currentAddress = 0;
  uint16_t pointsSent = 0;
  
  // Fixed: Changed > to < in the while condition
  while (currentAddress < writeAddress) {
    if (!sendDataPoint(currentAddress)) {
      sendSerialMessage(DATA_DOWNLOAD_ERROR);
      break;
    }
    currentAddress += sizeof(data);
    pointsSent++;
    delay(10); // Small delay between transmissions
  }
  
  flash.powerDown();
  
  Serial.print(F("Download complete: "));
  Serial.print(pointsSent);
  Serial.println(F(" points sent"));
  sendSerialMessage(DATA_DOWNLOAD_END);
}

void downloadNewData() {
  Serial.println(F("Starting new data download"));
  sendSerialMessage(DATA_DOWNLOAD_BEGIN);
  
  if (!flash.powerUp()) {
    Serial.println(F("ERROR: Flash power up failed"));
    sendSerialMessage(DATA_DOWNLOAD_ERROR);
    return;
  }
  
  delay(10);
  
  uint16_t pointsSent = 0;
  uint32_t currentAddress = readAddress;
  
  while (currentAddress < writeAddress) {
    if (!sendDataPoint(currentAddress)) {
      sendSerialMessage(DATA_DOWNLOAD_ERROR);
      break;
    }
    currentAddress += sizeof(data);
    pointsSent++;
    delay(10);
  }
  
  // Update read address to mark data as downloaded
  readAddress = currentAddress;
  updateEeprom();
  
  flash.powerDown();
  
  Serial.print(F("Download complete: "));
  Serial.print(pointsSent);
  Serial.println(F(" new points sent"));
  sendSerialMessage(DATA_DOWNLOAD_END);
}

//...................................//
//       CALIBRATION FUNCTIONS       //
//...................................//

void deviceCalibration() {
  Serial.println(F("==== DEVICE CALIBRATION ===="));
  sendSerialMessage(CALIBRATION_BEGIN);
  
  // Test Flash Memory
  Serial.println(F("Testing flash memory..."));
  sendSerialMessage(FLASH_DIAGNOSTICS);
  
  if (flash.powerUp()) {
    delay(10);
    uint16_t jedecID = flash.getJEDECID();
    if (jedecID != 0 && jedecID != 0xFFFF) {
      Serial.print(F("Flash JEDEC ID: 0x"));
      Serial.println(jedecID, HEX);
      Serial.print(F("Manufacturer ID: 0x"));
      Serial.println(flash.getManID(), HEX);
      sendSerialMessage(FLASH_SUCCESS);
    } else {
      Serial.println(F("ERROR: Flash memory not detected"));
      sendSerialMessage(FLASH_ERROR);
    }
    flash.powerDown();
  } else {
    Serial.println(F("ERROR: Flash power up failed"));
    sendSerialMessage(FLASH_ERROR);
  }
  
  sendSerialMessage(FLASH_DIAGNOSTICS_END);
  
  // Test GPS
  Serial.println(F("Testing GPS..."));
  sendSerialMessage(GPS_CALIBRATION);
  bool gpsSuccess = recordGPSData(true); // Will send GPS_SUCCESS or GPS_ERROR
  // bool gpsSuccess = true; // Temporary bypass for GPS testing
  sendSerialMessage(GPS_CALIBRATION_END);
  
  Serial.println(F("Calibration complete"));
  sendSerialMessage(CALIBRATION_END);
  
  // Send calibration summary data
  StaticJsonDocument<256> doc;
  String data;
  
  
  // GPS data from calibration
  if (gpsSuccess && gps.location.isValid()) {
    doc["LAT"] = gps.location.lat();
    doc["LNG"] = gps.location.lng();
    doc["HDOP"] = gps.hdop.hdop();
  } else {
    doc["LAT"] = 0;
    doc["LNG"] = 0;
    doc["HDOP"] = 0;
  }
  
  // Storage information
  doc["DataPoints"] = dataCount;
  doc["StorageUsed"] = writeAddress/FLASH_CAPACITY * 100.0;
  doc["StorageFree"] = (FLASH_CAPACITY - writeAddress)/FLASH_CAPACITY * 100.0;
  doc["BatteryV"] = readBatteryVoltage();
  
  serializeJson(doc, data);
  Serial.println(data);
  Serial.flush();
}

//...................................//
//      SERIAL COMMAND HANDLER       //
//...................................//

void handleSerialCommands(uint32_t timeout) {
  Serial.println(F("Waiting for serial commands..."));
  mTime = 0;
  
  while (mTime < timeout) {
    if (Serial.available() >= sizeof(reqPing)) {
      reqPing incomingPing;
      size_t bytesRead = Serial.readBytes((char*)&incomingPing, sizeof(incomingPing));
      
      if (bytesRead != sizeof(reqPing)) {
        Serial.println(F("Warning: Incomplete ping received"));
        continue;
      }
      
      // Verify the tag matches
      if (incomingPing.tag != tag) {
        Serial.println(F("Warning: Tag mismatch"));
        continue;
      }
      
      // Handle commands
      switch (incomingPing.request) {
        case FINISH_CALIBRATION:
          Serial.println(F("Calibration finished by user"));
          return;
          
        case DATA_DOWNLOAD_ALL:
          Serial.println(F("Download all data requested"));
          downloadAllData();
          break;
          
        case DATA_DOWNLOAD_NEW:
          Serial.println(F("Download new data requested"));
          downloadNewData();
          break;
          
        case MEMORY_CLEAR:
          Serial.println(F("Memory clear requested"));
          if (flash.powerUp()) {
            delay(10);
            if (flash.eraseChip()) {
              Serial.println(F("Memory cleared"));
              writeAddress = 0;
              readAddress = 0;
              dataCount = 0;
              updateEeprom();
              sendSerialMessage(MEMORY_CLEARED);
            } else {
              Serial.println(F("ERROR: Memory clear failed"));
              sendSerialMessage(MEMORY_CLEAR_ERROR);
            }
            flash.powerDown();
          } else {
            sendSerialMessage(MEMORY_CLEAR_ERROR);
          }
          break;
          
        case MEMORY_STATUS:
          Serial.println(F("Memory status requested"));
          sendMemoryStatus();
          break;
          
        case REQUEST_SETTINGS:
          Serial.println(F("Settings requested"));
          loadEepromSettings(); // Will print current settings
          break;
          
        default:
          Serial.print(F("Unknown command: "));
          Serial.println(incomingPing.request);
          break;
      }
    } else if (Serial.available() >= sizeof(setttings)) {
      setttings incomingSettings;
      size_t bytesRead = Serial.readBytes((char*)&incomingSettings, sizeof(incomingSettings));
      
      if (bytesRead != sizeof(setttings)) {
        Serial.println(F("Warning: Incomplete settings received"));
        continue;
      }
      
      if (incomingSettings.tag == tag) {
        Serial.println(F("Updating settings..."));
        gpsFrequency = incomingSettings.gpsFrq;
        gpsTimeout = incomingSettings.gpsTout;
        gpsHdop = incomingSettings.hdop;
        
        // Validate settings
        if (gpsFrequency < 1) gpsFrequency = 1;
        if (gpsFrequency > 1440) gpsFrequency = 1440; // Max 24 hours
        if (gpsTimeout < 10) gpsTimeout = 10;
        if (gpsTimeout > 300) gpsTimeout = 300; // Max 5 minutes
        if (gpsHdop < 1) gpsHdop = 1;
        if (gpsHdop > 20) gpsHdop = 20;
        
        updateEeprom();
        loadEepromSettings(); // Print new settings
        sendSerialMessage(SETTINGS_UPDATED);
      } else {
        Serial.println(F("ERROR: Settings tag mismatch"));
        sendSerialMessage(SETTINGS_UPDATE_ERROR);
      }
    }
    
    delay(10); // Small delay to prevent tight loop
  }
  
  Serial.println(F("Serial command timeout"));
}

//...................................//
//          RTC & SLEEP              //
//...................................//

void RTC_init(void) {
  while (RTC.STATUS > 0) {
    ; // Wait for all registers to be synchronized
  }
  
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc; // 32.768kHz Internal Oscillator
  RTC.PITINTCTRL = RTC_PI_bm;        // Enable PIT interrupt
  
  // Set period to 1 second (32768 cycles at 32.768kHz)
  RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc | RTC_PITEN_bm;
}

ISR(RTC_PIT_vect) {
  RTC.PITINTFLAGS = RTC_PI_bm; // Clear interrupt flag
  rtcCounter++;
}

//...................................//
//          SETUP & LOOP             //
//...................................//

void setup() { 
  // Initialize serial communications
  Serial.begin(9600);
  Serial1.begin(115200);  // GPS module baud rate
  
  // Initialize SPI
  SPI.begin();
  
  // Configure pins
  pinMode(GPS_PIN, OUTPUT);
  digitalWrite(GPS_PIN, LOW);
  pinMode(LCS, OUTPUT);
  digitalWrite(LCS, HIGH);

  analogReadResolution(12);     // Explicit (good practice)
  ADC0.SAMPCTRL = 15;           // Max sample length for high impedance

  
  delay(1000);
  
  Serial.println(F(""));
  Serial.println(F("================================"));
  Serial.println(F("   GPS LOGGER INITIALIZATION"));
  Serial.println(F("================================"));
  Serial.print(F("Firmware Version: ")); Serial.println(firmwareVersion);
  Serial.print(F("Device Tag: ")); Serial.println(tag);
  Serial.print(F("Device Type: ")); Serial.println(devType);
  Serial.println();
  
  // Initialize flash memory
  Serial.println(F("Initializing flash memory..."));
  
  // Power cycle sequence from original code
  if (!flash.powerUp()) {
    Serial.println(flash.error(VERBOSE));
  }
  if (!flash.powerDown()) {
    Serial.println(flash.error(VERBOSE));
  }
  if (!flash.powerUp()) {
    Serial.println(flash.error(VERBOSE));
  }
  
  if (!flash.begin(MB(64))) {
    Serial.println(F("ERROR: Flash initialization failed"));
    Serial.println(flash.error(VERBOSE));
    while (1) {
      delay(1000); // Halt if flash fails
    }
  }
  
  Serial.print(F("Flash Manufacturer ID: "));
  Serial.println(flash.getManID());
  Serial.println(F("Flash initialized successfully"));
  
  if (flash.powerDown()) {
    digitalWrite(LCS, HIGH);
  } else {
    Serial.println(flash.error(VERBOSE));
  }

  // Load settings from EEPROM
  loadEepromSettings();
  
  // Run device calibration
  deviceCalibration();
  
  // Wait for serial commands
  handleSerialCommands(SERIAL_TIMEOUT);
  
  // Load settings from EEPROM
  loadEepromSettings();
  
  // Initialize RTC and sleep mode
  RTC_init();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sei(); // Enable global interrupts
  
  Serial.println(F("Setup complete - entering main loop"));
  Serial.println(F("================================"));
  Serial.println();
}

void loop() {
  if (sleeping) {
    // Check if it's time to wake up and record data
    uint32_t sleepDuration = gpsFrequency * 60UL; // Convert minutes to seconds
    
    if (rtcCounter >= sleepDuration) {
      sleeping = false;
      Serial.println(F("Wake up - time to record data"));
    }
  } else {
    // Record GPS data
    Serial.print(F("Recording data point #"));
    Serial.println(dataCount + 1);
    
    bool success = recordGPSData(false);
    
    if (success) {
      Serial.println(F("Data recorded successfully"));
    } else {
      Serial.println(F("Warning: Data recording issue"));
    }
    
    // Reset sleep counter and enter sleep mode
    rtcCounter = 0;
    sleeping = true;
    
    Serial.println(F("Entering sleep mode"));
    Serial.println();
  }
  
  // Flush serial and enter sleep
  Serial.flush();
  sleep_cpu();
}  