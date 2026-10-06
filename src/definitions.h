#pragma once

// Firmware Version //
const float firmwareVersion = 4.0;

// Device Information //

const uint16_t tag = 10608;
const uint8_t devType = 107;

// Pin Definitions //
#define USART2_TX PA2
#define USART2_RX PA3
#define USART1_TX PB6
#define USART1_RX PB7
#define I2C1_SCL PB8
#define I2C1_SDA PB9
#define I2C2_SCL PA7
#define I2C2_SDA PA6
#define USB_DM PA11   // USB D- (was "USB_D-": '-' is not valid in a macro name)
#define USB_DP PA12   // USB D+ (was "USB_D+": '+' is not valid in a macro name)
#define SPI1_MOSI PB5
#define SPI1_MISO PB4
#define SPI1_SCK PB3
#define SPI1_NSS PA15
#define BAT_SNS PB1
#define CHRG_STAT PB2
#define SNS_EN PA5
#define GPS_EN PA4
#define PWR_WKUP1 PA0

#define MAX_ELECTRODES 4


// EEPROM MetaData Address //
inline int eepromAddress = 1;   // inline: safe to include from multiple TUs (C++17)

// Structs //

struct longPing{
    uint16_t ta;    
    uint16_t cnt;
    float la;
    float ln;
    uint8_t devtyp;
    bool mortality;
  }__attribute__((__packed__));

struct data{
    uint32_t datetime;
    uint16_t locktime;
    float lat;
    float lng;
    float hdop;
    float x;
    float y;
    float z;
    unsigned int count;
    uint16_t id;
    uint8_t sats;        // satellites used to acquire the fix
}__attribute__((__packed__));

struct reqPing{
    uint16_t tag;
    byte request;
  }__attribute__((__packed__));

struct setttings{
    uint16_t tag;
    int gpsFrq;
    int gpsTout;
    int hdop;
    int minSat;          // minimum satellites required to record a fix
    int radioFrq;
    int startHour;
    int endHour;
    bool scheduled;
}__attribute__((__packed__));

struct meta
  {
    uint32_t magic;      // validity marker (META_MAGIC) - rejects stale/foreign metadata
    int gfrq;
    int gto;
    int hdop;
    uint16_t count;
    uint32_t wa;
    uint32_t ra;
    int msat;            // persisted minimum-satellites setting
  }__attribute__((__packed__));