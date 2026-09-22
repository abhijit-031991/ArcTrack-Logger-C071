// Firmware Version //
const float firmwareVersion = 4.0;

// Device Information //

const uint16_t tag = 10606;
const uint8_t devType = 107;

// Pin Definitions //
#define GPS_PIN PIN_PD7
#define RINT PIN_PD6
#define AINT1 PIN_PD4
#define AINT2 PIN_PD5
#define LCS PIN_PA7
#define FCS PIN_PA7
#define BAT_SNS A2

#define MAX_ELECTRODES 4

// Battery sensing
#define BAT_DIVIDER_RATIO 2.0
#define ADC_MAX_VALUE 1023.0
#define ADC_REF_VOLTAGE 3.3   // change to 5.0 if AVCC = 5V
#define BAT_CALIBRATION 0.95


// EEPROM MetaData Address //
int eepromAddress = 1;

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
    int radioFrq;
    int startHour;
    int endHour;
    bool scheduled;
}__attribute__((__packed__));

struct meta
  {
    int gfrq;
    int gto;
    int hdop;
    uint16_t count;
    uint32_t wa;
    uint32_t ra;
  }__attribute__((__packed__));