/**
 * LoRaE5_SPIFlash.h
 * 
 * Custom SPI Flash Memory Library for LoRa-E5 (STM32WLE5JC)
 * Optimized for SPI2 peripheral and common flash memory chips
 * 
 * Supports:
 * - Winbond W25Q series (W25Q32, W25Q64, W25Q128, W25Q256)
 * - Microchip SST25/SST26 series  
 * - Macronix MX25L series
 * - Cypress S25FL series
 * - Generic SPI flash with JEDEC standard commands
 * 
 * Author: Custom implementation for LoRa-E5
 * License: MIT
 */

#ifndef LORAE5_SPIFLASH_H
#define LORAE5_SPIFLASH_H

#include <Arduino.h>
#include <SPI.h>

// Flash memory commands (JEDEC standard)
#define FLASH_CMD_WRITE_ENABLE        0x06
#define FLASH_CMD_WRITE_DISABLE       0x04
#define FLASH_CMD_READ_STATUS_REG     0x05
#define FLASH_CMD_WRITE_STATUS_REG    0x01
#define FLASH_CMD_READ_DATA           0x03
#define FLASH_CMD_FAST_READ           0x0B
#define FLASH_CMD_PAGE_PROGRAM        0x02
#define FLASH_CMD_SECTOR_ERASE        0x20
#define FLASH_CMD_BLOCK_ERASE_32K     0x52
#define FLASH_CMD_BLOCK_ERASE_64K     0xD8
#define FLASH_CMD_CHIP_ERASE          0xC7
#define FLASH_CMD_POWER_DOWN          0xB9
#define FLASH_CMD_RELEASE_POWER_DOWN  0xAB
#define FLASH_CMD_JEDEC_ID            0x9F
#define FLASH_CMD_UNIQUE_ID           0x4B

// Status register bits
#define FLASH_STATUS_BUSY             0x01
#define FLASH_STATUS_WEL              0x02
#define FLASH_STATUS_BP0              0x04
#define FLASH_STATUS_BP1              0x08
#define FLASH_STATUS_BP2              0x10
#define FLASH_STATUS_TB               0x20
#define FLASH_STATUS_SEC              0x40
#define FLASH_STATUS_SRP0             0x80

// Error codes
#define FLASH_OK                      0x00
#define FLASH_ERROR_TIMEOUT           0x01
#define FLASH_ERROR_WRITE_PROTECTED   0x02
#define FLASH_ERROR_INVALID_ADDRESS   0x03
#define FLASH_ERROR_NOT_ERASED        0x04
#define FLASH_ERROR_VERIFY_FAILED     0x05
#define FLASH_ERROR_NO_RESPONSE       0x06
#define FLASH_ERROR_UNKNOWN_CHIP      0x07

// Flash chip information structure
struct FlashChipInfo {
    uint32_t jedecId;
    uint32_t capacity;
    uint16_t pageSize;
    uint16_t sectorSize;
    uint32_t blockSize;
    const char* manufacturer;
    const char* model;
};

class LoRaE5_SPIFlash {
private:
    SPIClass* _spi;
    uint8_t _csPin;
    uint32_t _spiFrequency;
    FlashChipInfo _chipInfo;
    bool _initialized;
    
    // Internal SPI transaction methods
    void beginTransaction();
    void endTransaction();
    uint8_t transfer(uint8_t data);
    void transfer(uint8_t* buffer, size_t length);
    
    // Low-level flash operations
    void writeEnable();
    void writeDisable();
    uint8_t readStatusRegister();
    bool isWriteEnabled();
    bool isBusy();
    
    // Chip identification
    uint32_t readJEDECID();
    bool identifyChip();
    void setChipInfo(uint32_t jedecId);
    
    // Address validation
    bool isValidAddress(uint32_t address);
    bool isValidRange(uint32_t address, uint32_t length);
    
    // Internal write/read helpers
    uint8_t writeByteInternal(uint32_t address, uint8_t data, bool verify = true);
    uint8_t readByteInternal(uint32_t address);

public:
    // Constructors
    LoRaE5_SPIFlash(uint8_t csPin, SPIClass* spiInstance = &SPI, uint32_t spiFrequency = 1000000);
    
    // Initialization
    bool begin();
    void end();
    
    // Chip information
    FlashChipInfo getChipInfo() const { return _chipInfo; }
    uint32_t getCapacity() const { return _chipInfo.capacity; }
    uint16_t getPageSize() const { return _chipInfo.pageSize; }
    uint16_t getSectorSize() const { return _chipInfo.sectorSize; }
    const char* getManufacturer() const { return _chipInfo.manufacturer; }
    const char* getModel() const { return _chipInfo.model; }
    uint32_t getJEDECID() const { return _chipInfo.jedecId; }
    
    // Power management
    void powerDown();
    void powerUp();
    
    // Erase operations
    uint8_t eraseSector(uint32_t address);
    uint8_t eraseBlock32K(uint32_t address);
    uint8_t eraseBlock64K(uint32_t address);
    uint8_t eraseChip();
    
    // Basic read/write operations
    uint8_t readByte(uint32_t address);
    uint8_t writeByte(uint32_t address, uint8_t data, bool verify = true);
    
    // Array operations
    uint8_t readArray(uint32_t address, uint8_t* buffer, uint32_t length);
    uint8_t writeArray(uint32_t address, const uint8_t* buffer, uint32_t length, bool verify = true);
    
    // Data type operations
    uint8_t readUint16(uint32_t address, uint16_t& value);
    uint8_t writeUint16(uint32_t address, uint16_t value, bool verify = true);
    uint8_t readUint32(uint32_t address, uint32_t& value);
    uint8_t writeUint32(uint32_t address, uint32_t value, bool verify = true);
    uint8_t readFloat(uint32_t address, float& value);
    uint8_t writeFloat(uint32_t address, float value, bool verify = true);
    
    // String operations REMOVED: Arduino String pulled in dtostrf->sprintf->vfprintf
    // ->malloc (~3-4 KB) and the firmware never used them. Use writeArray/readArray
    // with a char buffer if raw string storage is ever needed.
    // uint8_t readString(uint32_t address, String& str, uint16_t maxLength = 255);
    // uint8_t writeString(uint32_t address, const String& str, bool verify = true);
    
    // Structure operations (templates)
    template<typename T>
    uint8_t readStruct(uint32_t address, T& data);
    
    template<typename T>
    uint8_t writeStruct(uint32_t address, const T& data, bool verify = true);
    
    // Page operations
    uint8_t writePage(uint32_t address, const uint8_t* buffer, uint16_t length, bool verify = true);
    
    // Fast read operations
    uint8_t fastRead(uint32_t address, uint8_t* buffer, uint32_t length);
    
    // Utility functions
    bool isBlank(uint32_t address, uint32_t length);
    uint8_t verify(uint32_t address, const uint8_t* buffer, uint32_t length);
    
    // Status and diagnostics
    bool isInitialized() const { return _initialized; }
    void printChipInfo();
    void runDiagnostics();
    void softwareReset();
    void forceReleaseFromPowerDown();
    void clearProtectionBits();
    bool waitForReady(uint32_t timeoutMs = 5000);
    bool recover();


    // Configuration
    void setSPIFrequency(uint32_t frequency);
    uint32_t getSPIFrequency() const { return _spiFrequency; }
};

// Template implementations
template<typename T>
uint8_t LoRaE5_SPIFlash::readStruct(uint32_t address, T& data) {
    if (!_initialized) return FLASH_ERROR_NO_RESPONSE;
    if (!isValidRange(address, sizeof(T))) return FLASH_ERROR_INVALID_ADDRESS;
    
    return readArray(address, reinterpret_cast<uint8_t*>(&data), sizeof(T));
}

template<typename T>
uint8_t LoRaE5_SPIFlash::writeStruct(uint32_t address, const T& data, bool verify) {
    if (!_initialized) return FLASH_ERROR_NO_RESPONSE;
    if (!isValidRange(address, sizeof(T))) return FLASH_ERROR_INVALID_ADDRESS;
    
    return writeArray(address, reinterpret_cast<const uint8_t*>(&data), sizeof(T), verify);
}

#endif // LORAE5_SPIFLASH_H