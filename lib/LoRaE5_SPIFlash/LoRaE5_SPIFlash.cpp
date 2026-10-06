/**
 * LoRaE5_SPIFlash.cpp
 *
 * Custom SPI Flash Memory Library Implementation for LoRa-E5 (STM32WLE5JC)
 *
 * Author: Custom implementation for LoRa-E5
 * License: MIT
 */

#include "LoRaE5_SPIFlash.h"

// Known flash chip database
static const FlashChipInfo knownChips[] = {
    // Winbond W25Q series
    {0xEF4016, 4194304,   256, 4096, 65536, "Winbond", "W25Q32"},
    {0xEF4017, 8388608,   256, 4096, 65536, "Winbond", "W25Q64"},
    {0xEF4018, 16777216,  256, 4096, 65536, "Winbond", "W25Q128"},
    {0xEF4019, 33554432,  256, 4096, 65536, "Winbond", "W25Q256"},

    // Macronix MX25L series
    {0xC22016, 4194304,   256, 4096, 65536, "Macronix", "MX25L32"},
    {0xC22017, 8388608,   256, 4096, 65536, "Macronix", "MX25L64"},
    {0xC22018, 16777216,  256, 4096, 65536, "Macronix", "MX25L128"},

    // Microchip SST25 series
    {0xBF2541, 2097152,   256, 4096, 65536, "Microchip", "SST25VF016B"},
    {0xBF254A, 4194304,   256, 4096, 65536, "Microchip", "SST25VF032B"},

    // Cypress S25FL series
    {0x012018, 16777216,  256, 4096, 65536, "Cypress", "S25FL128S"},
    {0x012019, 33554432,  256, 4096, 65536, "Cypress", "S25FL256S"},

    // Generic fallback (must be last)
    {0x000000, 0, 256, 4096, 65536, "Unknown", "Generic"}
};

LoRaE5_SPIFlash::LoRaE5_SPIFlash(uint8_t csPin, SPIClass* spiInstance, uint32_t spiFrequency)
    : _spi(spiInstance), _csPin(csPin), _spiFrequency(spiFrequency), _initialized(false) {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    _chipInfo = {0, 0, 256, 4096, 65536, "Unknown", "Unknown"};
}

bool LoRaE5_SPIFlash::begin() {
    if (_spi == nullptr) return false;
    _spi->begin();
    powerUp();
    delay(10);
    if (!identifyChip()) {
        return false;
    }
    _initialized = true;
    return true;
}

void LoRaE5_SPIFlash::end() {
    if (_initialized) {
        powerDown();
        _spi->end();
        _initialized = false;
    }
}

// ============================================================
// SPI transaction helpers
// ============================================================

void LoRaE5_SPIFlash::beginTransaction() {
    _spi->beginTransaction(SPISettings(_spiFrequency, MSBFIRST, SPI_MODE0));
    digitalWrite(_csPin, LOW);
}

void LoRaE5_SPIFlash::endTransaction() {
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
}

uint8_t LoRaE5_SPIFlash::transfer(uint8_t data) {
    return _spi->transfer(data);
}

void LoRaE5_SPIFlash::transfer(uint8_t* buffer, size_t length) {
    _spi->transfer(buffer, length);
}

void LoRaE5_SPIFlash::writeEnable() {
    beginTransaction();
    transfer(FLASH_CMD_WRITE_ENABLE);
    endTransaction();
}

void LoRaE5_SPIFlash::writeDisable() {
    beginTransaction();
    transfer(FLASH_CMD_WRITE_DISABLE);
    endTransaction();
}

uint8_t LoRaE5_SPIFlash::readStatusRegister() {
    beginTransaction();
    transfer(FLASH_CMD_READ_STATUS_REG);
    uint8_t status = transfer(0x00);
    endTransaction();
    return status;
}

bool LoRaE5_SPIFlash::isWriteEnabled() {
    return (readStatusRegister() & FLASH_STATUS_WEL) != 0;
}

bool LoRaE5_SPIFlash::isBusy() {
    return (readStatusRegister() & FLASH_STATUS_BUSY) != 0;
}

// ============================================================
// Chip identification
// ============================================================

uint32_t LoRaE5_SPIFlash::readJEDECID() {
    beginTransaction();
    transfer(FLASH_CMD_JEDEC_ID);
    uint32_t id = 0;
    id |= (uint32_t)transfer(0x00) << 16;
    id |= (uint32_t)transfer(0x00) << 8;
    id |= (uint32_t)transfer(0x00);
    endTransaction();
    return id;
}

// FIX 5: validate manufacturer and memory-type bytes individually.
// A partial-clock response can produce a plausible 24-bit value like 0xEF00FF
// that passes the all-zeros / all-ones guard but is clearly invalid.
bool LoRaE5_SPIFlash::identifyChip() {
    uint32_t jedecId = readJEDECID();

    uint8_t mfr     = (jedecId >> 16) & 0xFF;
    uint8_t memType = (jedecId >>  8) & 0xFF;

    if (mfr == 0x00 || mfr == 0xFF)         return false;
    if (memType == 0x00 || memType == 0xFF)  return false;

    setChipInfo(jedecId);

    // Reject unknown chips whose capacity could not be determined
    if (_chipInfo.capacity == 0) return false;

    return true;
}

// FIX 4: guard against undefined behaviour from (1 << capacityCode).
// Left-shift on a 32-bit int is only defined for shift amounts 0..31.
// A capacityCode of 0 or >= 32 (e.g. 0xFF from a noisy bus) causes UB.
void LoRaE5_SPIFlash::setChipInfo(uint32_t jedecId) {
    for (size_t i = 0; i < sizeof(knownChips) / sizeof(knownChips[0]) - 1; i++) {
        if (knownChips[i].jedecId == jedecId) {
            _chipInfo = knownChips[i];
            return;
        }
    }

    // Unknown chip: infer capacity from JEDEC capacity byte.
    // Guard: only shift when capacityCode is in the range 1..31.
    uint8_t  capacityCode = jedecId & 0xFF;
    uint32_t capacity     = 0;
    if (capacityCode >= 1 && capacityCode <= 31) {
        capacity = (uint32_t)1 << capacityCode;
    }

    _chipInfo.jedecId      = jedecId;
    _chipInfo.capacity     = capacity;
    _chipInfo.pageSize     = 256;
    _chipInfo.sectorSize   = 4096;
    _chipInfo.blockSize    = 65536;
    _chipInfo.manufacturer = "Unknown";
    _chipInfo.model        = "Generic";
}

// ============================================================
// Address validation
// ============================================================

bool LoRaE5_SPIFlash::isValidAddress(uint32_t address) {
    return address < _chipInfo.capacity;
}

// FIX 3: overflow-safe range check.
// address + length wraps to 0 for large values, bypassing the capacity guard.
// Rewrite as: length <= (capacity - address) to avoid the overflow.
bool LoRaE5_SPIFlash::isValidRange(uint32_t address, uint32_t length) {
    if (_chipInfo.capacity == 0) return false;
    if (length == 0)             return true;
    return (address < _chipInfo.capacity) &&
           (length <= (_chipInfo.capacity - address));
}

// ============================================================
// Power management
// ============================================================

void LoRaE5_SPIFlash::powerDown() {
    beginTransaction();
    transfer(FLASH_CMD_POWER_DOWN);
    endTransaction();
}

// FIX 6: add tRES1 delay after CS# de-assertion.
// W25Q64 datasheet: tRES1 >= 3 us. Without this delay the first command
// after wakeup can be mis-clocked and return garbage.
void LoRaE5_SPIFlash::powerUp() {
    beginTransaction();
    transfer(FLASH_CMD_RELEASE_POWER_DOWN);
    endTransaction();
    delayMicroseconds(5);
}

// ============================================================
// Wait, reset, and recovery helpers
// ============================================================

bool LoRaE5_SPIFlash::waitForReady(uint32_t timeoutMs) {
    uint32_t start = millis();
    while (isBusy()) {
        if (millis() - start > timeoutMs) {
            return false;
        }
        delay(1);
    }
    return true;
}

void LoRaE5_SPIFlash::softwareReset() {
    beginTransaction();
    transfer(0x66);
    endTransaction();

    beginTransaction();
    transfer(0x99);
    endTransaction();

    delayMicroseconds(50);
}

void LoRaE5_SPIFlash::forceReleaseFromPowerDown() {
    beginTransaction();
    transfer(FLASH_CMD_RELEASE_POWER_DOWN);
    endTransaction();
    delayMicroseconds(50);
}

// FIX 12: write SR1 only (remove the second byte).
// Sending a second data byte to WRITE_STATUS_REG (0x01) targets SR2 on chips
// that support two-byte writes. On chips that do not, it corrupts SR1.
// We only need to clear BP bits in SR1, so one byte is correct.
void LoRaE5_SPIFlash::clearProtectionBits() {
    writeEnable();
    beginTransaction();
    transfer(FLASH_CMD_WRITE_STATUS_REG);
    transfer(0x00);
    endTransaction();
    waitForReady(100);
}

// FIX 13: reset _initialized and re-run identifyChip() so _chipInfo is
// repopulated and _initialized is set to true on success.
// The old code left _initialized false, making every API call return
// FLASH_ERROR_NO_RESPONSE even after a successful hardware recovery.
bool LoRaE5_SPIFlash::recover() {
    _initialized = false;

    forceReleaseFromPowerDown();
    softwareReset();

    if (!waitForReady(100)) {
        return false;
    }

    clearProtectionBits();

    if (!identifyChip()) {
        return false;
    }

    _initialized = true;
    return true;
}

// ============================================================
// FIX 1: Erase operations - propagate timeout from waitForReady()
// ============================================================

uint8_t LoRaE5_SPIFlash::eraseSector(uint32_t address) {
    if (!_initialized) return FLASH_ERROR_NO_RESPONSE;
    if (!isValidAddress(address)) return FLASH_ERROR_INVALID_ADDRESS;

    address = (address / _chipInfo.sectorSize) * _chipInfo.sectorSize;

    writeEnable();
    if (!isWriteEnabled()) return FLASH_ERROR_WRITE_PROTECTED;

    beginTransaction();
    transfer(FLASH_CMD_SECTOR_ERASE);
    transfer((address >> 16) & 0xFF);
    transfer((address >> 8) & 0xFF);
    transfer(address & 0xFF);
    endTransaction();

    if (!waitForReady(10000)) return FLASH_ERROR_TIMEOUT;

    return FLASH_OK;
}

uint8_t LoRaE5_SPIFlash::eraseBlock32K(uint32_t address) {
    if (!_initialized) return FLASH_ERROR_NO_RESPONSE;
    if (!isValidAddress(address)) return FLASH_ERROR_INVALID_ADDRESS;

    address = (address / 32768) * 32768;

    writeEnable();
    if (!isWriteEnabled()) return FLASH_ERROR_WRITE_PROTECTED;

    beginTransaction();
    transfer(FLASH_CMD_BLOCK_ERASE_32K);
    transfer((address >> 16) & 0xFF);
    transfer((address >> 8) & 0xFF);
    transfer(address & 0xFF);
    endTransaction();

    if (!waitForReady(30000)) return FLASH_ERROR_TIMEOUT;

    return FLASH_OK;
}

uint8_t LoRaE5_SPIFlash::eraseBlock64K(uint32_t address) {
    if (!_initialized) return FLASH_ERROR_NO_RESPONSE;
    if (!isValidAddress(address)) return FLASH_ERROR_INVALID_ADDRESS;

    address = (address / _chipInfo.blockSize) * _chipInfo.blockSize;

    writeEnable();
    if (!isWriteEnabled()) return FLASH_ERROR_WRITE_PROTECTED;

    beginTransaction();
    transfer(FLASH_CMD_BLOCK_ERASE_64K);
    transfer((address >> 16) & 0xFF);
    transfer((address >> 8) & 0xFF);
    transfer(address & 0xFF);
    endTransaction();

    if (!waitForReady(30000)) return FLASH_ERROR_TIMEOUT;

    return FLASH_OK;
}

uint8_t LoRaE5_SPIFlash::eraseChip() {
    if (!_initialized) return FLASH_ERROR_NO_RESPONSE;

    writeEnable();
    if (!isWriteEnabled()) return FLASH_ERROR_WRITE_PROTECTED;

    beginTransaction();
    transfer(FLASH_CMD_CHIP_ERASE);
    endTransaction();

    if (!waitForReady(120000)) return FLASH_ERROR_TIMEOUT;

    return FLASH_OK;
}

// ============================================================
// Byte-level read / write
// ============================================================

uint8_t LoRaE5_SPIFlash::readByteInternal(uint32_t address) {
    beginTransaction();
    transfer(FLASH_CMD_READ_DATA);
    transfer((address >> 16) & 0xFF);
    transfer((address >> 8) & 0xFF);
    transfer(address & 0xFF);
    uint8_t data = transfer(0x00);
    endTransaction();
    return data;
}

uint8_t LoRaE5_SPIFlash::readByte(uint32_t address) {
    if (!_initialized) return 0xFF;
    if (!isValidAddress(address)) return 0xFF;
    return readByteInternal(address);
}

uint8_t LoRaE5_SPIFlash::writeByteInternal(uint32_t address, uint8_t data, bool verify) {
    writeEnable();
    if (!isWriteEnabled()) return FLASH_ERROR_WRITE_PROTECTED;

    beginTransaction();
    transfer(FLASH_CMD_PAGE_PROGRAM);
    transfer((address >> 16) & 0xFF);
    transfer((address >> 8) & 0xFF);
    transfer(address & 0xFF);
    transfer(data);
    endTransaction();

    if (!waitForReady(5000)) return FLASH_ERROR_TIMEOUT;

    if (verify) {
        uint8_t readBack = readByteInternal(address);
        if (readBack != data) {
            return FLASH_ERROR_VERIFY_FAILED;
        }
    }
    return FLASH_OK;
}

uint8_t LoRaE5_SPIFlash::writeByte(uint32_t address, uint8_t data, bool verify) {
    if (!_initialized) return FLASH_ERROR_NO_RESPONSE;
    if (!isValidAddress(address)) return FLASH_ERROR_INVALID_ADDRESS;
    return writeByteInternal(address, data, verify);
}

// ============================================================
// Array read / write
// ============================================================

uint8_t LoRaE5_SPIFlash::readArray(uint32_t address, uint8_t* buffer, uint32_t length) {
    if (!_initialized) return FLASH_ERROR_NO_RESPONSE;
    if (!isValidRange(address, length)) return FLASH_ERROR_INVALID_ADDRESS;
    if (buffer == nullptr) return FLASH_ERROR_INVALID_ADDRESS;

    beginTransaction();
    transfer(FLASH_CMD_READ_DATA);
    transfer((address >> 16) & 0xFF);
    transfer((address >> 8) & 0xFF);
    transfer(address & 0xFF);

    for (uint32_t i = 0; i < length; i++) {
        buffer[i] = transfer(0x00);
    }

    endTransaction();
    return FLASH_OK;
}

uint8_t LoRaE5_SPIFlash::writeArray(uint32_t address, const uint8_t* buffer,
                                     uint32_t length, bool verify) {
    if (!_initialized) return FLASH_ERROR_NO_RESPONSE;
    if (!isValidRange(address, length)) return FLASH_ERROR_INVALID_ADDRESS;
    if (buffer == nullptr) return FLASH_ERROR_INVALID_ADDRESS;

    uint32_t bytesWritten = 0;

    while (bytesWritten < length) {
        uint32_t currentAddress = address + bytesWritten;
        uint32_t pageOffset     = currentAddress % _chipInfo.pageSize;
        uint32_t bytesThisPage  = min(length - bytesWritten,
                                      (uint32_t)(_chipInfo.pageSize - pageOffset));

        uint8_t result = writePage(currentAddress, buffer + bytesWritten,
                                   (uint16_t)bytesThisPage, verify);
        if (result != FLASH_OK) return result;

        bytesWritten += bytesThisPage;
    }

    return FLASH_OK;
}

// ============================================================
// FIX 2: writePage() - guard against page-boundary crossing
// ============================================================
// PAGE_PROGRAM silently wraps its write pointer at the 256-byte page boundary.
// Bytes that overflow wrap back to the start of the same page, corrupting data.
// Reject any call whose write would cross a boundary; writeArray() always
// splits writes correctly before calling here.
uint8_t LoRaE5_SPIFlash::writePage(uint32_t address, const uint8_t* buffer,
                                    uint16_t length, bool verify) {
    if (!_initialized) return FLASH_ERROR_NO_RESPONSE;
    if (length == 0) return FLASH_OK;

    uint32_t pageOffset = address % _chipInfo.pageSize;
    if ((uint32_t)pageOffset + (uint32_t)length > (uint32_t)_chipInfo.pageSize) {
        return FLASH_ERROR_INVALID_ADDRESS;
    }

    writeEnable();
    if (!isWriteEnabled()) return FLASH_ERROR_WRITE_PROTECTED;

    beginTransaction();
    transfer(FLASH_CMD_PAGE_PROGRAM);
    transfer((address >> 16) & 0xFF);
    transfer((address >> 8) & 0xFF);
    transfer(address & 0xFF);

    for (uint16_t i = 0; i < length; i++) {
        transfer(buffer[i]);
    }

    endTransaction();

    if (!waitForReady(5000)) return FLASH_ERROR_TIMEOUT;

    if (verify) {
        return this->verify(address, buffer, length);
    }

    return FLASH_OK;
}

// ============================================================
// Fast read
// ============================================================

uint8_t LoRaE5_SPIFlash::fastRead(uint32_t address, uint8_t* buffer, uint32_t length) {
    if (!_initialized) return FLASH_ERROR_NO_RESPONSE;
    if (!isValidRange(address, length)) return FLASH_ERROR_INVALID_ADDRESS;
    if (buffer == nullptr) return FLASH_ERROR_INVALID_ADDRESS;

    beginTransaction();
    transfer(FLASH_CMD_FAST_READ);
    transfer((address >> 16) & 0xFF);
    transfer((address >> 8) & 0xFF);
    transfer(address & 0xFF);
    transfer(0x00);

    for (uint32_t i = 0; i < length; i++) {
        buffer[i] = transfer(0x00);
    }

    endTransaction();
    return FLASH_OK;
}

// ============================================================
// FIX 7: Typed integer helpers - little-endian to match STM32 native order
// ============================================================
// STM32 (Cortex-M) is little-endian. writeStruct/readStruct use reinterpret_cast
// which preserves native (little-endian) byte order. The scalar helpers must use
// the same order so that a field written by writeStruct can be read by readUint32
// (or vice versa) without byte-swapping.
// Old code used big-endian (MSB first), which silently inverted every value.

uint8_t LoRaE5_SPIFlash::readUint16(uint32_t address, uint16_t& value) {
    uint8_t buffer[2];
    uint8_t result = readArray(address, buffer, 2);
    if (result == FLASH_OK) {
        value = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
    }
    return result;
}

uint8_t LoRaE5_SPIFlash::writeUint16(uint32_t address, uint16_t value, bool verify) {
    uint8_t buffer[2] = {
        (uint8_t)(value & 0xFF),
        (uint8_t)(value >> 8)
    };
    return writeArray(address, buffer, 2, verify);
}

uint8_t LoRaE5_SPIFlash::readUint32(uint32_t address, uint32_t& value) {
    uint8_t buffer[4];
    uint8_t result = readArray(address, buffer, 4);
    if (result == FLASH_OK) {
        value = (uint32_t)buffer[0]         |
                ((uint32_t)buffer[1] <<  8) |
                ((uint32_t)buffer[2] << 16) |
                ((uint32_t)buffer[3] << 24);
    }
    return result;
}

uint8_t LoRaE5_SPIFlash::writeUint32(uint32_t address, uint32_t value, bool verify) {
    uint8_t buffer[4] = {
        (uint8_t)(value & 0xFF),
        (uint8_t)(value >>  8),
        (uint8_t)(value >> 16),
        (uint8_t)(value >> 24)
    };
    return writeArray(address, buffer, 4, verify);
}

uint8_t LoRaE5_SPIFlash::readFloat(uint32_t address, float& value) {
    return readArray(address, reinterpret_cast<uint8_t*>(&value), sizeof(float));
}

uint8_t LoRaE5_SPIFlash::writeFloat(uint32_t address, float value, bool verify) {
    return writeArray(address, reinterpret_cast<const uint8_t*>(&value),
                      sizeof(float), verify);
}

// ============================================================
// readString()/writeString() REMOVED - Arduino String dragged in
// dtostrf->sprintf->vfprintf->malloc (~3-4 KB); unused by the firmware.
// ============================================================
#if 0
uint8_t LoRaE5_SPIFlash::readString(uint32_t address, String& str, uint16_t maxLength) {
    if (!_initialized) return FLASH_ERROR_NO_RESPONSE;
    if (!isValidRange(address, maxLength)) return FLASH_ERROR_INVALID_ADDRESS;

    str = "";

    const uint16_t CHUNK = 64;
    uint8_t  chunk[CHUNK];
    uint16_t remaining = maxLength;
    uint32_t offset    = 0;
    bool     done      = false;

    while (remaining > 0 && !done) {
        uint16_t count  = (remaining > CHUNK) ? CHUNK : remaining;
        uint8_t  result = readArray(address + offset, chunk, count);
        if (result != FLASH_OK) return result;

        for (uint16_t i = 0; i < count; i++) {
            if (chunk[i] == 0) { done = true; break; }
            str += (char)chunk[i];
        }
        offset    += count;
        remaining -= count;
    }

    return FLASH_OK;
}

// ============================================================
// FIX 11: writeString() - fixed stack buffer, no heap allocation
// ============================================================
// Dynamic allocation (new/delete) on embedded MCUs causes heap fragmentation
// and can fail silently under repeated use. A 256-byte stack buffer is
// sufficient for all reasonable strings and behaves deterministically.
uint8_t LoRaE5_SPIFlash::writeString(uint32_t address, const String& str, bool verify) {
    const uint16_t MAX_LEN = 255;
    uint16_t strLen = (uint16_t)str.length();
    if (strLen > MAX_LEN) strLen = MAX_LEN;

    uint8_t buffer[MAX_LEN + 1];
    for (uint16_t i = 0; i < strLen; i++) {
        buffer[i] = (uint8_t)str.charAt(i);
    }
    buffer[strLen] = 0;

    return writeArray(address, buffer, strLen + 1, verify);
}
#endif  // readString/writeString removed

// ============================================================
// FIX 9: isBlank() - 64-byte chunk reads
// ============================================================

bool LoRaE5_SPIFlash::isBlank(uint32_t address, uint32_t length) {
    if (!_initialized) return false;
    if (!isValidRange(address, length)) return false;

    const uint32_t CHUNK = 64;
    uint8_t  chunk[CHUNK];
    uint32_t offset = 0;

    while (offset < length) {
        uint32_t count = length - offset;
        if (count > CHUNK) count = CHUNK;

        if (readArray(address + offset, chunk, count) != FLASH_OK) return false;

        for (uint32_t i = 0; i < count; i++) {
            if (chunk[i] != 0xFF) return false;
        }
        offset += count;
    }
    return true;
}

// ============================================================
// FIX 8: verify() - 64-byte chunk reads with memcmp
// ============================================================
// Per-byte readByte() opens and closes a full SPI transaction per byte.
// 64-byte chunk reads reduce that overhead by a factor of 64.
uint8_t LoRaE5_SPIFlash::verify(uint32_t address, const uint8_t* buffer, uint32_t length) {
    const uint32_t CHUNK = 64;
    uint8_t  chunk[CHUNK];
    uint32_t offset = 0;

    while (offset < length) {
        uint32_t count = length - offset;
        if (count > CHUNK) count = CHUNK;

        uint8_t result = readArray(address + offset, chunk, count);
        if (result != FLASH_OK) return result;

        if (memcmp(chunk, buffer + offset, count) != 0) {
            return FLASH_ERROR_VERIFY_FAILED;
        }
        offset += count;
    }
    return FLASH_OK;
}

// ============================================================
// Configuration
// ============================================================

void LoRaE5_SPIFlash::setSPIFrequency(uint32_t frequency) {
    _spiFrequency = frequency;
}

// ============================================================
// Diagnostics and info
// ============================================================

void LoRaE5_SPIFlash::printChipInfo() {
    Serial.println("=== Flash Chip Information ===");
    Serial.print("Manufacturer: ");
    Serial.println(_chipInfo.manufacturer);
    Serial.print("Model: ");
    Serial.println(_chipInfo.model);
    Serial.print("JEDEC ID: 0x");
    Serial.println(_chipInfo.jedecId, HEX);
    Serial.print("Capacity: ");
    Serial.print(_chipInfo.capacity);
    Serial.println(" bytes");
    Serial.print("Page Size: ");
    Serial.print(_chipInfo.pageSize);
    Serial.println(" bytes");
    Serial.print("Sector Size: ");
    Serial.print(_chipInfo.sectorSize);
    Serial.println(" bytes");
    Serial.print("Block Size: ");
    Serial.print(_chipInfo.blockSize);
    Serial.println(" bytes");
    Serial.println("===============================");
}

void LoRaE5_SPIFlash::runDiagnostics() {
    Serial.println("=== Flash Memory Diagnostics ===");

    Serial.print("1. Communication Test: ");
    uint32_t jedecId = readJEDECID();
    if (jedecId != 0x000000 && jedecId != 0xFFFFFF) {
        Serial.println("PASS");
        Serial.print("   JEDEC ID: 0x");
        Serial.println(jedecId, HEX);
    } else {
        Serial.println("FAIL - No response from chip");
        return;
    }

    Serial.print("2. Status Register Test: ");
    uint8_t status = readStatusRegister();
    Serial.println("PASS");
    Serial.print("   Status: 0x");
    Serial.println(status, HEX);
    Serial.print("   Busy: ");
    Serial.println(isBusy() ? "Yes" : "No");
    Serial.print("   Write Enabled: ");
    Serial.println(isWriteEnabled() ? "Yes" : "No");

    Serial.print("3. Write/Read Test: ");
    uint32_t testAddr = 0x1000;
    uint8_t  testData = 0xAA;

    if (eraseSector(testAddr) == FLASH_OK) {
        if (writeByte(testAddr, testData) == FLASH_OK) {
            uint8_t readData = readByte(testAddr);
            if (readData == testData) {
                Serial.println("PASS");
            } else {
                Serial.print("FAIL - expected 0x");
                Serial.print(testData, HEX);
                Serial.print(", got 0x");
                Serial.println(readData, HEX);
            }
        } else {
            Serial.println("FAIL - Write operation");
        }
    } else {
        Serial.println("FAIL - Sector erase");
    }

    Serial.print("4. Array Operations Test: ");
    uint8_t testArray[16];
    uint8_t readBuf[16];
    for (int i = 0; i < 16; i++) {
        testArray[i] = (uint8_t)(i * 16 + i);
    }

    if (writeArray(testAddr + 0x100, testArray, 16) == FLASH_OK) {
        if (this->readArray(testAddr + 0x100, readBuf, 16) == FLASH_OK) {
            bool match = (memcmp(testArray, readBuf, 16) == 0);
            Serial.println(match ? "PASS" : "FAIL - Data mismatch");
        } else {
            Serial.println("FAIL - Array read");
        }
    } else {
        Serial.println("FAIL - Array write");
    }

    // String Operations Test removed with readString/writeString (String pulled in
    // sprintf/malloc; unused by the firmware).

    Serial.println("=== Diagnostics Complete ===");
}
