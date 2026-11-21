#include "LSM6DSV.h"

namespace {
constexpr uint8_t REG_CTRL3_C = 0x12;
constexpr uint8_t REG_CTRL1_XL = 0x10;
constexpr uint8_t REG_CTRL2_G = 0x11;
constexpr uint8_t REG_CTRL8_XL = 0x17;
constexpr uint8_t REG_OUTX_L_A = 0x28;
constexpr uint8_t REG_WHO_AM_I = 0x0F;

constexpr uint8_t LSM6DSV_WHO_AM_I = 0x70;
constexpr uint8_t CTRL3_SW_RESET = 0x01;
constexpr uint8_t CTRL3_BDU_IF_INC = 0x44;
constexpr uint8_t ODR_120_HZ = 0x06;
constexpr uint8_t ACCEL_FS_16_G = 0x03;

constexpr float ACCEL_M_S2_PER_LSB = 0.000488f * 9.80665f;
constexpr float ACCEL_RAD_TO_DEG = 57.2957795131f;

int16_t decodeInt16(const uint8_t *bytes) {
    const uint16_t value = uint16_t(bytes[0]) | (uint16_t(bytes[1]) << 8);
    const int32_t signedValue = (value & 0x8000) ? int32_t(value) - 0x10000 : int32_t(value);
    return int16_t(signedValue);
}
}

bool LSM6DSV::readRegisters(uint8_t reg, uint8_t *data, size_t length) {
    if (!wire_ || !data || length == 0) return false;

    wire_->beginTransmission(address_);
    wire_->write(reg);
    if (wire_->endTransmission(false) != 0) return false;

    const size_t received = wire_->requestFrom(address_, length);
    if (received != length) {
        while (wire_->available()) wire_->read();
        return false;
    }

    for (size_t i = 0; i < length; ++i) {
        const int value = wire_->read();
        if (value < 0) return false;
        data[i] = uint8_t(value);
    }
    return true;
}

bool LSM6DSV::writeRegister(uint8_t reg, uint8_t value) {
    if (!wire_) return false;

    wire_->beginTransmission(address_);
    wire_->write(reg);
    wire_->write(value);
    return wire_->endTransmission() == 0;
}

bool LSM6DSV::begin(uint8_t address, TwoWire *wire) {
    initialized_ = false;
    address_ = address;
    wire_ = wire;
    if (!wire_ || address_ > 0x7F) return false;

    uint8_t whoAmI = 0;
    bool whoAmIRead = readRegisters(REG_WHO_AM_I, &whoAmI, 1);
    if (!whoAmIRead || whoAmI != LSM6DSV_WHO_AM_I) {
        const uint8_t requestedAddress = address_;
        const uint8_t requestedWhoAmI = whoAmI;
        const bool requestedRead = whoAmIRead;
        address_ = (address_ == 0x6A) ? 0x6B : 0x6A;
        whoAmI = 0;
        whoAmIRead = readRegisters(REG_WHO_AM_I, &whoAmI, 1);
        if (!whoAmIRead || whoAmI != LSM6DSV_WHO_AM_I) {
            if (debug_) {
                Serial.printf("LSM6DSV: WHO_AM_I failed at 0x%02X (read=%s, value=0x%02X) and 0x%02X (read=%s, value=0x%02X); expected 0x%02X\n",
                              requestedAddress, requestedRead ? "OK" : "FAILED", requestedWhoAmI, address_,
                              whoAmIRead ? "OK" : "FAILED", whoAmI, LSM6DSV_WHO_AM_I);
            }
            address_ = requestedAddress;
            return false;
        }
        if (debug_) {
            Serial.printf("LSM6DSV: detected at alternate I2C address 0x%02X (WHO_AM_I=0x%02X)\n",
                          address_, whoAmI);
        }
    } else if (debug_) {
        Serial.printf("LSM6DSV: detected at I2C address 0x%02X (WHO_AM_I=0x%02X)\n",
                      address_, whoAmI);
    }

    // Reset to a known state. The gyroscope remains disabled.
    if (!writeRegister(REG_CTRL3_C, CTRL3_SW_RESET)) return false;

    bool resetComplete = false;
    for (uint8_t i = 0; i < 100; ++i) {
        uint8_t ctrl3 = 0;
        if (!readRegisters(REG_CTRL3_C, &ctrl3, 1)) return false;
        if ((ctrl3 & CTRL3_SW_RESET) == 0) {
            resetComplete = true;
            break;
        }
        delay(1);
    }
    if (!resetComplete) return false;

    // Keep a coherent multi-byte sample and enable register auto-increment.
    if (!writeRegister(REG_CTRL3_C, CTRL3_BDU_IF_INC) ||
        !writeRegister(REG_CTRL1_XL, ODR_120_HZ) ||
        !writeRegister(REG_CTRL2_G, 0x00) ||
        !writeRegister(REG_CTRL8_XL, ACCEL_FS_16_G)) {
        return false;
    }

    delay(10); // Allow the first 120 Hz samples to become available.
    initialized_ = true;
    return true;
}

bool LSM6DSV::readAccelerometer(LSM6DSVData &data) {
    if (!initialized_) return false;

    uint8_t raw[6];
    if (!readRegisters(REG_OUTX_L_A, raw, sizeof(raw))) return false;

    const float accelX = float(decodeInt16(&raw[0])) * ACCEL_M_S2_PER_LSB;
    const float accelY = float(decodeInt16(&raw[2])) * ACCEL_M_S2_PER_LSB;
    const float accelZ = float(decodeInt16(&raw[4])) * ACCEL_M_S2_PER_LSB;
    const float horizontalMagnitude = sqrtf(accelX * accelX + accelY * accelY);

    data.accelMagnitude = sqrtf(horizontalMagnitude * horizontalMagnitude + accelZ * accelZ);
    const float deviationFromVertical = atan2f(horizontalMagnitude, fabsf(accelZ)) * ACCEL_RAD_TO_DEG;
    data.accelDeviation = accelZ < 0.0f
        ? -(180.0f - deviationFromVertical)
        : deviationFromVertical;
    return true;
}
