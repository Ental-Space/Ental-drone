#ifndef LSM6DSV_H
#define LSM6DSV_H

#include <Arduino.h>
#include <Wire.h>

struct LSM6DSVData {
    float accelX; // m/s^2
    float accelY; // m/s^2
    float accelZ; // m/s^2
    float gyroX;  // degrees/s
    float gyroY;  // degrees/s
    float gyroZ;  // degrees/s
};

class LSM6DSV {
public:
    // Call Wire.begin(...) before begin(). The default I2C address is 0x6A;
    // use 0x6B if the SA0 pin is high.
    bool begin(uint8_t address = 0x6A, TwoWire *wire = &Wire);
    void setDebug(bool enabled) { debug_ = enabled; }

    // Reads one sample from all six axes. Returns false on an I2C error or
    // when called before successful initialization.
    bool readIMU(LSM6DSVData &data);

private:
    TwoWire *wire_ = nullptr;
    uint8_t address_ = 0x6A;
    bool initialized_ = false;
    bool debug_ = false;

    bool readRegisters(uint8_t reg, uint8_t *data, size_t length);
    bool writeRegister(uint8_t reg, uint8_t value);
};

#endif
