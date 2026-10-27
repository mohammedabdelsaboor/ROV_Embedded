#pragma once

#include <Arduino.h>
#include <Wire.h>

// Raw IMU reading in body units after axis sign correction.
// Mounting convention: X forward, Y left, Z up (chip face up, X arrow forward).
struct ImuSample {
    float ax = 0.0f, ay = 0.0f, az = 0.0f;  // g
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;  // deg/s, gyro bias removed
    float tempC = 0.0f;
};

// Minimal MPU6050 driver: +/-4 g, +/-500 deg/s, 44 Hz digital low-pass.
class Mpu6050 {
public:
    static constexpr uint8_t DEFAULT_ADDRESS = 0x68;  // AD0 low; 0x69 if high

    bool begin(TwoWire& wire, uint8_t address = DEFAULT_ADDRESS);

    // Multiply raw axes by +1/-1 if the board is mounted rotated 180 deg about
    // an axis. Applies to both accelerometer and gyro.
    void setAxisSigns(int8_t x, int8_t y, int8_t z);

    // Averages the gyro while the vehicle is still. Call after begin().
    bool calibrateGyro(uint16_t samples = 500);

    bool read(ImuSample& sample);

    uint8_t whoAmI() const { return whoAmI_; }

private:
    bool writeReg(uint8_t reg, uint8_t value);
    bool readRegs(uint8_t reg, uint8_t* buf, uint8_t len);
    bool readRaw(int16_t raw[7]);

    TwoWire* wire_        = nullptr;
    uint8_t  address_     = DEFAULT_ADDRESS;
    uint8_t  whoAmI_      = 0;
    int8_t   sign_[3]     = {1, 1, 1};
    float    gyroBias_[3] = {0.0f, 0.0f, 0.0f};
};

// Complementary filter. Output angles follow the ROV body convention:
//   roll  + right side down    pitch + nose up    yaw + clockwise from above
// Yaw is integrated gyro only (no magnetometer), so it drifts slowly. It is
// meant for short-term heading hold, not navigation.
class AttitudeFilter {
public:
    explicit AttitudeFilter(float gyroWeight = 0.98f) : gyroWeight_(gyroWeight) {}

    void update(const ImuSample& s, float dt);
    void resetYaw() { yaw_ = 0.0f; }

    float roll() const { return roll_; }            // deg
    float pitch() const { return pitch_; }          // deg
    float yaw() const { return yaw_; }              // deg, -180..180
    float rollRate() const { return rollRate_; }    // deg/s
    float pitchRate() const { return pitchRate_; }  // deg/s
    float yawRate() const { return yawRate_; }      // deg/s

private:
    float gyroWeight_;
    bool  initialized_ = false;
    float roll_ = 0.0f, pitch_ = 0.0f, yaw_ = 0.0f;
    float rollRate_ = 0.0f, pitchRate_ = 0.0f, yawRate_ = 0.0f;
};

// Wraps an angle in degrees to -180..180.
float wrapDegrees(float deg);
