#include "Mpu6050.hpp"

#include <math.h>

namespace {
constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_WHO_AM_I     = 0x75;

constexpr float ACCEL_LSB_PER_G  = 8192.0f;  // +/-4 g
constexpr float GYRO_LSB_PER_DPS = 65.5f;    // +/-500 deg/s
constexpr float RAD_TO_DEGF      = 57.2957795f;
}  // namespace

bool Mpu6050::begin(TwoWire& wire, uint8_t address) {
    wire_    = &wire;
    address_ = address;

    if (!writeReg(REG_PWR_MGMT_1, 0x80)) {  // device reset
        return false;
    }
    delay(100);
    if (!readRegs(REG_WHO_AM_I, &whoAmI_, 1)) {
        return false;
    }
    // Genuine parts report 0x68; many clones report something else but work
    // the same, so the value is only exposed for diagnostics.
    return writeReg(REG_PWR_MGMT_1, 0x01)        // wake, clock from gyro X PLL
           && writeReg(REG_CONFIG, 0x03)         // DLPF 44 Hz
           && writeReg(REG_SMPLRT_DIV, 4)        // 1 kHz / (1 + 4) = 200 Hz
           && writeReg(REG_GYRO_CONFIG, 0x08)    // +/-500 deg/s
           && writeReg(REG_ACCEL_CONFIG, 0x08);  // +/-4 g
}

void Mpu6050::setAxisSigns(int8_t x, int8_t y, int8_t z) {
    sign_[0] = x < 0 ? -1 : 1;
    sign_[1] = y < 0 ? -1 : 1;
    sign_[2] = z < 0 ? -1 : 1;
}

bool Mpu6050::calibrateGyro(uint16_t samples) {
    float   sum[3] = {0.0f, 0.0f, 0.0f};
    int16_t raw[7];
    for (uint16_t n = 0; n < samples; ++n) {
        if (!readRaw(raw)) {
            return false;
        }
        for (uint8_t i = 0; i < 3; ++i) {
            sum[i] += raw[4 + i] / GYRO_LSB_PER_DPS;
        }
        delay(2);
    }
    for (uint8_t i = 0; i < 3; ++i) {
        gyroBias_[i] = sum[i] / samples;
    }
    return true;
}

bool Mpu6050::read(ImuSample& s) {
    int16_t raw[7];
    if (!readRaw(raw)) {
        return false;
    }
    s.ax    = sign_[0] * raw[0] / ACCEL_LSB_PER_G;
    s.ay    = sign_[1] * raw[1] / ACCEL_LSB_PER_G;
    s.az    = sign_[2] * raw[2] / ACCEL_LSB_PER_G;
    s.tempC = raw[3] / 340.0f + 36.53f;
    s.gx    = sign_[0] * (raw[4] / GYRO_LSB_PER_DPS - gyroBias_[0]);
    s.gy    = sign_[1] * (raw[5] / GYRO_LSB_PER_DPS - gyroBias_[1]);
    s.gz    = sign_[2] * (raw[6] / GYRO_LSB_PER_DPS - gyroBias_[2]);
    return true;
}

bool Mpu6050::readRaw(int16_t raw[7]) {
    uint8_t buf[14];
    if (!readRegs(REG_ACCEL_XOUT_H, buf, sizeof(buf))) {
        return false;
    }
    for (uint8_t i = 0; i < 7; ++i) {
        raw[i] = static_cast<int16_t>((buf[2 * i] << 8) | buf[2 * i + 1]);
    }
    return true;
}

bool Mpu6050::writeReg(uint8_t reg, uint8_t value) {
    wire_->beginTransmission(address_);
    wire_->write(reg);
    wire_->write(value);
    return wire_->endTransmission() == 0;
}

bool Mpu6050::readRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
    wire_->beginTransmission(address_);
    wire_->write(reg);
    if (wire_->endTransmission(false) != 0) {
        return false;
    }
    if (wire_->requestFrom(address_, len) != len) {
        return false;
    }
    for (uint8_t i = 0; i < len; ++i) {
        buf[i] = wire_->read();
    }
    return true;
}

float wrapDegrees(float deg) {
    while (deg > 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

void AttitudeFilter::update(const ImuSample& s, float dt) {
    // Sensor axes are X forward, Y left, Z up. Convert rates to the body
    // convention used by the controller (+pitch nose up, +yaw clockwise).
    rollRate_  = s.gx;
    pitchRate_ = -s.gy;
    yawRate_   = -s.gz;

    const float accRoll  = atan2f(s.ay, s.az) * RAD_TO_DEGF;
    const float accPitch = atan2f(s.ax, sqrtf(s.ay * s.ay + s.az * s.az)) * RAD_TO_DEGF;

    if (!initialized_) {
        roll_        = accRoll;
        pitch_       = accPitch;
        initialized_ = true;
        return;
    }

    roll_ += rollRate_ * dt;
    pitch_ += pitchRate_ * dt;
    yaw_ = wrapDegrees(yaw_ + yawRate_ * dt);

    // Only trust the accelerometer when it mostly measures gravity.
    const float accNorm = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
    if (accNorm > 0.85f && accNorm < 1.15f) {
        roll_  = gyroWeight_ * roll_ + (1.0f - gyroWeight_) * accRoll;
        pitch_ = gyroWeight_ * pitch_ + (1.0f - gyroWeight_) * accPitch;
    }
}
