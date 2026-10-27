#include "Thruster.hpp"

namespace {
constexpr float HALF_RANGE = (Thruster::PULSE_MAX - Thruster::PULSE_MIN) / 2.0f;

#if defined(ARDUINO_ARCH_ESP32)
// 50 Hz servo frame, 14-bit duty: ~1.2 us resolution.
constexpr uint32_t PWM_FREQ_HZ   = 50;
constexpr uint8_t  PWM_RES_BITS  = 14;
constexpr uint32_t PWM_PERIOD_US = 1000000UL / PWM_FREQ_HZ;
constexpr uint32_t PWM_MAX_DUTY  = (1UL << PWM_RES_BITS) - 1;
#endif
}  // namespace

Thruster::Thruster(uint8_t pin, bool reversed) : pin_(pin), reversed_(reversed) {}

void Thruster::begin() {
    if (!attached_) {
#if defined(ARDUINO_ARCH_ESP32)
        ledcAttach(pin_, PWM_FREQ_HZ, PWM_RES_BITS);
#else
        esc_.attach(pin_, PULSE_MIN, PULSE_MAX);
#endif
        attached_ = true;
    }
    stop();
}

void Thruster::end() {
    if (attached_) {
        stop();
#if defined(ARDUINO_ARCH_ESP32)
        ledcDetach(pin_);
#else
        esc_.detach();
#endif
        attached_ = false;
    }
}

void Thruster::setThrottle(float throttle) {
    throttle = constrain(throttle, -maxThrottle_, maxThrottle_);
    if (reversed_) {
        throttle = -throttle;
    }
    setPulse(static_cast<uint16_t>(PULSE_STOP + throttle * HALF_RANGE + 0.5f));
}

void Thruster::setPulse(uint16_t us) {
    const uint16_t window = static_cast<uint16_t>(maxThrottle_ * HALF_RANGE + 0.5f);
    pulse_                = constrain(us, PULSE_STOP - window, PULSE_STOP + window);

    throttle_ = (static_cast<int16_t>(pulse_) - static_cast<int16_t>(PULSE_STOP)) / HALF_RANGE;
    if (reversed_) {
        throttle_ = -throttle_;
    }

    if (attached_) {
        writePulse(pulse_);
    }
}

void Thruster::writePulse(uint16_t us) {
#if defined(ARDUINO_ARCH_ESP32)
    ledcWrite(pin_, static_cast<uint32_t>(us) * PWM_MAX_DUTY / PWM_PERIOD_US);
#else
    esc_.writeMicroseconds(us);
#endif
}

void Thruster::stop() {
    setPulse(PULSE_STOP);
}

void Thruster::setMaxThrottle(float limit) {
    maxThrottle_ = constrain(limit, 0.0f, 1.0f);
    setThrottle(throttle_);
}

void Thruster::armAll(Thruster* thrusters, size_t count, uint16_t waitMs) {
    for (size_t i = 0; i < count; ++i) {
        thrusters[i].begin();
    }
    delay(waitMs);
}
