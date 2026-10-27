#pragma once

#include <Arduino.h>

#if !defined(ARDUINO_ARCH_ESP32)
#include <Servo.h>
#endif

// Driver for a Blue Robotics thruster (T100 / T200) driven by a Basic ESC.
//
// The ESC takes a standard servo PWM signal:
//   1100 us = full reverse, 1500 us = stop, 1900 us = full forward.
// The ESC must see the stop signal (1500 us) for a few seconds after power-up
// before it will accept commands (arming).
//
// Backends: AVR uses the Servo library, ESP32 uses the LEDC PWM peripheral.
class Thruster {
public:
    static constexpr uint16_t PULSE_MIN   = 1100;
    static constexpr uint16_t PULSE_STOP  = 1500;
    static constexpr uint16_t PULSE_MAX   = 1900;
    static constexpr uint16_t ARM_TIME_MS = 7000;

    // pin:      any PWM-capable pin
    // reversed: flip direction in software (e.g. for CW/CCW propeller pairs)
    explicit Thruster(uint8_t pin, bool reversed = false);

    // Attaches the PWM output and sends the stop signal.
    void begin();

    // Releases the PWM pin. The ESC will stop driving the motor.
    void end();

    // Throttle in the range [-1.0, 1.0]. Values are clamped to +/- maxThrottle.
    void setThrottle(float throttle);

    // Raw pulse width in microseconds. Clamped to [PULSE_MIN, PULSE_MAX] and
    // to the window allowed by maxThrottle, so the limit cannot be bypassed.
    void setPulse(uint16_t us);

    void stop();

    // Limits output magnitude (0.0 - 1.0). Useful for bench testing.
    void setMaxThrottle(float limit);

    void setReversed(bool reversed) { reversed_ = reversed; }

    float    maxThrottle() const { return maxThrottle_; }
    float    throttle() const { return throttle_; }
    uint16_t pulse() const { return pulse_; }
    uint8_t  pin() const { return pin_; }
    bool     attached() const { return attached_; }

    // Sends stop to every given thruster and waits for the ESCs to arm.
    static void armAll(Thruster* thrusters, size_t count, uint16_t waitMs = ARM_TIME_MS);

private:
    void writePulse(uint16_t us);

#if !defined(ARDUINO_ARCH_ESP32)
    Servo esc_;
#endif
    uint8_t  pin_;
    bool     reversed_;
    bool     attached_    = false;
    float    maxThrottle_ = 1.0f;
    float    throttle_    = 0.0f;
    uint16_t pulse_       = PULSE_STOP;
};
