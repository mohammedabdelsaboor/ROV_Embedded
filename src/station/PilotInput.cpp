#include "PilotInput.hpp"

#include "config.hpp"

void Button::begin() {
    pinMode(pin_, INPUT_PULLUP);
}

void Button::update(uint32_t nowMs) {
    const bool down = digitalRead(pin_) == LOW;
    if (down != lastReading_) {
        lastReading_  = down;
        lastChangeMs_ = nowMs;
    }
    if (down != stableDown_ && nowMs - lastChangeMs_ >= config::DEBOUNCE_MS) {
        stableDown_ = down;
        if (down) {
            pressEvent_ = true;
        }
    }
}

bool Button::pressed() {
    const bool event = pressEvent_;
    pressEvent_      = false;
    return event;
}

void JoystickAxis::calibrate() {
    long sum = 0;
    for (uint8_t i = 0; i < 16; ++i) {
        sum += analogRead(pin_);
        delay(2);
    }
    center_ = sum / 16;
}

int16_t JoystickAxis::read() const {
    const int offset = analogRead(pin_) - center_;
    long      value  = 0;
    if (offset > config::JOY_DEADBAND) {
        value = (long)(offset - config::JOY_DEADBAND) * rov::AXIS_MAX /
                (1023 - center_ - config::JOY_DEADBAND);
    } else if (offset < -config::JOY_DEADBAND) {
        value = (long)(offset + config::JOY_DEADBAND) * rov::AXIS_MAX /
                (center_ - config::JOY_DEADBAND);
    }
    value = constrain(value, -rov::AXIS_MAX, rov::AXIS_MAX);
    return static_cast<int16_t>(inverted_ ? -value : value);
}

PilotInput::PilotInput()
    : surge_(config::PIN_SURGE, config::INVERT_SURGE),
      sway_(config::PIN_SWAY, config::INVERT_SWAY),
      heave_(config::PIN_HEAVE, config::INVERT_HEAVE),
      yaw_(config::PIN_YAW, config::INVERT_YAW),
      arm_(config::PIN_BTN_ARM),
      mode_(config::PIN_BTN_MODE),
      gainUp_(config::PIN_BTN_GAIN_UP),
      gainDown_(config::PIN_BTN_GAIN_DOWN) {}

void PilotInput::begin() {
    arm_.begin();
    mode_.begin();
    gainUp_.begin();
    gainDown_.begin();
    if (config::USE_JOYSTICKS) {
        surge_.calibrate();
        sway_.calibrate();
        heave_.calibrate();
        yaw_.calibrate();
    }
}

void PilotInput::update(uint32_t nowMs) {
    arm_.update(nowMs);
    mode_.update(nowMs);
    gainUp_.update(nowMs);
    gainDown_.update(nowMs);
}

rov::MotionCmd PilotInput::motion() const {
    rov::MotionCmd cmd;
    if (config::USE_JOYSTICKS) {
        cmd.surge = surge_.read();
        cmd.sway  = sway_.read();
        cmd.heave = heave_.read();
        cmd.yaw   = yaw_.read();
    }
    return cmd;
}
