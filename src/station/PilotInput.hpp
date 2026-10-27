#pragma once

#include <Arduino.h>
#include <RovProtocol.hpp>

// Debounced push button to GND (internal pull-up). pressed() reports each
// press once.
class Button {
public:
    explicit Button(uint8_t pin) : pin_(pin) {}

    void begin();
    void update(uint32_t nowMs);
    bool pressed();

private:
    uint8_t  pin_;
    bool     stableDown_   = false;
    bool     lastReading_  = false;
    bool     pressEvent_   = false;
    uint32_t lastChangeMs_ = 0;
};

// One analog thumbstick axis, centered at boot and scaled to +/-AXIS_MAX.
class JoystickAxis {
public:
    JoystickAxis(uint8_t pin, bool inverted) : pin_(pin), inverted_(inverted) {}

    // Samples the resting position. Leave the sticks centered during boot.
    void    calibrate();
    int16_t read() const;

private:
    uint8_t pin_;
    bool    inverted_;
    int     center_ = 512;
};

// Joysticks + buttons of the surface station.
class PilotInput {
public:
    PilotInput();

    void begin();
    void update(uint32_t nowMs);

    rov::MotionCmd motion() const;

    bool armPressed() { return arm_.pressed(); }
    bool modePressed() { return mode_.pressed(); }
    bool gainUpPressed() { return gainUp_.pressed(); }
    bool gainDownPressed() { return gainDown_.pressed(); }

private:
    JoystickAxis surge_, sway_, heave_, yaw_;
    Button       arm_, mode_, gainUp_, gainDown_;
};
