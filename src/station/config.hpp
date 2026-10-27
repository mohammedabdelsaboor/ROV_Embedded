#pragma once

#include <Arduino.h>

// Surface station configuration: Arduino Uno + MCP2515 CAN module.
namespace config {

// ---- MCP2515 module (SPI: D11 MOSI, D12 MISO, D13 SCK) ----------------------
constexpr uint8_t PIN_CAN_CS = 10;
// Crystal on the MCP2515 module: most blue modules have 8 MHz, some 16 MHz.
// Read the marking on the metal can next to the chip.
#define STATION_MCP_CLOCK MCP_8MHZ

// ---- Joysticks (2 x analog thumbsticks, e.g. KY-023) ------------------------
// Left stick:  Y = heave (up/down), X = yaw (turn)
// Right stick: Y = surge (forward), X = sway (sideways)
#ifndef STATION_USE_JOYSTICKS
#define STATION_USE_JOYSTICKS 1
#endif
constexpr bool    USE_JOYSTICKS = STATION_USE_JOYSTICKS;
constexpr uint8_t PIN_HEAVE     = A0;  // left Y
constexpr uint8_t PIN_YAW       = A1;  // left X
constexpr uint8_t PIN_SURGE     = A2;  // right Y
constexpr uint8_t PIN_SWAY      = A3;  // right X
// Flip an axis that moves the wrong way.
constexpr bool INVERT_HEAVE = false;
constexpr bool INVERT_YAW   = false;
constexpr bool INVERT_SURGE = false;
constexpr bool INVERT_SWAY  = false;
constexpr int  JOY_DEADBAND = 40;  // ADC counts around the center

// ---- Buttons (to GND, internal pull-ups) -------------------------------------
constexpr uint8_t  PIN_BTN_ARM       = 3;  // toggle arm / disarm
constexpr uint8_t  PIN_BTN_MODE      = 4;  // toggle manual / stabilize
constexpr uint8_t  PIN_BTN_GAIN_UP   = 5;
constexpr uint8_t  PIN_BTN_GAIN_DOWN = 6;
constexpr uint16_t DEBOUNCE_MS       = 30;

// ---- LEDs ----------------------------------------------------------------------
constexpr uint8_t PIN_LED_ARMED = 7;  // on = ROV reports armed
constexpr uint8_t PIN_LED_LINK  = 8;  // on = telemetry received

// ---- Behaviour -------------------------------------------------------------------
constexpr uint8_t GAIN_DEFAULT = 40;  // percent
constexpr uint8_t GAIN_STEP    = 10;
constexpr uint8_t GAIN_MIN     = 10;
constexpr uint8_t GAIN_MAX     = 100;

constexpr uint16_t CMD_PERIOD_MS    = 50;    // 20 Hz to the ROV
constexpr uint16_t PRINT_PERIOD_MS  = 1000;  // telemetry line on serial
constexpr uint16_t TLM_TIMEOUT_MS   = 1000;  // no telemetry -> link lost
constexpr uint8_t  TEST_MAX_SECONDS = 3;

}  // namespace config
