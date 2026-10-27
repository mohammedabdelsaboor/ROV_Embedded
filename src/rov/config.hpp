#pragma once

#include <stdint.h>

// ROV configuration: ESP32-S3 PowerFeather.
// Header labels are in comments, GPIO numbers in code.
namespace config {

// ---- IMU (MPU6050 on the header I2C pins) --------------------------------
constexpr int      PIN_IMU_SDA = 35;  // SDA
constexpr int      PIN_IMU_SCL = 36;  // SCL
constexpr uint32_t IMU_I2C_HZ  = 400000;
// Board mounted X forward, Y left, Z up. If it is mounted upside down, use
// {1, -1, -1}; if it is turned around (X backward), use {-1, -1, 1}.
constexpr int8_t IMU_AXIS_SIGN[3] = {1, 1, 1};

// ---- CAN transceiver (SN65HVD230 / TJA1051T/3, 3.3 V logic) ---------------
constexpr int PIN_CAN_TX = 10;  // A0 -> transceiver TXD / CTX
constexpr int PIN_CAN_RX = 9;   // A1 -> transceiver RXD / CRX

// ---- Status LED (on-board green LED) --------------------------------------
constexpr int PIN_LED = 46;

// ---- Thrusters (Basic ESC signal wires) -----------------------------------
// Index 0..5 = T1..T6, see lib/ThrusterMixer for the frame layout.
constexpr uint8_t PIN_THRUSTER[6] = {
    15,  // D5  T1 front-right
    16,  // D6  T2 front-left
    17,  // D9  T3 rear-right
    18,  // D10 T4 rear-left
    12,  // D12 T5 vertical right
    11,  // D13 T6 vertical left
};
// Flip a thruster that pushes the wrong way (CCW prop or reversed wiring).
// Check each one with the station's "test" command.
constexpr bool THRUSTER_REVERSED[6] = {false, false, false, false, false, false};

// ---- Output limits ----------------------------------------------------------
// Set ROV_OUT_OF_WATER=0 in platformio.ini when the vehicle is in the water.
#ifndef ROV_OUT_OF_WATER
#define ROV_OUT_OF_WATER 1
#endif
constexpr bool  OUT_OF_WATER     = ROV_OUT_OF_WATER;
constexpr float OUTPUT_LIMIT_WET = 1.00f;  // fraction of full thrust
constexpr float OUTPUT_LIMIT_DRY = 0.15f;  // T200s are water-cooled
// Out of water, outputs are cut after running this long; disarm to reset.
constexpr uint32_t DRY_MAX_RUN_MS = 10000;

constexpr uint32_t MOTOR_TEST_MAX_MS = 3000;

// ---- Control ----------------------------------------------------------------
constexpr uint32_t CONTROL_PERIOD_US   = 10000;  // 100 Hz
constexpr uint32_t TELEMETRY_PERIOD_MS = 100;    // 10 Hz
constexpr uint32_t SUPPLY_PERIOD_MS    = 1000;
constexpr uint32_t DEBUG_PERIOD_MS     = 500;  // USB serial status print

// Pilot inputs must be inside this band (fraction of full scale) to arm.
constexpr float ARM_DEADBAND = 0.05f;
// Heading hold engages when the yaw stick is inside this band.
constexpr float YAW_HOLD_DEADBAND = 0.05f;

// Stabilize mode. Error in degrees, output in fraction of full thrust.
constexpr float ROLL_KP      = 0.020f;
constexpr float ROLL_KI      = 0.005f;
constexpr float ROLL_KD      = 0.004f;  // per deg/s
constexpr float ROLL_I_LIMIT = 0.20f;
constexpr float ROLL_OUT_MAX = 0.50f;

constexpr float YAW_KP      = 0.015f;
constexpr float YAW_KI      = 0.000f;
constexpr float YAW_KD      = 0.003f;  // per deg/s
constexpr float YAW_I_LIMIT = 0.20f;
constexpr float YAW_OUT_MAX = 0.50f;

}  // namespace config
