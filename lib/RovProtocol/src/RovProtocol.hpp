#pragma once

#include <stddef.h>
#include <stdint.h>

// CAN bus protocol between the surface station (Arduino Uno + MCP2515) and the
// ROV (ESP32-S3 PowerFeather + TWAI transceiver).
//
// Classic CAN 2.0A, 11-bit ids, little-endian payloads. Lower id = higher
// priority, so commands (0x1xx) win arbitration over telemetry (0x2xx).
namespace rov {

constexpr uint32_t CAN_BITRATE = 250000;  // good up to ~250 m of tether

// Station -> ROV
constexpr uint16_t ID_CMD_MOTION     = 0x100;  // MotionCmd, 20 Hz
constexpr uint16_t ID_CMD_STATE      = 0x101;  // StateCmd, 20 Hz (heartbeat)
constexpr uint16_t ID_CMD_MOTOR_TEST = 0x102;  // MotorTestCmd, on demand
// ROV -> station
constexpr uint16_t ID_TLM_ATTITUDE  = 0x200;  // AttitudeTlm, 10 Hz
constexpr uint16_t ID_TLM_STATUS    = 0x201;  // StatusTlm, 10 Hz
constexpr uint16_t ID_TLM_THRUSTERS = 0x202;  // ThrusterTlm, 10 Hz

constexpr uint8_t NUM_THRUSTERS = 6;
constexpr int16_t AXIS_MAX      = 1000;  // full-scale pilot input

// ROV stops all thrusters and disarms if no StateCmd arrives for this long.
constexpr uint16_t LINK_TIMEOUT_MS = 500;

struct CanFrame {
    uint16_t id      = 0;
    uint8_t  len     = 0;
    uint8_t  data[8] = {};
};

enum class Mode : uint8_t {
    Manual    = 0,  // pilot inputs go straight to the mixer
    Stabilize = 1,  // roll held level, heading held when yaw stick is centered
};

// Pilot inputs, each in [-AXIS_MAX, AXIS_MAX].
//   surge  + forward     sway + right
//   heave  + up          yaw  + turn right (clockwise seen from above)
struct MotionCmd {
    int16_t surge = 0;
    int16_t sway  = 0;
    int16_t heave = 0;
    int16_t yaw   = 0;
};

struct StateCmd {
    bool    armed = false;
    Mode    mode  = Mode::Manual;
    uint8_t gain  = 0;  // pilot gain in percent (0-100)
    uint8_t seq   = 0;  // increments every frame, echoed in StatusTlm
};

// Spins one thruster (motor 1-6) while the ROV is disarmed, to check numbering
// and direction. Throttle is still capped by the ROV's output limit.
struct MotorTestCmd {
    uint8_t motor      = 0;  // 1..6
    int8_t  throttle   = 0;  // percent, -100..100
    uint8_t durationDs = 0;  // deciseconds, ROV caps it at 3 s
};

struct AttitudeTlm {
    int16_t roll    = 0;  // centidegrees, + right side down
    int16_t pitch   = 0;  // centidegrees, + nose up
    int16_t yaw     = 0;  // centidegrees, gyro-integrated heading (drifts)
    int16_t yawRate = 0;  // centidegrees / s
};

namespace status {
constexpr uint8_t ARMED        = 1 << 0;
constexpr uint8_t IMU_OK       = 1 << 1;
constexpr uint8_t FAILSAFE     = 1 << 2;  // link lost, outputs stopped
constexpr uint8_t OUT_OF_WATER = 1 << 3;  // dry-test output limit active
constexpr uint8_t DRY_TIMEOUT  = 1 << 4;  // ran too long out of water
constexpr uint8_t MOTOR_TEST   = 1 << 5;
}  // namespace status

struct StatusTlm {
    uint8_t  flags    = 0;
    Mode     mode     = Mode::Manual;
    uint8_t  gain     = 0;  // percent
    uint8_t  seq      = 0;  // last StateCmd seq received
    uint16_t supplyMv = 0;  // PowerFeather supply voltage, mV
    int16_t  imuTemp  = 0;  // centidegrees C
};

struct ThrusterTlm {
    int8_t output[NUM_THRUSTERS] = {};  // percent of full scale, -100..100
};

void encode(const MotionCmd& msg, CanFrame& frame);
void encode(const StateCmd& msg, CanFrame& frame);
void encode(const MotorTestCmd& msg, CanFrame& frame);
void encode(const AttitudeTlm& msg, CanFrame& frame);
void encode(const StatusTlm& msg, CanFrame& frame);
void encode(const ThrusterTlm& msg, CanFrame& frame);

// Each decode returns false if the frame id or length does not match.
bool decode(const CanFrame& frame, MotionCmd& msg);
bool decode(const CanFrame& frame, StateCmd& msg);
bool decode(const CanFrame& frame, MotorTestCmd& msg);
bool decode(const CanFrame& frame, AttitudeTlm& msg);
bool decode(const CanFrame& frame, StatusTlm& msg);
bool decode(const CanFrame& frame, ThrusterTlm& msg);

}  // namespace rov
