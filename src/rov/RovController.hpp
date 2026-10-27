#pragma once

#include <Mpu6050.hpp>
#include <Pid.hpp>
#include <RovProtocol.hpp>
#include <ThrusterMixer.hpp>

// Vehicle logic: arming, link failsafe, stabilize mode, motor test and the
// out-of-water run limit. Hardware-free: it turns commands + attitude into six
// thruster outputs in [-1, 1], which main.cpp scales and sends to the ESCs.
class RovController {
public:
    static constexpr uint8_t NUM_MOTORS = ThrusterMixer::NUM_MOTORS;

    RovController();

    void onMotion(const rov::MotionCmd& cmd);
    void onState(const rov::StateCmd& cmd, uint32_t nowMs);
    void onMotorTest(const rov::MotorTestCmd& cmd, uint32_t nowMs);

    // imuOk: when false, stabilize mode behaves like manual mode.
    void update(const AttitudeFilter& attitude, bool imuOk, float dt, uint32_t nowMs,
                float out[NUM_MOTORS]);

    bool      armed() const { return armed_; }
    rov::Mode mode() const { return mode_; }
    uint8_t   gainPercent() const { return gainPercent_; }
    uint8_t   lastSeq() const { return seq_; }
    uint8_t   statusFlags() const;

private:
    bool pilotNeutral() const;
    void disarm();

    rov::MotionCmd motion_;
    rov::Mode      mode_        = rov::Mode::Manual;
    uint8_t        gainPercent_ = 0;
    uint8_t        seq_         = 0;
    bool           armed_       = false;
    bool           linkSeen_    = false;
    bool           failsafe_    = true;
    // Set at boot and on link loss: arming needs an explicit disarm first, so
    // a station that still requests "armed" can't re-arm the ROV by itself.
    bool     rearmBlocked_ = true;
    uint32_t lastStateMs_  = 0;

    Pid   rollPid_;
    Pid   yawPid_;
    bool  holdingHeading_ = false;
    float headingTarget_  = 0.0f;

    uint8_t  testMotor_    = 0;  // 0 = no test running
    float    testThrottle_ = 0.0f;
    uint32_t testEndMs_    = 0;

    uint32_t dryRunStartMs_ = 0;
    bool     dryRunning_    = false;
    bool     dryTimeout_    = false;
};
