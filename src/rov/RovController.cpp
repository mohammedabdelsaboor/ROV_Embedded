#include "RovController.hpp"

#include <math.h>

#include "config.hpp"

namespace {
float axis(int16_t v) {
    if (v > rov::AXIS_MAX) {
        v = rov::AXIS_MAX;
    } else if (v < -rov::AXIS_MAX) {
        v = -rov::AXIS_MAX;
    }
    return static_cast<float>(v) / rov::AXIS_MAX;
}
}  // namespace

RovController::RovController()
    : rollPid_(config::ROLL_KP, config::ROLL_KI, config::ROLL_KD, config::ROLL_I_LIMIT,
               config::ROLL_OUT_MAX),
      yawPid_(config::YAW_KP, config::YAW_KI, config::YAW_KD, config::YAW_I_LIMIT,
              config::YAW_OUT_MAX) {}

void RovController::onMotion(const rov::MotionCmd& cmd) {
    motion_ = cmd;
}

void RovController::onState(const rov::StateCmd& cmd, uint32_t nowMs) {
    lastStateMs_ = nowMs;
    linkSeen_    = true;
    failsafe_    = false;
    seq_         = cmd.seq;
    gainPercent_ = cmd.gain;

    if (cmd.mode != mode_) {
        mode_ = cmd.mode;
        rollPid_.reset();
        yawPid_.reset();
        holdingHeading_ = false;
    }

    if (!cmd.armed) {
        disarm();
        rearmBlocked_ = false;
    } else if (!armed_ && !rearmBlocked_ && pilotNeutral()) {
        // Only arm with the sticks centered so the ROV can't lurch on arming.
        armed_     = true;
        testMotor_ = 0;
        rollPid_.reset();
        yawPid_.reset();
        holdingHeading_ = false;
    }
}

void RovController::onMotorTest(const rov::MotorTestCmd& cmd, uint32_t nowMs) {
    if (armed_ || cmd.motor < 1 || cmd.motor > NUM_MOTORS) {
        return;
    }
    uint32_t durationMs = static_cast<uint32_t>(cmd.durationDs) * 100;
    if (durationMs > config::MOTOR_TEST_MAX_MS) {
        durationMs = config::MOTOR_TEST_MAX_MS;
    }
    testMotor_    = cmd.motor;
    testThrottle_ = constrain(cmd.throttle, -100, 100) / 100.0f;
    testEndMs_    = nowMs + durationMs;
}

void RovController::update(const AttitudeFilter& attitude, bool imuOk, float dt, uint32_t nowMs,
                           float out[NUM_MOTORS]) {
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        out[i] = 0.0f;
    }

    if (!linkSeen_ || nowMs - lastStateMs_ > rov::LINK_TIMEOUT_MS) {
        failsafe_     = true;
        rearmBlocked_ = true;
        motion_       = rov::MotionCmd();
        disarm();
        testMotor_ = 0;
        return;
    }

    if (testMotor_ != 0) {
        if (static_cast<int32_t>(nowMs - testEndMs_) >= 0) {
            testMotor_ = 0;
        } else {
            out[testMotor_ - 1] = testThrottle_;
        }
        return;
    }

    if (!armed_) {
        return;
    }

    const float          gain = gainPercent_ / 100.0f;
    ThrusterMixer::Input in;
    in.forward = axis(motion_.surge) * gain;
    in.lateral = axis(motion_.sway) * gain;
    in.climb   = axis(motion_.heave) * gain;
    in.yaw     = axis(motion_.yaw) * gain;

    if (mode_ == rov::Mode::Stabilize && imuOk) {
        in.roll = rollPid_.update(-attitude.roll(), attitude.rollRate(), dt);

        if (fabsf(axis(motion_.yaw)) < config::YAW_HOLD_DEADBAND) {
            if (!holdingHeading_) {
                headingTarget_  = attitude.yaw();
                holdingHeading_ = true;
                yawPid_.reset();
            }
            const float error = wrapDegrees(headingTarget_ - attitude.yaw());
            in.yaw            = yawPid_.update(error, attitude.yawRate(), dt);
        } else {
            holdingHeading_ = false;
        }
    }

    ThrusterMixer::mix(in, out);

    if (config::OUT_OF_WATER) {
        bool spinning = false;
        for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
            if (fabsf(out[i]) > 0.02f) {
                spinning = true;
            }
        }
        if (spinning && !dryRunning_) {
            dryRunning_    = true;
            dryRunStartMs_ = nowMs;
        } else if (!spinning) {
            dryRunning_ = false;
        }
        if (dryRunning_ && nowMs - dryRunStartMs_ > config::DRY_MAX_RUN_MS) {
            dryTimeout_ = true;
        }
        if (dryTimeout_) {
            for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
                out[i] = 0.0f;
            }
        }
    }
}

uint8_t RovController::statusFlags() const {
    uint8_t flags = 0;
    // clang-format off
    if (armed_)               flags |= rov::status::ARMED;
    if (failsafe_)            flags |= rov::status::FAILSAFE;
    if (config::OUT_OF_WATER) flags |= rov::status::OUT_OF_WATER;
    if (dryTimeout_)          flags |= rov::status::DRY_TIMEOUT;
    if (testMotor_ != 0)      flags |= rov::status::MOTOR_TEST;
    // clang-format on
    return flags;
}

bool RovController::pilotNeutral() const {
    return fabsf(axis(motion_.surge)) < config::ARM_DEADBAND &&
           fabsf(axis(motion_.sway)) < config::ARM_DEADBAND &&
           fabsf(axis(motion_.heave)) < config::ARM_DEADBAND &&
           fabsf(axis(motion_.yaw)) < config::ARM_DEADBAND;
}

void RovController::disarm() {
    armed_      = false;
    dryRunning_ = false;
    dryTimeout_ = false;
}
