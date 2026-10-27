#pragma once

// PID controller with integral clamp and derivative-on-measurement.
// Passing the measured rate (e.g. gyro) as the derivative term avoids the
// kick a derivative-on-error controller gives when the setpoint jumps.
class Pid {
public:
    Pid(float kp, float ki, float kd, float integralLimit, float outputLimit);

    // error: setpoint - measurement
    // rate:  derivative of the measurement (same units per second)
    // dt:    seconds since the last update
    float update(float error, float rate, float dt);

    void reset() { integral_ = 0.0f; }

    void setGains(float kp, float ki, float kd);

private:
    float kp_, ki_, kd_;
    float integralLimit_;
    float outputLimit_;
    float integral_ = 0.0f;
};
