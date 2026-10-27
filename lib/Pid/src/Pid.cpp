#include "Pid.hpp"

namespace {
float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
}  // namespace

Pid::Pid(float kp, float ki, float kd, float integralLimit, float outputLimit)
    : kp_(kp), ki_(ki), kd_(kd), integralLimit_(integralLimit), outputLimit_(outputLimit) {}

void Pid::setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

float Pid::update(float error, float rate, float dt) {
    integral_ = clampf(integral_ + ki_ * error * dt, -integralLimit_, integralLimit_);
    return clampf(kp_ * error + integral_ - kd_ * rate, -outputLimit_, outputLimit_);
}
