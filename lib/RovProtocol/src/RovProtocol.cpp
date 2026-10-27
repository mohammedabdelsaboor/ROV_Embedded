#include "RovProtocol.hpp"

namespace rov {

namespace {

void putI16(uint8_t* p, int16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((static_cast<uint16_t>(v) >> 8) & 0xFF);
}

void putU16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>(v >> 8);
}

int16_t getI16(const uint8_t* p) {
    return static_cast<int16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

uint16_t getU16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

void header(CanFrame& frame, uint16_t id, uint8_t len) {
    frame.id  = id;
    frame.len = len;
    for (uint8_t i = 0; i < sizeof(frame.data); ++i) {
        frame.data[i] = 0;
    }
}

bool matches(const CanFrame& frame, uint16_t id, uint8_t len) {
    return frame.id == id && frame.len >= len;
}

}  // namespace

void encode(const MotionCmd& msg, CanFrame& frame) {
    header(frame, ID_CMD_MOTION, 8);
    putI16(&frame.data[0], msg.surge);
    putI16(&frame.data[2], msg.sway);
    putI16(&frame.data[4], msg.heave);
    putI16(&frame.data[6], msg.yaw);
}

bool decode(const CanFrame& frame, MotionCmd& msg) {
    if (!matches(frame, ID_CMD_MOTION, 8)) {
        return false;
    }
    msg.surge = getI16(&frame.data[0]);
    msg.sway  = getI16(&frame.data[2]);
    msg.heave = getI16(&frame.data[4]);
    msg.yaw   = getI16(&frame.data[6]);
    return true;
}

void encode(const StateCmd& msg, CanFrame& frame) {
    header(frame, ID_CMD_STATE, 4);
    frame.data[0] = msg.armed ? 1 : 0;
    frame.data[1] = static_cast<uint8_t>(msg.mode);
    frame.data[2] = msg.gain;
    frame.data[3] = msg.seq;
}

bool decode(const CanFrame& frame, StateCmd& msg) {
    if (!matches(frame, ID_CMD_STATE, 4)) {
        return false;
    }
    msg.armed = frame.data[0] == 1;
    msg.mode =
        frame.data[1] == static_cast<uint8_t>(Mode::Stabilize) ? Mode::Stabilize : Mode::Manual;
    msg.gain = frame.data[2] > 100 ? 100 : frame.data[2];
    msg.seq  = frame.data[3];
    return true;
}

void encode(const MotorTestCmd& msg, CanFrame& frame) {
    header(frame, ID_CMD_MOTOR_TEST, 3);
    frame.data[0] = msg.motor;
    frame.data[1] = static_cast<uint8_t>(msg.throttle);
    frame.data[2] = msg.durationDs;
}

bool decode(const CanFrame& frame, MotorTestCmd& msg) {
    if (!matches(frame, ID_CMD_MOTOR_TEST, 3)) {
        return false;
    }
    msg.motor      = frame.data[0];
    msg.throttle   = static_cast<int8_t>(frame.data[1]);
    msg.durationDs = frame.data[2];
    return true;
}

void encode(const AttitudeTlm& msg, CanFrame& frame) {
    header(frame, ID_TLM_ATTITUDE, 8);
    putI16(&frame.data[0], msg.roll);
    putI16(&frame.data[2], msg.pitch);
    putI16(&frame.data[4], msg.yaw);
    putI16(&frame.data[6], msg.yawRate);
}

bool decode(const CanFrame& frame, AttitudeTlm& msg) {
    if (!matches(frame, ID_TLM_ATTITUDE, 8)) {
        return false;
    }
    msg.roll    = getI16(&frame.data[0]);
    msg.pitch   = getI16(&frame.data[2]);
    msg.yaw     = getI16(&frame.data[4]);
    msg.yawRate = getI16(&frame.data[6]);
    return true;
}

void encode(const StatusTlm& msg, CanFrame& frame) {
    header(frame, ID_TLM_STATUS, 8);
    frame.data[0] = msg.flags;
    frame.data[1] = static_cast<uint8_t>(msg.mode);
    frame.data[2] = msg.gain;
    frame.data[3] = msg.seq;
    putU16(&frame.data[4], msg.supplyMv);
    putI16(&frame.data[6], msg.imuTemp);
}

bool decode(const CanFrame& frame, StatusTlm& msg) {
    if (!matches(frame, ID_TLM_STATUS, 8)) {
        return false;
    }
    msg.flags = frame.data[0];
    msg.mode =
        frame.data[1] == static_cast<uint8_t>(Mode::Stabilize) ? Mode::Stabilize : Mode::Manual;
    msg.gain     = frame.data[2];
    msg.seq      = frame.data[3];
    msg.supplyMv = getU16(&frame.data[4]);
    msg.imuTemp  = getI16(&frame.data[6]);
    return true;
}

void encode(const ThrusterTlm& msg, CanFrame& frame) {
    header(frame, ID_TLM_THRUSTERS, NUM_THRUSTERS);
    for (uint8_t i = 0; i < NUM_THRUSTERS; ++i) {
        frame.data[i] = static_cast<uint8_t>(msg.output[i]);
    }
}

bool decode(const CanFrame& frame, ThrusterTlm& msg) {
    if (!matches(frame, ID_TLM_THRUSTERS, NUM_THRUSTERS)) {
        return false;
    }
    for (uint8_t i = 0; i < NUM_THRUSTERS; ++i) {
        msg.output[i] = static_cast<int8_t>(frame.data[i]);
    }
    return true;
}

}  // namespace rov
