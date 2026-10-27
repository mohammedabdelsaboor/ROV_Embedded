#include "CanMcp2515.hpp"

bool CanMcp2515::begin(uint32_t bitrate, CAN_CLOCK crystal) {
    CAN_SPEED speed;
    switch (bitrate) {
    case 125000: speed = CAN_125KBPS; break;
    case 250000: speed = CAN_250KBPS; break;
    case 500000: speed = CAN_500KBPS; break;
    case 1000000: speed = CAN_1000KBPS; break;
    default: return false;
    }
    return mcp_.reset() == MCP2515::ERROR_OK &&
           mcp_.setBitrate(speed, crystal) == MCP2515::ERROR_OK &&
           mcp_.setNormalMode() == MCP2515::ERROR_OK;
}

bool CanMcp2515::send(const rov::CanFrame& frame) {
    can_frame out;
    out.can_id  = frame.id;
    out.can_dlc = frame.len;
    for (uint8_t i = 0; i < frame.len; ++i) {
        out.data[i] = frame.data[i];
    }
    return mcp_.sendMessage(&out) == MCP2515::ERROR_OK;
}

bool CanMcp2515::receive(rov::CanFrame& frame) {
    can_frame in;
    while (mcp_.readMessage(&in) == MCP2515::ERROR_OK) {
        if ((in.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG)) || in.can_dlc > 8) {
            continue;  // not part of our protocol
        }
        frame.id  = static_cast<uint16_t>(in.can_id & CAN_SFF_MASK);
        frame.len = in.can_dlc;
        for (uint8_t i = 0; i < in.can_dlc; ++i) {
            frame.data[i] = in.data[i];
        }
        return true;
    }
    return false;
}
