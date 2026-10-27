#pragma once

#include <mcp2515.h>
#include <RovProtocol.hpp>

// CAN driver for an MCP2515 SPI module, using the shared rov::CanFrame type.
class CanMcp2515 {
public:
    explicit CanMcp2515(uint8_t csPin) : mcp_(csPin) {}

    bool begin(uint32_t bitrate, CAN_CLOCK crystal);
    bool send(const rov::CanFrame& frame);
    bool receive(rov::CanFrame& frame);

    // True if the controller reports error-passive / bus-off conditions.
    bool hasError() { return mcp_.checkError(); }

private:
    MCP2515 mcp_;
};
