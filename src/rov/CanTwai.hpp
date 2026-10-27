#pragma once

#include <RovProtocol.hpp>

// CAN driver for the ESP32 built-in TWAI controller. Needs an external 3.3 V
// transceiver (e.g. SN65HVD230) between the TX/RX pins and the bus.
class CanTwai {
public:
    bool begin(int txPin, int rxPin, uint32_t bitrate);

    // Non-blocking. Returns false if the transmit queue is full or the bus is
    // down (e.g. no other node to acknowledge).
    bool send(const rov::CanFrame& frame);

    // Non-blocking. Returns true and fills frame if one was waiting.
    bool receive(rov::CanFrame& frame);

    // Call periodically: restarts the controller after a bus-off condition.
    void service();

    bool busOff() const { return busOff_; }

private:
    bool started_ = false;
    bool busOff_  = false;
};
