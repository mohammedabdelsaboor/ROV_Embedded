#include "CanTwai.hpp"

#include <driver/twai.h>
#include <string.h>

bool CanTwai::begin(int txPin, int rxPin, uint32_t bitrate) {
    twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
        static_cast<gpio_num_t>(txPin), static_cast<gpio_num_t>(rxPin), TWAI_MODE_NORMAL);
    general.tx_queue_len = 16;
    general.rx_queue_len = 32;

    twai_timing_config_t timing;
    switch (bitrate) {
    case 125000: timing = TWAI_TIMING_CONFIG_125KBITS(); break;
    case 250000: timing = TWAI_TIMING_CONFIG_250KBITS(); break;
    case 500000: timing = TWAI_TIMING_CONFIG_500KBITS(); break;
    case 1000000: timing = TWAI_TIMING_CONFIG_1MBITS(); break;
    default: return false;
    }

    const twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&general, &timing, &filter) != ESP_OK) {
        return false;
    }
    started_ = twai_start() == ESP_OK;
    return started_;
}

bool CanTwai::send(const rov::CanFrame& frame) {
    if (!started_ || busOff_) {
        return false;
    }
    twai_message_t msg   = {};
    msg.identifier       = frame.id;
    msg.data_length_code = frame.len;
    memcpy(msg.data, frame.data, frame.len);
    return twai_transmit(&msg, 0) == ESP_OK;
}

bool CanTwai::receive(rov::CanFrame& frame) {
    if (!started_) {
        return false;
    }
    twai_message_t msg;
    while (twai_receive(&msg, 0) == ESP_OK) {
        if (msg.extd || msg.rtr || msg.data_length_code > 8) {
            continue;  // not part of our protocol
        }
        frame.id  = static_cast<uint16_t>(msg.identifier);
        frame.len = msg.data_length_code;
        memcpy(frame.data, msg.data, msg.data_length_code);
        return true;
    }
    return false;
}

void CanTwai::service() {
    if (!started_) {
        return;
    }
    twai_status_info_t status;
    if (twai_get_status_info(&status) != ESP_OK) {
        return;
    }
    switch (status.state) {
    case TWAI_STATE_BUS_OFF:
        if (!busOff_) {
            busOff_ = true;
            twai_initiate_recovery();
        }
        break;
    case TWAI_STATE_RECOVERING: break;
    case TWAI_STATE_STOPPED:
        // Recovery finished: the controller must be restarted explicitly.
        if (twai_start() == ESP_OK) {
            busOff_ = false;
        }
        break;
    default: busOff_ = false; break;
    }
}
