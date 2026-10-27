// ROV firmware: ESP32-S3 PowerFeather.
//
//  - 6 x Blue Robotics thrusters (Basic ESC), BlueROV2 vectored layout
//  - MPU6050 IMU on SDA 35 / SCL 36 for stabilize mode
//  - CAN bus to the surface station through a 3.3 V transceiver
//
// USB serial (115200) prints a status line for bench debugging.

#include <Arduino.h>
#include <PowerFeather.h>
#include <Wire.h>

#include <Mpu6050.hpp>
#include <RovProtocol.hpp>
#include <Thruster.hpp>

#include "CanTwai.hpp"
#include "RovController.hpp"
#include "config.hpp"

namespace {

constexpr uint8_t NUM_MOTORS = RovController::NUM_MOTORS;
const float       OUTPUT_LIMIT =
    config::OUT_OF_WATER ? config::OUTPUT_LIMIT_DRY : config::OUTPUT_LIMIT_WET;

Thruster thrusters[NUM_MOTORS] = {
    Thruster(config::PIN_THRUSTER[0], config::THRUSTER_REVERSED[0]),
    Thruster(config::PIN_THRUSTER[1], config::THRUSTER_REVERSED[1]),
    Thruster(config::PIN_THRUSTER[2], config::THRUSTER_REVERSED[2]),
    Thruster(config::PIN_THRUSTER[3], config::THRUSTER_REVERSED[3]),
    Thruster(config::PIN_THRUSTER[4], config::THRUSTER_REVERSED[4]),
    Thruster(config::PIN_THRUSTER[5], config::THRUSTER_REVERSED[5]),
};

Mpu6050        imu;
AttitudeFilter attitude;
ImuSample      imuSample;
CanTwai        can;
RovController  controller;

bool     imuOk               = false;
bool     canOk               = false;
bool     powerFeatherOk      = false;
uint16_t supplyMv            = 0;
float    outputs[NUM_MOTORS] = {};

uint32_t lastControlUs   = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastSupplyMs    = 0;
uint32_t lastServiceMs   = 0;
uint32_t lastDebugMs     = 0;

int16_t centi(float v) {
    return static_cast<int16_t>(constrain(v * 100.0f, -32768.0f, 32767.0f));
}

void handleCan(uint32_t nowMs) {
    rov::CanFrame frame;
    while (can.receive(frame)) {
        rov::MotionCmd    motion;
        rov::StateCmd     state;
        rov::MotorTestCmd test;
        if (rov::decode(frame, motion)) {
            controller.onMotion(motion);
        } else if (rov::decode(frame, state)) {
            controller.onState(state, nowMs);
        } else if (rov::decode(frame, test)) {
            controller.onMotorTest(test, nowMs);
        }
    }
}

void runControl() {
    const uint32_t nowUs = micros();
    if (nowUs - lastControlUs < config::CONTROL_PERIOD_US) {
        return;
    }
    const float dt = (nowUs - lastControlUs) * 1e-6f;
    lastControlUs  = nowUs;

    if (imu.read(imuSample)) {
        attitude.update(imuSample, dt);
        imuOk = true;
    } else {
        imuOk = false;
    }

    controller.update(attitude, imuOk, dt, millis(), outputs);
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        // Motor test outputs are absolute; pilot outputs scale with the limit.
        const bool testing = controller.statusFlags() & rov::status::MOTOR_TEST;
        thrusters[i].setThrottle(testing ? outputs[i] : outputs[i] * OUTPUT_LIMIT);
    }
}

void sendTelemetry() {
    rov::CanFrame frame;

    rov::AttitudeTlm att;
    att.roll    = centi(attitude.roll());
    att.pitch   = centi(attitude.pitch());
    att.yaw     = centi(attitude.yaw());
    att.yawRate = centi(attitude.yawRate());
    rov::encode(att, frame);
    can.send(frame);

    rov::StatusTlm status;
    status.flags    = controller.statusFlags() | (imuOk ? rov::status::IMU_OK : 0);
    status.mode     = controller.mode();
    status.gain     = controller.gainPercent();
    status.seq      = controller.lastSeq();
    status.supplyMv = supplyMv;
    status.imuTemp  = centi(imuSample.tempC);
    rov::encode(status, frame);
    can.send(frame);

    rov::ThrusterTlm thr;
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        thr.output[i] = static_cast<int8_t>(lroundf(thrusters[i].throttle() * 100.0f));
    }
    rov::encode(thr, frame);
    can.send(frame);
}

void readSupply() {
    float volts = 0.0f;
    if (powerFeatherOk && PowerFeather::Board.getSupplyVoltage(volts) == PowerFeather::Result::Ok) {
        supplyMv = static_cast<uint16_t>(volts * 1000.0f);
    }
}

// Solid = armed, slow blink = disarmed, fast blink = no link from station.
void updateLed(uint32_t nowMs) {
    const uint8_t flags = controller.statusFlags();
    bool          on;
    if (flags & rov::status::ARMED) {
        on = true;
    } else if (flags & rov::status::FAILSAFE) {
        on = (nowMs / 100) % 2;
    } else {
        on = (nowMs / 500) % 2;
    }
    digitalWrite(config::PIN_LED, on ? HIGH : LOW);
}

void printDebug() {
    const uint8_t flags = controller.statusFlags();
    Serial.printf("%s %-9s gain %3u%% | roll %6.1f pitch %6.1f yaw %6.1f | out",
                  (flags & rov::status::ARMED) ? "ARMED   " : "DISARMED",
                  controller.mode() == rov::Mode::Stabilize ? "STABILIZE" : "MANUAL",
                  controller.gainPercent(), attitude.roll(), attitude.pitch(), attitude.yaw());
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        Serial.printf(" %4d", thrusters[i].pulse());
    }
    Serial.printf(" | %.2f V%s%s%s%s\n", supplyMv / 1000.0f,
                  (flags & rov::status::FAILSAFE) ? " NO-LINK" : "", imuOk ? "" : " NO-IMU",
                  can.busOff() ? " CAN-BUS-OFF" : "",
                  (flags & rov::status::DRY_TIMEOUT) ? " DRY-TIMEOUT" : "");
}

}  // namespace

void setup() {
    Serial.begin(115200);
    pinMode(config::PIN_LED, OUTPUT);

    // Send the stop signal first so the ESCs start arming immediately.
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        thrusters[i].begin();
        thrusters[i].setMaxThrottle(OUTPUT_LIMIT);
    }
    const uint32_t escArmStart = millis();

    powerFeatherOk = PowerFeather::Board.init() == PowerFeather::Result::Ok;

    Wire.begin(config::PIN_IMU_SDA, config::PIN_IMU_SCL, config::IMU_I2C_HZ);
    imuOk = imu.begin(Wire);
    if (imuOk) {
        imu.setAxisSigns(config::IMU_AXIS_SIGN[0], config::IMU_AXIS_SIGN[1],
                         config::IMU_AXIS_SIGN[2]);
        imuOk = imu.calibrateGyro();  // keep the ROV still during boot
    }

    canOk = can.begin(config::PIN_CAN_TX, config::PIN_CAN_RX, rov::CAN_BITRATE);

    while (millis() - escArmStart < Thruster::ARM_TIME_MS) {
        digitalWrite(config::PIN_LED, (millis() / 50) % 2);
    }

    Serial.println();
    Serial.println("=== ROV ready ===");
    Serial.printf("IMU %s (WHO_AM_I 0x%02X), CAN %s, PowerFeather %s, %s, output limit %.0f%%\n",
                  imuOk ? "ok" : "FAILED", imu.whoAmI(), canOk ? "ok" : "FAILED",
                  powerFeatherOk ? "ok" : "FAILED",
                  config::OUT_OF_WATER ? "OUT OF WATER" : "in water", OUTPUT_LIMIT * 100.0f);

    lastControlUs = micros();
}

void loop() {
    const uint32_t nowMs = millis();

    handleCan(nowMs);
    runControl();

    if (nowMs - lastTelemetryMs >= config::TELEMETRY_PERIOD_MS) {
        lastTelemetryMs = nowMs;
        sendTelemetry();
    }
    if (nowMs - lastServiceMs >= 100) {
        lastServiceMs = nowMs;
        can.service();
        updateLed(nowMs);
    }
    if (nowMs - lastSupplyMs >= config::SUPPLY_PERIOD_MS) {
        lastSupplyMs = nowMs;
        readSupply();
    }
    if (nowMs - lastDebugMs >= config::DEBUG_PERIOD_MS) {
        lastDebugMs = nowMs;
        printDebug();
    }
}
