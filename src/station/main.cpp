// Surface station firmware: Arduino Uno + MCP2515 CAN module.
//
// Reads the pilot controls (joysticks + buttons, or serial console commands),
// sends them to the ROV over CAN at 20 Hz and prints the ROV telemetry.
// Serial console: 115200 baud, newline line ending, type "h" for help.

#include <Arduino.h>
#include <RovProtocol.hpp>

#include "CanMcp2515.hpp"
#include "PilotInput.hpp"
#include "config.hpp"

namespace {

CanMcp2515 can(config::PIN_CAN_CS);
PilotInput pilot;

rov::StateCmd  request;       // what the station asks the ROV to do
rov::MotionCmd serialMotion;  // motion from the console when no joysticks

rov::AttitudeTlm attitude;
rov::StatusTlm   status;
rov::ThrusterTlm thrusters;

bool     canOk       = false;
bool     linkUp      = false;
bool     printTlm    = true;
uint32_t lastTlmMs   = 0;
uint32_t lastCmdMs   = 0;
uint32_t lastPrintMs = 0;

char   lineBuf[48];
size_t lineLen = 0;

rov::MotionCmd currentMotion() {
    return config::USE_JOYSTICKS ? pilot.motion() : serialMotion;
}

bool neutral(const rov::MotionCmd& m) {
    const int16_t band = rov::AXIS_MAX / 20;  // 5 %
    return abs(m.surge) < band && abs(m.sway) < band && abs(m.heave) < band && abs(m.yaw) < band;
}

void requestArm() {
    if (!linkUp) {
        Serial.println(F("Can't arm: no link to the ROV."));
    } else if (!neutral(currentMotion())) {
        Serial.println(F("Can't arm: center all sticks first."));
    } else {
        request.armed = true;
        Serial.println(F("Arming..."));
    }
}

void requestDisarm() {
    request.armed = false;
    Serial.println(F("Disarmed."));
}

void setMode(rov::Mode mode) {
    request.mode = mode;
    Serial.println(mode == rov::Mode::Stabilize ? F("Mode: STABILIZE") : F("Mode: MANUAL"));
}

void setGain(int gain) {
    request.gain = constrain(gain, config::GAIN_MIN, config::GAIN_MAX);
    Serial.print(F("Gain: "));
    Serial.print(request.gain);
    Serial.println('%');
}

int16_t percentToAxis(const char* tok) {
    const long pct = tok ? constrain(atol(tok), -100L, 100L) : 0;
    return static_cast<int16_t>(pct * rov::AXIS_MAX / 100);
}

void sendMotorTest(const char* motorTok, const char* throttleTok, const char* secondsTok) {
    if (request.armed) {
        Serial.println(F("Disarm before a motor test."));
        return;
    }
    const int motor = motorTok ? atoi(motorTok) : 0;
    if (motor < 1 || motor > rov::NUM_THRUSTERS || throttleTok == nullptr) {
        Serial.println(F("Usage: test <motor 1-6> <throttle -100..100> [seconds 1-3]"));
        return;
    }
    rov::MotorTestCmd test;
    test.motor      = static_cast<uint8_t>(motor);
    test.throttle   = static_cast<int8_t>(constrain(atoi(throttleTok), -100, 100));
    const int secs  = secondsTok ? constrain(atoi(secondsTok), 1, config::TEST_MAX_SECONDS) : 2;
    test.durationDs = static_cast<uint8_t>(secs * 10);

    rov::CanFrame frame;
    rov::encode(test, frame);
    Serial.println(can.send(frame) ? F("Motor test sent.") : F("CAN send failed."));
}

void printHelp() {
    Serial.println(F("\nCommands:"));
    Serial.println(F("  arm | disarm             arm / disarm the ROV"));
    Serial.println(F("  manual | stab            flight mode"));
    Serial.println(F("  gain <10-100>            pilot gain in percent"));
    if (!config::USE_JOYSTICKS) {
        Serial.println(F("  m <surge> <sway> <heave> <yaw>   motion in percent, e.g. m 30 0 0 0"));
    }
    Serial.println(F("  stop                     zero motion and disarm"));
    Serial.println(F("  test <motor> <pct> [s]   spin one thruster (disarmed only)"));
    Serial.println(F("  tlm                      toggle telemetry printing"));
    Serial.println(F("  h                        this help"));
}

void handleCommand(char* line) {
    char* cmd = strtok(line, " ");
    if (cmd == nullptr) {
        return;
    }
    char* a1 = strtok(nullptr, " ");
    char* a2 = strtok(nullptr, " ");
    char* a3 = strtok(nullptr, " ");
    char* a4 = strtok(nullptr, " ");

    if (!strcmp(cmd, "arm")) {
        requestArm();
    } else if (!strcmp(cmd, "disarm")) {
        requestDisarm();
    } else if (!strcmp(cmd, "manual")) {
        setMode(rov::Mode::Manual);
    } else if (!strcmp(cmd, "stab")) {
        setMode(rov::Mode::Stabilize);
    } else if (!strcmp(cmd, "gain") && a1) {
        setGain(atoi(a1));
    } else if (!strcmp(cmd, "m")) {
        if (config::USE_JOYSTICKS) {
            Serial.println(F(
                "Joysticks are enabled; build with STATION_USE_JOYSTICKS=0 to drive from serial."));
            return;
        }
        serialMotion.surge = percentToAxis(a1);
        serialMotion.sway  = percentToAxis(a2);
        serialMotion.heave = percentToAxis(a3);
        serialMotion.yaw   = percentToAxis(a4);
    } else if (!strcmp(cmd, "stop") || !strcmp(cmd, "s")) {
        serialMotion = rov::MotionCmd();
        requestDisarm();
    } else if (!strcmp(cmd, "test")) {
        sendMotorTest(a1, a2, a3);
    } else if (!strcmp(cmd, "tlm")) {
        printTlm = !printTlm;
    } else if (!strcmp(cmd, "h") || !strcmp(cmd, "help")) {
        printHelp();
    } else {
        Serial.println(F("Unknown command, type 'h' for help."));
    }
}

void readConsole() {
    while (Serial.available()) {
        const char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (lineLen > 0) {
                lineBuf[lineLen] = '\0';
                lineLen          = 0;
                handleCommand(lineBuf);
            }
        } else if (lineLen < sizeof(lineBuf) - 1) {
            lineBuf[lineLen++] = c;
        }
    }
}

void readButtons(uint32_t nowMs) {
    pilot.update(nowMs);
    if (pilot.armPressed()) {
        if (request.armed) {
            requestDisarm();
        } else {
            requestArm();
        }
    }
    if (pilot.modePressed()) {
        setMode(request.mode == rov::Mode::Manual ? rov::Mode::Stabilize : rov::Mode::Manual);
    }
    if (pilot.gainUpPressed()) {
        setGain(request.gain + config::GAIN_STEP);
    }
    if (pilot.gainDownPressed()) {
        setGain(request.gain - config::GAIN_STEP);
    }
}

void readTelemetry(uint32_t nowMs) {
    rov::CanFrame frame;
    while (can.receive(frame)) {
        if (rov::decode(frame, status)) {
            lastTlmMs = nowMs;
        } else if (rov::decode(frame, attitude) || rov::decode(frame, thrusters)) {
            // stored for printing
        }
    }

    const bool up = lastTlmMs != 0 && nowMs - lastTlmMs < config::TLM_TIMEOUT_MS;
    if (linkUp && !up) {
        Serial.println(F("LINK LOST"));
        if (request.armed) {
            // The ROV disarms itself on link loss; match it so it can't
            // re-arm unexpectedly when the link comes back.
            requestDisarm();
        }
    } else if (!linkUp && up) {
        Serial.println(F("Link up."));
    }
    linkUp = up;
}

void sendCommands() {
    rov::CanFrame frame;
    rov::encode(currentMotion(), frame);
    can.send(frame);

    ++request.seq;
    rov::encode(request, frame);
    can.send(frame);
}

void printAngle(int16_t centideg) {
    Serial.print(centideg / 100.0f, 1);
}

void printTelemetry() {
    if (!linkUp) {
        Serial.println(canOk ? F("ROV: no link")
                             : F("CAN module not responding - check wiring / crystal setting"));
        return;
    }
    Serial.print(F("ROV "));
    Serial.print((status.flags & rov::status::ARMED) ? F("ARMED    ") : F("disarmed "));
    Serial.print(status.mode == rov::Mode::Stabilize ? F("STAB   ") : F("MANUAL "));
    Serial.print(status.gain);
    Serial.print(F("% | roll "));
    printAngle(attitude.roll);
    Serial.print(F(" pitch "));
    printAngle(attitude.pitch);
    Serial.print(F(" yaw "));
    printAngle(attitude.yaw);
    Serial.print(F(" | T"));
    for (uint8_t i = 0; i < rov::NUM_THRUSTERS; ++i) {
        Serial.print(' ');
        Serial.print(thrusters.output[i]);
    }
    Serial.print(F(" | "));
    Serial.print(status.supplyMv / 1000.0f, 2);
    Serial.print(F(" V "));
    Serial.print(status.imuTemp / 100.0f, 1);
    Serial.print(F(" C"));
    // clang-format off
    if (!(status.flags & rov::status::IMU_OK))      Serial.print(F(" NO-IMU"));
    if (status.flags & rov::status::OUT_OF_WATER)   Serial.print(F(" DRY-LIMIT"));
    if (status.flags & rov::status::DRY_TIMEOUT)    Serial.print(F(" DRY-TIMEOUT(disarm)"));
    if (status.flags & rov::status::MOTOR_TEST)     Serial.print(F(" MOTOR-TEST"));
    if (request.armed && !(status.flags & rov::status::ARMED)) Serial.print(F(" (arm refused: disarm, center sticks, arm)"));
    // clang-format on
    Serial.println();
}

void updateLeds() {
    digitalWrite(config::PIN_LED_ARMED,
                 (linkUp && (status.flags & rov::status::ARMED)) ? HIGH : LOW);
    digitalWrite(config::PIN_LED_LINK, linkUp ? HIGH : LOW);
}

}  // namespace

void setup() {
    Serial.begin(115200);
    pinMode(config::PIN_LED_ARMED, OUTPUT);
    pinMode(config::PIN_LED_LINK, OUTPUT);

    Serial.println(F("\n=== ROV surface station ==="));
    pilot.begin();  // calibrates joystick centers: don't touch the sticks
    request.gain = config::GAIN_DEFAULT;

    canOk = can.begin(rov::CAN_BITRATE, STATION_MCP_CLOCK);
    Serial.println(canOk ? F("CAN ok, 250 kbit/s") : F("CAN init FAILED"));
    Serial.println(config::USE_JOYSTICKS ? F("Input: joysticks + buttons")
                                         : F("Input: serial console"));
    printHelp();
}

void loop() {
    const uint32_t nowMs = millis();

    readConsole();
    readButtons(nowMs);
    readTelemetry(nowMs);

    if (nowMs - lastCmdMs >= config::CMD_PERIOD_MS) {
        lastCmdMs = nowMs;
        sendCommands();
        updateLeds();
    }
    if (printTlm && nowMs - lastPrintMs >= config::PRINT_PERIOD_MS) {
        lastPrintMs = nowMs;
        printTelemetry();
    }
}
