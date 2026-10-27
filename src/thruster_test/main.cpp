// Blue Robotics thruster bench test for Arduino Mega 2560.
//
// Wiring (per thruster / Basic ESC):
//   ESC signal (white)  -> Mega PWM pin listed in THRUSTER_PINS
//   ESC ground (black)  -> Mega GND  (common ground is required)
//   ESC power           -> 12-16 V battery / PSU, NOT the Arduino
//
// Open the serial monitor at 115200 baud (newline line ending) and type 'h'.

#include <Arduino.h>
#include <Thruster.hpp>

// ---- Configuration --------------------------------------------------------
const uint8_t THRUSTER_PINS[] = {2, 3, 4, 5, 6, 7, 8, 9};
const size_t  NUM_THRUSTERS   = sizeof(THRUSTER_PINS) / sizeof(THRUSTER_PINS[0]);

// Set to false only when the thrusters are submerged.
// T200s are water-cooled and water-lubricated: out of water, keep runs short
// (a few seconds) and slow.
const bool OUT_OF_WATER = true;

// Hard throttle cap while OUT_OF_WATER. The 'm' command cannot go above it.
const float DRY_MAX_THROTTLE = 0.15f;
// Out of water, any thruster running longer than this is stopped automatically.
const uint32_t DRY_MAX_RUN_MS = 5000;

// Forward -> reverse dry test ('d' command).
const float    DRY_TEST_THROTTLE = 0.10f;  // ESC deadband is ~1475-1525 us
const uint32_t DRY_STEP_MS       = 2000;   // time in each direction
const uint32_t DRY_REST_MS       = 1000;   // stop between directions / thrusters

// Thruster id to run the dry test on automatically after arming, without any
// serial input (-1 = all, -2 = off). Pin 3 is id 1.
const int AUTO_TEST_ID = 1;

// Stop all thrusters if no command is received for this long (0 = disabled).
const uint32_t COMMAND_TIMEOUT_MS = 10000;
// ---------------------------------------------------------------------------

Thruster thrusters[NUM_THRUSTERS] = {
    Thruster(THRUSTER_PINS[0]), Thruster(THRUSTER_PINS[1]), Thruster(THRUSTER_PINS[2]),
    Thruster(THRUSTER_PINS[3]), Thruster(THRUSTER_PINS[4]), Thruster(THRUSTER_PINS[5]),
    Thruster(THRUSTER_PINS[6]), Thruster(THRUSTER_PINS[7]),
};

uint32_t lastCommandMs = 0;
uint32_t runStartMs    = 0;
bool     running       = false;
bool     timedOut      = false;
char     lineBuf[48];
size_t   lineLen = 0;

void printHelp() {
    Serial.println(F("\nCommands (id = thruster index, or 'a' for all):"));
    Serial.println(F("  d <id>              dry test: forward -> reverse at 10% (one at a time)"));
    Serial.println(F("  t <id> <-100..100>  set throttle in percent (clamped to limit)"));
    Serial.println(F("  p <id> <1100..1900> set raw pulse width in us (clamped to limit)"));
    Serial.println(F("  r <id>              ramp test: 0 -> +limit -> -limit -> 0"));
    Serial.println(F("  m <0..100>          set throttle limit in percent"));
    Serial.println(F("  s                   stop all thrusters"));
    Serial.println(F("  i                   show status"));
    Serial.println(F("  h                   show this help"));
}

void printStatus() {
    Serial.print(F("  limit "));
    Serial.print(thrusters[0].maxThrottle() * 100.0f, 0);
    Serial.println(OUT_OF_WATER ? F("%  (OUT OF WATER mode)") : F("%"));
    for (size_t i = 0; i < NUM_THRUSTERS; ++i) {
        Serial.print(F("  ["));
        Serial.print(i);
        Serial.print(F("] pin "));
        Serial.print(thrusters[i].pin());
        Serial.print(F("  throttle "));
        Serial.print(thrusters[i].throttle() * 100.0f, 0);
        Serial.print(F("%  pulse "));
        Serial.print(thrusters[i].pulse());
        Serial.println(F(" us"));
    }
}

void stopAll() {
    for (size_t i = 0; i < NUM_THRUSTERS; ++i) {
        thrusters[i].stop();
    }
}

bool anyRunning() {
    for (size_t i = 0; i < NUM_THRUSTERS; ++i) {
        if (thrusters[i].pulse() != Thruster::PULSE_STOP) {
            return true;
        }
    }
    return false;
}

// Parses a thruster id token. Returns -1 for "all", -2 on error.
int parseId(const char* tok) {
    if (tok == nullptr) {
        return -2;
    }
    if (tok[0] == 'a' || tok[0] == 'A') {
        return -1;
    }
    int id = atoi(tok);
    if (id < 0 || static_cast<size_t>(id) >= NUM_THRUSTERS) {
        return -2;
    }
    return id;
}

// Delay that aborts early if a key arrives on serial. Line endings left over
// from the command that started the test (\r\n) are ignored.
bool interruptibleDelay(uint32_t ms) {
    uint32_t start = millis();
    while (millis() - start < ms) {
        if (Serial.available()) {
            int c = Serial.peek();
            if (c == '\r' || c == '\n') {
                Serial.read();
                continue;
            }
            return false;
        }
    }
    return true;
}

void abortTest() {
    stopAll();
    while (Serial.available()) {
        Serial.read();
    }
    Serial.println(F("Aborted, all stopped."));
}

// Runs one thruster at a time: forward, stop, reverse, stop.
void dryTest(int id) {
    Serial.println(F("Dry test forward -> reverse (send any key to abort)"));
    const float directions[] = {DRY_TEST_THROTTLE, -DRY_TEST_THROTTLE};
    for (size_t i = 0; i < NUM_THRUSTERS; ++i) {
        if (id != -1 && static_cast<size_t>(id) != i) {
            continue;
        }
        Serial.print(F("  ["));
        Serial.print(i);
        Serial.print(F("] "));
        for (float throttle : directions) {
            thrusters[i].setThrottle(throttle);
            Serial.print(throttle > 0 ? F("FORWARD ") : F("REVERSE "));
            Serial.print(thrusters[i].throttle() * 100.0f, 0);
            Serial.print(F("%  "));
            if (!interruptibleDelay(DRY_STEP_MS)) {
                abortTest();
                return;
            }

            thrusters[i].stop();
            Serial.print(F("STOP  "));
            if (!interruptibleDelay(DRY_REST_MS)) {
                abortTest();
                return;
            }
        }
        Serial.println();
    }
    Serial.println(F("Dry test done."));
}

void rampTest(int id) {
    Serial.println(F("Ramp test (send any key to abort)"));
    const int steps[] = {0, 25, 50, 75, 100, 75, 50, 25, 0, -25, -50, -75, -100, -75, -50, -25, 0};
    // Keep each direction under DRY_MAX_RUN_MS when out of water.
    const uint32_t stepMs = OUT_OF_WATER ? 500 : 1000;
    for (int pct : steps) {
        for (size_t i = 0; i < NUM_THRUSTERS; ++i) {
            if (id == -1 || static_cast<size_t>(id) == i) {
                thrusters[i].setThrottle(pct / 100.0f * thrusters[i].maxThrottle());
            }
        }
        Serial.print(F("  "));
        Serial.print(pct);
        Serial.println(F("% of limit"));
        if (!interruptibleDelay(stepMs)) {
            abortTest();
            return;
        }
    }
    stopAll();
    Serial.println(F("Ramp done."));
}

void handleCommand(char* line) {
    char* cmd = strtok(line, " ");
    if (cmd == nullptr) {
        return;
    }
    char* arg1 = strtok(nullptr, " ");
    char* arg2 = strtok(nullptr, " ");

    switch (cmd[0]) {
    case 't':
    case 'p': {
        int id = parseId(arg1);
        if (id == -2 || arg2 == nullptr) {
            Serial.println(F("Usage: t <id|a> <percent>  or  p <id|a> <us>"));
            return;
        }
        long value = atol(arg2);
        for (size_t i = 0; i < NUM_THRUSTERS; ++i) {
            if (id == -1 || static_cast<size_t>(id) == i) {
                if (cmd[0] == 't') {
                    thrusters[i].setThrottle(value / 100.0f);
                } else {
                    thrusters[i].setPulse(static_cast<uint16_t>(constrain(value, 0L, 65535L)));
                }
            }
        }
        printStatus();
        break;
    }
    case 'd':
    case 'r': {
        int id = parseId(arg1);
        if (id == -2) {
            Serial.println(F("Usage: d <id|a>  or  r <id|a>"));
            return;
        }
        if (cmd[0] == 'd') {
            dryTest(id);
        } else {
            rampTest(id);
        }
        break;
    }
    case 'm': {
        if (arg1 == nullptr) {
            Serial.println(F("Usage: m <0..100>"));
            return;
        }
        float limit = atoi(arg1) / 100.0f;
        if (OUT_OF_WATER && limit > DRY_MAX_THROTTLE) {
            limit = DRY_MAX_THROTTLE;
            Serial.println(
                F("Out of water: limit capped (set OUT_OF_WATER = false when submerged)."));
        }
        for (size_t i = 0; i < NUM_THRUSTERS; ++i) {
            thrusters[i].setMaxThrottle(limit);
        }
        printStatus();
        break;
    }
    case 's':
        stopAll();
        Serial.println(F("All stopped."));
        break;
    case 'i': printStatus(); break;
    case 'h':
    case '?': printHelp(); break;
    default: Serial.println(F("Unknown command, type 'h' for help.")); break;
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("\n=== Blue Robotics thruster test ==="));
    if (OUT_OF_WATER) {
        Serial.println(F("OUT OF WATER mode: low throttle, short runs only."));
    }
    Serial.print(F("Arming "));
    Serial.print(NUM_THRUSTERS);
    Serial.println(F(" ESC(s), sending stop signal..."));

    Thruster::armAll(thrusters, NUM_THRUSTERS);
    const float limit = OUT_OF_WATER ? DRY_MAX_THROTTLE : 1.0f;
    for (size_t i = 0; i < NUM_THRUSTERS; ++i) {
        thrusters[i].setMaxThrottle(limit);
    }

    Serial.println(F("ESCs armed."));
    if (AUTO_TEST_ID != -2) {
        dryTest(AUTO_TEST_ID);
    }
    printHelp();
    printStatus();
    lastCommandMs = millis();
}

void loop() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (lineLen > 0) {
                lineBuf[lineLen] = '\0';
                lineLen          = 0;
                lastCommandMs    = millis();
                timedOut         = false;
                handleCommand(lineBuf);
            }
        } else if (lineLen < sizeof(lineBuf) - 1) {
            lineBuf[lineLen++] = c;
        }
    }

    // Out of water, never let a thruster spin continuously for too long.
    if (anyRunning()) {
        if (!running) {
            running    = true;
            runStartMs = millis();
        } else if (OUT_OF_WATER && millis() - runStartMs > DRY_MAX_RUN_MS) {
            stopAll();
            running = false;
            Serial.println(F("Out of water: max run time reached, all thrusters stopped."));
        }
    } else {
        running = false;
    }

    if (COMMAND_TIMEOUT_MS > 0 && !timedOut && millis() - lastCommandMs > COMMAND_TIMEOUT_MS) {
        stopAll();
        timedOut = true;
        Serial.println(F("Command timeout: all thrusters stopped."));
    }
}
