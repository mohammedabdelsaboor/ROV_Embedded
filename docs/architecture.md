# Software architecture

## Overview

```
 STATION (Uno)                                     ROV (ESP32-S3)
 ┌───────────────────────────┐                     ┌──────────────────────────────────────────┐
 │ PilotInput                │                     │ CanTwai ──► handleCan()                  │
 │  joysticks, buttons       │                     │               │                          │
 │        │                  │   MotionCmd 20 Hz   │               ▼                          │
 │        ▼                  │   StateCmd  20 Hz   │         RovController                    │
 │ main.cpp ── CanMcp2515 ───┼────────────────────►│   arming, failsafe, motor test,          │
 │  console, arm/mode/gain   │                     │   stabilize (Pid x2), dry-run limit      │
 │        ▲                  │   Attitude  10 Hz   │               │                          │
 │        │                  │   Status    10 Hz   │               ▼                          │
 │  telemetry print, LEDs ◄──┼─────────────────────┤         ThrusterMixer ──► Thruster x6    │
 └───────────────────────────┘   Thrusters 10 Hz   │               ▲                          │
                                                   │ Mpu6050 ──► AttitudeFilter               │
                                                   └──────────────────────────────────────────┘
```

## Code layout rules

- **`lib/`** holds code shared between boards, or code with no board-specific
  logic. Each library has a `library.json` and keeps its `.hpp` / `.cpp` in
  `src/`.
- **`src/<board>/`** holds one application per board. `main.cpp` only wires
  hardware to logic. Pins and tunable values live in that board's
  `config.hpp`.
- **Hardware-free libraries** (`RovProtocol`, `ThrusterMixer`, `Pid`) build on
  the PC and have unit tests in `test/`.
- **`RovController`** contains all the ROV's decision-making and never touches
  hardware. It takes commands and attitude in, and gives six outputs in
  [-1, 1] back.

## ROV main loop

Everything runs in the Arduino `loop()`. Nothing blocks after `setup()`.

| Task | Rate | What |
|---|---|---|
| `handleCan()` | every loop | drain the CAN receive queue, pass commands to `RovController` |
| `runControl()` | 100 Hz | read IMU → update attitude → `RovController::update()` → write ESCs |
| `sendTelemetry()` | 10 Hz | 3 CAN frames |
| `can.service()` + LED | 10 Hz | recover from bus-off |
| `readSupply()` | 1 Hz | PowerFeather supply voltage |
| `printDebug()` | 2 Hz | status line on USB serial |

`setup()` sends the stop pulse to the ESCs first, then initialises the
PowerFeather SDK, the IMU (including gyro calibration) and CAN, and finally
waits for the rest of the 7 s ESC arming time.

## RovController::update

1. **Link check.** If no `StateCmd` arrived for 500 ms: failsafe, disarm,
   block re-arming, return zero.
2. **Motor test.** If one is running, output only that motor and return.
3. **Disarmed:** return zero.
4. **Scale inputs.** Scale the pilot axes by the gain into a
   `ThrusterMixer::Input`.
5. **STABILIZE** (when the IMU is ok):
   - Roll PID with target 0°, using the gyro rate as the derivative term.
   - Heading hold PID when the yaw stick is centered.
6. **Mix.** `ThrusterMixer::mix()` computes the outputs. If any output would
   exceed 1, all of them are scaled down together, so the direction of motion
   is kept.
7. **Out-of-water limit.** If any output has been non-zero for more than
   10 s, cut all outputs until disarm.

`main.cpp` then multiplies the outputs by the output limit (15 % dry, 100 %
wet). The `Thruster` driver also clamps to the same limit, as a backstop.

## Coordinate conventions

| Quantity | Positive direction |
|---|---|
| surge / forward | forward |
| sway / lateral | right |
| heave / climb | up |
| yaw | clockwise seen from above |
| roll | right side down |
| pitch | nose up |

MPU6050 mounting: X forward, Y left, Z up. `AttitudeFilter` converts sensor
axes to these conventions.

## Libraries

| Library | Depends on | Notes |
|---|---|---|
| `RovProtocol` | – | Plain C++, no Arduino headers |
| `ThrusterMixer` | – | Plain C++ |
| `Pid` | – | Plain C++ |
| `Mpu6050` | Arduino `Wire` | Driver + complementary filter (98 % gyro) |
| `Thruster` | Arduino; `Servo` on AVR | ESP32 backend uses LEDC at 50 Hz / 14 bit |

## External dependencies

| Environment | Package |
|---|---|
| `rov` | `platformio/espressif32` (Arduino core 3.x), `powerfeather/PowerFeather-SDK` |
| `station` | `platformio/atmelavr`, `autowp/autowp-mcp2515` |
| `thruster_test` | `platformio/atmelavr`, `arduino-libraries/Servo` |
| `native` | Unity test framework |

`src/rov/PowerFeatherCompat.cpp` works around a link error in
PowerFeather-SDK 2.1.x under the Arduino framework.
