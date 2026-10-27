# Hardware and wiring

## Parts

| Where | Part | Notes |
|---|---|---|
| ROV | ESP32-S3 PowerFeather | Main controller. Its supply input accepts up to 18 V |
| ROV | MPU6050 breakout (GY-521) | Accelerometer + gyro, on I2C |
| ROV | SN65HVD230 CAN transceiver | 3.3 V logic. Do **not** use a 5 V TJA1050 |
| ROV | 6 x Blue Robotics T200 + Basic ESC | 4 horizontal at 45°, 2 vertical |
| ROV | 12–16 V battery | Powers the ESCs and the PowerFeather |
| Surface | Arduino Uno | Station controller |
| Surface | MCP2515 CAN module (with TJA1050) | 5 V, SPI |
| Surface | 2 x analog thumbstick (KY-023) | Optional, see `STATION_USE_JOYSTICKS` |
| Surface | 4 push buttons, 2 LEDs + 330 Ω | Arm, mode, gain +, gain − |
| Tether | One twisted pair + ground | CAN_H / CAN_L / GND |
| Bench | Arduino Mega 2560 | Only for the `thruster_test` sketch |

## Thruster layout

```
              FRONT
        T2 //       \\ T1
            |  T6  T5 |
        T4 \\       // T3
              REAR
```

| Thruster | Position | ESP32 GPIO | PowerFeather pin |
|---|---|---|---|
| T1 | front-right, 45° | 15 | D5 |
| T2 | front-left, 45° | 16 | D6 |
| T3 | rear-right, 45° | 17 | D9 |
| T4 | rear-left, 45° | 18 | D10 |
| T5 | vertical right | 12 | D12 |
| T6 | vertical left | 11 | D13 |

Like the BlueROV2 standard frame, this layout controls roll but not pitch.

## ROV: ESP32-S3 PowerFeather

| Signal | PowerFeather | GPIO | Goes to |
|---|---|---|---|
| I2C SDA | SDA | 35 | MPU6050 SDA |
| I2C SCL | SCL | 36 | MPU6050 SCL |
| 3.3 V | 3V3 | – | MPU6050 VCC, transceiver VCC |
| CAN TX | A0 | 10 | SN65HVD230 `CTX` / `TXD` |
| CAN RX | A1 | 9 | SN65HVD230 `CRX` / `RXD` |
| ESC signals | D5 D6 D9 D10 D12 D13 | 15 16 17 18 12 11 | Basic ESC white wires (T1–T6) |
| GND | GND | – | ESC signal grounds (black), MPU6050, transceiver |
| Status LED | on-board | 46 | Solid = armed, slow blink = disarmed, fast blink = no link |

All pins are set in [`src/rov/config.hpp`](../src/rov/config.hpp).

- **ESC power** comes straight from the battery. Only the signal and ground
  wires of each ESC go to the PowerFeather.
- **The Basic ESC accepts a 3.3 V signal**, so no level shifter is needed.
- **Mount the MPU6050** with the X arrow pointing forward and the chip side up.
  If it has to go in another way, change `IMU_AXIS_SIGN`.
- **The 3V3 and VSQT rails** are switched on by `Board.init()` from the
  PowerFeather SDK.

## Station: Arduino Uno

| Uno pin | Goes to |
|---|---|
| D10 | MCP2515 `CS` |
| D11 / D12 / D13 | MCP2515 `SI` / `SO` / `SCK` |
| 5V / GND | MCP2515 `VCC` / `GND` |
| A0 | left stick Y → heave (up/down) |
| A1 | left stick X → yaw (turn) |
| A2 | right stick Y → surge (forward/back) |
| A3 | right stick X → sway (left/right) |
| D3 | ARM button (to GND) |
| D4 | MODE button (to GND) |
| D5 | GAIN + button (to GND) |
| D6 | GAIN − button (to GND) |
| D7 | "armed" LED (through 330 Ω) |
| D8 | "link" LED (through 330 Ω) |

All pins are set in [`src/station/config.hpp`](../src/station/config.hpp).

**Check the MCP2515 crystal.** Read the marking on the metal can next to the
chip. Most modules have 8 MHz, which is the default. If yours says 16.000,
set `#define STATION_MCP_CLOCK MCP_16MHZ`.

## CAN bus through the tether

- **Use one twisted pair for CAN_H / CAN_L**, plus a ground wire.
- **Terminate with 120 Ω at both ends of the bus, and only there.** On the
  MCP2515 module that is jumper `J1`. Many SN65HVD230 boards already have a
  120 Ω resistor (R2) fitted.
- **At 250 kbit/s the bus works up to roughly 250 m.** The bitrate is
  `rov::CAN_BITRATE` in the protocol library.
- **Check the termination with a multimeter:** with both boards unpowered, you
  should read about 60 Ω between CAN_H and CAN_L.

## Bench test: Arduino Mega + one thruster

| Mega | Goes to |
|---|---|
| D2–D9 | Basic ESC signal (thruster ids 0–7) |
| GND | ESC signal ground |

The ESC is powered from a 12–16 V supply, never from the Arduino.
