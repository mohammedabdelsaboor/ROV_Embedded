# ROV-Embedded

[![CI](https://github.com/OWNER/ROV-Embedded/actions/workflows/ci.yml/badge.svg)](https://github.com/OWNER/ROV-Embedded/actions/workflows/ci.yml)

Embedded firmware for a 6-thruster ROV in the BlueROV2 "vectored" layout. A
surface station sends pilot commands over a CAN bus that runs through the
tether.

> **Status: work in progress.** Everything builds and the unit tests pass, but
> the firmware has not yet been run on the real vehicle. See
> [docs/roadmap.md](docs/roadmap.md) for what is unfinished.

```
 SURFACE                                  TETHER (twisted pair)           ROV
┌──────────────────────┐                                      ┌─────────────────────────────┐
│ Arduino Uno          │   CAN_H ──────────────────── CAN_H   │ ESP32-S3 PowerFeather       │
│  + MCP2515 module    ├── CAN_L ──────────────────── CAN_L ──┤  + SN65HVD230 transceiver   │
│  + 2 joysticks       │   GND   ──────────────────── GND     │  + MPU6050 (SDA 35 / SCL 36)│
│  + 4 buttons, 2 LEDs │         250 kbit/s, 120 Ω each end   │  + 6 x Basic ESC + T200     │
│ USB serial console   │                                      │                             │
└──────────────────────┘                                      └─────────────────────────────┘
```

## Features

- **Thruster mixing:** 6-thruster vectored mixing, using the same motor
  factors as ArduSub's `VECTORED` frame.
- **Modes:** MANUAL, and STABILIZE (roll held level, heading held when the
  yaw stick is centered).
- **CAN protocol:** a small CAN bus protocol with commands at 20 Hz and
  telemetry at 10 Hz.
- **Safety:** link-loss failsafe, a re-arm lock, arming only with centered
  sticks, out-of-water thrust limits, and a time-limited single-motor test.
- **One PlatformIO project for every board:** ROV, station, and a Mega
  bench-test sketch.
- **Unit tests** for the protocol and mixer that run on the PC.

## Quick start

Requires [PlatformIO](https://platformio.org/) (the VS Code extension or the
CLI).

```bash
git clone https://github.com/OWNER/ROV-Embedded.git
cd ROV-Embedded

pio run                             # build ROV + station
pio run -e rov -t upload            # flash the ESP32-S3 PowerFeather
pio run -e station -t upload        # flash the Arduino Uno
pio device monitor                  # serial console, 115200 baud
pio test -e native                  # unit tests on the PC
```

In VS Code, open this folder itself (not a parent folder) so the PlatformIO
toolbar appears. Then pick the environment in the status bar.

> The ROV firmware starts in **out-of-water mode** (15 % thrust limit, 10 s
> run limit). Set `-DROV_OUT_OF_WATER=0` in `platformio.ini` once the vehicle
> is in the water.

## Repository layout

```
ROV-Embedded/
├── platformio.ini          build environments: rov, station, thruster_test, native
├── src/
│   ├── rov/                ESP32-S3 PowerFeather firmware
│   ├── station/            Arduino Uno surface station firmware
│   └── thruster_test/      Arduino Mega single-thruster bench test
├── lib/                    shared, board-independent libraries
│   ├── RovProtocol/        CAN message ids + encode/decode
│   ├── ThrusterMixer/      6-thruster vectored mixer
│   ├── Thruster/           Blue Robotics Basic ESC driver (AVR + ESP32)
│   ├── Mpu6050/            IMU driver + attitude filter
│   └── Pid/                PID controller
├── test/                   host unit tests (Unity)
├── docs/                   documentation
└── .github/workflows/      CI: build, test, format check
```

## Documentation

| Document | Contents |
|---|---|
| [docs/hardware.md](docs/hardware.md) | Parts, pinouts, wiring, CAN bus and power |
| [docs/operation.md](docs/operation.md) | Controls, modes, safety, first power-up checklist, tuning |
| [docs/architecture.md](docs/architecture.md) | Software structure, control loop, data flow |
| [docs/protocol.md](docs/protocol.md) | CAN message reference |
| [docs/roadmap.md](docs/roadmap.md) | Unfinished work and planned features |
| [CONTRIBUTING.md](CONTRIBUTING.md) | Code style, formatting, tests, pull requests |

## References

- [ArduSub `AP_Motors6DOF.cpp`](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Motors/AP_Motors6DOF.cpp): source of the vectored frame factors
- [BlueOS](https://github.com/bluerobotics/blueos): Blue Robotics' open-source ROV platform
- [MUR](https://github.com/scottmayberry/MUR): an open-source 6-thruster vectored ROV
- [Orca](https://github.com/clydemcqueen/orca): ROS-based BlueROV2 control
- [ESP32-S3 PowerFeather docs](https://docs.powerfeather.dev/)
- [autowp-mcp2515](https://github.com/autowp/arduino-mcp2515)

## License

[MIT](LICENSE)
