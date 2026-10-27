# Roadmap and unfinished work

This page lists what is **not done yet**. Tick items off (or open a GitHub
issue for each) as they are completed.

Legend: 🔴 needed before the first dive · 🟡 important · 🟢 nice to have

## 1. Verification on real hardware

Everything compiles and the unit tests pass, but none of the following has
been checked on the real vehicle.

- [ ] 🔴 ROV firmware boots on the PowerFeather: IMU, CAN and PowerFeather SDK all report `ok`
- [ ] 🔴 Station firmware talks to the MCP2515 (crystal setting confirmed)
- [ ] 🔴 CAN link works across the full tether length, and termination is measured at ~60 Ω
- [ ] 🔴 ESC PWM from the ESP32 (LEDC, 50 Hz) is accepted by all six Basic ESCs
- [ ] 🔴 Thruster numbering confirmed with `test 1..6`
- [ ] 🔴 Thruster directions confirmed in water; `THRUSTER_REVERSED` filled in
- [ ] 🔴 IMU axis signs confirmed (roll right = positive, nose up = positive)
- [ ] 🔴 Link failsafe confirmed: unplug the tether while armed, and the thrusters stop within 0.5 s
- [ ] 🟡 Joystick axes and directions confirmed (`INVERT_*` in the station config)
- [ ] 🟡 Supply voltage telemetry matches a multimeter

## 2. Control

- [ ] 🔴 Tune the roll PID (`ROLL_KP/KI/KD`) in the water. The current values are untuned starting points
- [ ] 🔴 Tune the heading-hold PID (`YAW_KP/KD`)
- [ ] 🟡 **Depth sensor + depth hold:** the MPU6050 cannot measure depth. Add a Blue Robotics Bar30 (MS5837) on the same I2C bus and a depth PID on heave
- [ ] 🟡 **Magnetometer for heading:** heading is gyro-only and drifts. Add a magnetometer (e.g. QMC5883L / LIS3MDL), or replace the IMU with a 9-axis one (ICM-20948 / BNO085)
- [ ] 🟡 **Accelerometer calibration:** only the gyro is calibrated, at boot. Add an accel offset calibration and store it in flash (NVS)
- [ ] 🟢 Forward/vertical coupling compensation, like ArduSub's `forward_coupling_limit`
- [ ] 🟢 Expo / response curves on the sticks
- [ ] 🟢 Runtime PID tuning over CAN or the serial console instead of reflashing

## 3. Safety

- [ ] 🔴 **Leak detection:** no leak sensor yet. Add a leak probe (e.g. Blue Robotics SOS) on a GPIO, a status flag, and a station alarm
- [ ] 🟡 **Low-battery failsafe:** supply voltage is reported, but nothing acts on it. Add warning and critical thresholds
- [ ] 🟡 **Hardware watchdog:** enable the ESP32 task watchdog on the main loop, so a hang stops the thrusters
- [ ] 🟡 **Main battery current sensing:** the ESCs draw from the battery directly, and that current is not measured
- [ ] 🟢 Internal temperature / humidity sensor in the electronics tube

## 4. Communication

- [ ] 🟡 **Protocol version / handshake:** a mismatched ROV and station firmware currently fail silently
- [ ] 🟡 **CAN error counters in telemetry:** TX/RX error counts and bus-off events, sent to the station
- [ ] 🟡 **Motor test acknowledgement:** the ROV gives no explicit reply to a `MotorTestCmd`
- [ ] 🟢 More telemetry: depth, leak, battery current, loop timing
- [ ] 🟢 Accessory commands: lights (PWM), camera tilt servo, gripper

## 5. Surface station

- [ ] 🟡 **Local display:** telemetry only goes to the USB serial console. Add an I2C OLED or LCD on the Uno
- [ ] 🟡 **PC control app** (planned as a separate `rov_control` repo): gamepad input, a telemetry dashboard and logging, talking to the station over USB serial
- [ ] 🟡 Machine-readable serial telemetry output (e.g. CSV or JSON lines) for the PC app
- [ ] 🟢 Consider moving the station to an ESP32 or a bigger AVR. The Uno has 2 KB of RAM and 32 KB of flash (about 21 % RAM and 32 % flash used today)

## 6. Video

- [ ] 🟡 **Camera link:** CAN cannot carry video. Choose a separate path, such as analog camera over a spare tether pair, or Ethernet over a Fathom-X tether interface

## 7. Tooling and project

- [ ] 🟡 Unit tests for `RovController` (arming, failsafe, re-arm lock, dry-run limit). It is hardware-free but currently includes `config.hpp` and Arduino helpers
- [ ] 🟡 Unit tests for `Pid` and `AttitudeFilter`
- [ ] 🟢 Over-the-air firmware updates for the ROV (no need to open the hull)
- [ ] 🟢 Remove `src/rov/PowerFeatherCompat.cpp` once PowerFeather-SDK fixes the `MasterI2C` vtable issue upstream
- [ ] 🟢 Wiring diagram / schematic (KiCad) in `docs/`
- [ ] 🟢 Photos of the assembled vehicle and station

## Known limitations

- The vectored frame with two side-mounted vertical thrusters **cannot control
  pitch**. This is a property of the frame layout, not a software bug.
- Heading drifts by a few degrees per minute (gyro only).
- Out-of-water mode is the **default**. Remember to build with
  `ROV_OUT_OF_WATER=0` for water tests, or thrust stays limited to 15 %.
