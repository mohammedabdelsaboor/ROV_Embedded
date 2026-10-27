# CAN protocol

- **Frames:** classic CAN 2.0A, 11-bit identifiers, 250 kbit/s.
- **Byte order:** multi-byte fields are little-endian.
- **Priority:** commands use lower ids than telemetry, so they win bus
  arbitration.
- **Source of truth:** [`lib/RovProtocol/src/RovProtocol.hpp`](../lib/RovProtocol/src/RovProtocol.hpp).
  Both firmwares use it, so they cannot disagree.

## Messages

| Id | Name | Direction | Rate | Length |
|---|---|---|---|---|
| 0x100 | `MotionCmd` | station → ROV | 20 Hz | 8 |
| 0x101 | `StateCmd` | station → ROV | 20 Hz | 4 |
| 0x102 | `MotorTestCmd` | station → ROV | on demand | 3 |
| 0x200 | `AttitudeTlm` | ROV → station | 10 Hz | 8 |
| 0x201 | `StatusTlm` | ROV → station | 10 Hz | 8 |
| 0x202 | `ThrusterTlm` | ROV → station | 10 Hz | 6 |

### 0x100 MotionCmd

| Bytes | Field | Type | Meaning |
|---|---|---|---|
| 0–1 | surge | int16 | ±1000, + forward |
| 2–3 | sway | int16 | ±1000, + right |
| 4–5 | heave | int16 | ±1000, + up |
| 6–7 | yaw | int16 | ±1000, + clockwise seen from above |

### 0x101 StateCmd (heartbeat)

| Byte | Field | Meaning |
|---|---|---|
| 0 | armed | 1 = arm requested |
| 1 | mode | 0 = MANUAL, 1 = STABILIZE |
| 2 | gain | pilot gain, percent (clamped to 100) |
| 3 | seq | increments every frame, echoed in `StatusTlm` |

The ROV treats this frame as the link heartbeat. If none arrives for
`LINK_TIMEOUT_MS` (500 ms), it stops all thrusters and disarms.

### 0x102 MotorTestCmd

| Byte | Field | Meaning |
|---|---|---|
| 0 | motor | 1–6 |
| 1 | throttle | int8 percent, −100..100 |
| 2 | duration | tenths of a second. The ROV caps it at 3 s |

The ROV ignores this command while armed.

### 0x200 AttitudeTlm

| Bytes | Field | Type | Meaning |
|---|---|---|---|
| 0–1 | roll | int16 | centidegrees, + right side down |
| 2–3 | pitch | int16 | centidegrees, + nose up |
| 4–5 | yaw | int16 | centidegrees, gyro-integrated, −18000..18000 |
| 6–7 | yawRate | int16 | centidegrees per second |

### 0x201 StatusTlm

| Bytes | Field | Meaning |
|---|---|---|
| 0 | flags | bit field, see below |
| 1 | mode | 0 = MANUAL, 1 = STABILIZE |
| 2 | gain | percent |
| 3 | seq | last `StateCmd.seq` received |
| 4–5 | supplyMv | uint16, PowerFeather supply voltage in mV |
| 6–7 | imuTemp | int16, centidegrees C |

| Bit | Flag | Meaning |
|---|---|---|
| 0 | `ARMED` | thrusters respond to pilot input |
| 1 | `IMU_OK` | last IMU read succeeded |
| 2 | `FAILSAFE` | link lost, outputs stopped |
| 3 | `OUT_OF_WATER` | dry-test output limit is active |
| 4 | `DRY_TIMEOUT` | ran too long out of water, outputs cut until disarm |
| 5 | `MOTOR_TEST` | a motor test is running |

### 0x202 ThrusterTlm

Six int8 values, T1..T6: the actual thruster output in percent of full
thrust, after limits and direction reversal.

## Changing the protocol

1. Edit `RovProtocol.hpp` / `RovProtocol.cpp`.
2. Add or update a round-trip test in `test/test_protocol/`.
3. Flash **both** the ROV and the station: the boards don't check protocol
   versions yet (see [roadmap](roadmap.md)).
