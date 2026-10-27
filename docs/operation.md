# Operation

## Station controls

| Control | Action |
|---|---|
| Right stick | forward/back (surge), left/right (sway) |
| Left stick | up/down (heave), turn (yaw) |
| ARM button | arm / disarm. Sticks must be centered to arm |
| MODE button | MANUAL ↔ STABILIZE |
| GAIN + / − | pilot gain 10–100 % in 10 % steps, starts at 40 % |
| "armed" LED | ROV reports armed |
| "link" LED | telemetry is arriving from the ROV |

## Serial console

Open with `pio device monitor` (115200 baud, newline line ending).

```
arm | disarm                 arm / disarm
manual | stab                flight mode
gain 50                      pilot gain in percent
m 30 0 0 0                   (serial mode only) surge sway heave yaw in percent
stop                         zero motion and disarm
test 1 20 2                  spin thruster 1 at 20 % for 2 s (disarmed only)
tlm                          toggle the 1 Hz telemetry line
h                            help
```

To drive without joysticks, build the station with
`-DSTATION_USE_JOYSTICKS=0`. The `m` command then sets the motion, and it
stays until the next `m` or `stop`.

The telemetry line looks like this:

```
ROV ARMED    STAB   40% | roll 1.2 pitch -0.4 yaw 12.0 | T -6 -6 6 6 0 0 | 15.82 V 31.4 C DRY-LIMIT
```

## Modes

**MANUAL:** stick inputs are scaled by the gain and go straight through the
mixer.

**STABILIZE:**

- **Roll:** a PID loop holds the ROV level using the two vertical thrusters.
- **Heading:** when the yaw stick is centered, the heading at that moment is
  held. Moving the yaw stick turns the ROV normally, and the new heading is
  captured when the stick is released.
- **Heading drift:** heading comes from the gyro only (there is no
  magnetometer), so it drifts slowly. It is fine for holding a course for a
  few minutes, not for navigation.
- **No IMU:** if the IMU stops responding, STABILIZE behaves like MANUAL.

## Safety features

| Feature | Behaviour |
|---|---|
| Link failsafe | The ROV stops all thrusters and disarms if no state command arrives for 500 ms |
| Re-arm lock | After boot or link loss, the ROV arms only after it has seen a disarm |
| Station link check | The station drops its arm request when telemetry stops for 1 s |
| Centered sticks | Both the station and the ROV refuse to arm unless all sticks are centered |
| Out-of-water mode | `ROV_OUT_OF_WATER=1`: output limited to 15 %, thrusters cut after 10 s of running. Disarm to reset |
| Output limit | Applied inside the thruster driver, so no command can exceed it |
| Motor test | Disarmed only, at most 3 s, still capped by the output limit |
| ESC arming | The ROV sends the stop signal for 7 s at boot |

## First power-up checklist

1. **Station alone:** flash `station` and open the monitor. Expect `CAN ok`,
   then `ROV: no link` every second. If it says `CAN init FAILED`, check the
   SPI wiring and the crystal setting.
2. **ROV alone on USB:** flash `rov` and open the monitor. The boot line
   should show `IMU ok`, `CAN ok`, `PowerFeather ok` and `OUT OF WATER`. Keep
   the ROV still while it boots, because the gyro is calibrated then.
3. **Connect the CAN bus.** The station prints `Link up.` and a telemetry line
   every second. Tilt the ROV: right side down should give positive roll, and
   nose up should give positive pitch. If either is wrong, fix
   `IMU_AXIS_SIGN`.
4. **Check the thruster numbering** with the ROV out of water. Run `test 1 20`
   through `test 6 20` and make sure the right thruster spins each time.
5. **Check thruster directions in a tank or pool.** Set `ROV_OUT_OF_WATER=0`
   and gain 20 %, arm, then push each stick axis on its own:
   - Forward should move the ROV forward.
   - Right should move it right.
   - Up should move it up.
   - Turning right should rotate it clockwise.

   If an axis is wrong, find the thruster that pushes the wrong way and set
   it to `true` in `THRUSTER_REVERSED` in `src/rov/config.hpp`. Never edit the
   mixer matrix.
6. **Try STABILIZE** (see tuning below).

## Tuning stabilize

The gains are in `src/rov/config.hpp`. The error is in degrees and the output
is a fraction of full thrust.

| Symptom | Change |
|---|---|
| Roll oscillates / wobbles | lower `ROLL_KP`, or raise `ROLL_KD` slightly |
| Roll recovers slowly | raise `ROLL_KP` |
| Stays leaning a few degrees | raise `ROLL_KI` |
| Heading hold overshoots | lower `YAW_KP`, raise `YAW_KD` |
| Heading hold is weak | raise `YAW_KP` |

Change one value at a time, by about 25 % each step.

## Bench test (Arduino Mega)

`pio run -e thruster_test -t upload`, then in the monitor:

| Command | Action |
|---|---|
| `d <id>` | forward 10 % for 2 s, stop, reverse 10 % for 2 s, stop |
| `t <id> <pct>` | set throttle (capped at 15 %) |
| `p <id> <us>` | raw pulse width (capped at the same limit) |
| `r <id>` | ramp up / down / reverse |
| `s` | stop all |

Thruster id = position in `THRUSTER_PINS` (pin 2 = id 0, pin 3 = id 1, ...).
The `d` test also runs once at boot for `AUTO_TEST_ID` (set it to `-2` to
disable).
