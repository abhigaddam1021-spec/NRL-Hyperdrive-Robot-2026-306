# HYPERDRIVE V4.2 — complete control software, turn-direction revision

**Classification: SOFTWARE COMPLETE — HARDWARE COMMISSIONING PENDING**
**Firmware label: NOT READY FOR HARDWARE UPLOAD**

Every requested feature is implemented, host-tested and compiled. Nothing has been
run on the robot. Every output ships disabled because the physical values each
feature needs have not been measured. Nothing was flashed, erased or reset.

| Status | Meaning here | Applies to |
|---|---|---|
| IMPLEMENTED | real control logic in `RobotFirmware/opmodes/` | all features below |
| HOST-TESTED | executed against SDK doubles on the Mac | all features below |
| BUILT | compiled and linked for ESP32-S3 with SDK 1.5.0 | the delivered image |
| HARDWARE-VERIFIED | observed on the robot | **nothing** |
| DEPLOYMENT-READY | may be uploaded | **no** |

## Where things are

| Path | Contents |
|---|---|
| `RobotFirmware/opmodes/HyperdriveConfig.h` | the one hardware configuration, validation, drive mixer |
| `RobotFirmware/opmodes/HyperdriveTeleOp.cpp` | coordinator: drive, claw, crane, payload, R2, R3, turns, RT, faults |
| `RobotFirmware/opmodes/HyperdriveCommissioning.cpp` | `HD Input Monitor`, `HD Wheel Check` |
| `RobotFirmware/opmodes/CoreLiftJog.cpp`, `CoreServoLift.cpp`, `LiftJog.h` | earlier bench diagnostics, unchanged, disabled |
| `tests/` | five host suites; `v4_test.cpp` holds the 96 directive cases |
| `firmware/` | built image, ELF, SHA-256 |
| `build-logs/` | full test and build output |
| `docs/` | commissioning, worksheets, checklist, restoration, limitations, reports |

## Controls

| Control | Action | Configuration status |
|---|---|---|
| Left stick Y (`leftY`) | throttle | implemented; sign unverified; drive disabled |
| Right stick X (`rightX`) | steering | implemented; sign unverified; drive disabled |
| LB `0x200` | toggle claw 85° / 105° | mask from SDK; claw disabled (pulse, travel, settle time unmeasured) |
| RB `0x100` | toggle crane 50° / lowered | mask from SDK; crane disabled (lowered endpoint unverified) |
| R1 / R4 | payload raise / lower | button unassigned; payload disabled |
| R2 | automatic CORE collection | button unassigned; pickups disabled |
| R3 | quick pickup and lift | button unassigned; pickups disabled |
| L1 / L3 / L4 | 180° / right 90° / left 90° | buttons unassigned; turns disabled |
| RT `0x800` | cancel, then safe stow when calibrated | implemented; stow uncalibrated, so RT cancels and inhibits |
| LT `0x400` | STOP / exit | SDK behaviour preserved; also handled first in the coordinator |

## What changed in V4.2

- New `turns.rightTurnSign`: which way the L-labelled wheel pair is commanded for the
  robot to turn right. L3 now always means "robot turns right" and L4 "robot turns
  left", wherever the labelled wheels physically sit. Turns cannot be enabled until it
  is set from an observation.
- Manual steering is unchanged and still uses `steeringSign`.

## What changed in V4.1

- The board's MOTOR 1 / MOTOR 2 groups are recorded per wheel, with RB and RF marked
  observed and LF and LB marked inferred. The drive cannot be enabled on an inferred label.
- Paired groups (two wheels per driver output) are validated as a whole: a paired group
  cannot be split across outputs, and both groups cannot share one.
- No change to claw, crane, payload, pickups, turns, RT or button handling.
- Finding and remaining checks: `docs/MOTOR-GROUPS.md`.

## What changed from V3

- Configuration reorganised into DriveConfig, Servo configs, ControllerMap,
  PickupConfig, TurnConfig, StowConfig, SafetyConfig and TeamConfig. `check()` returns
  a specific reason for every rejection.
- Each stage can be enabled on its own evidence. V3 demanded a confirmed seven-button
  layout before even the drivetrain could be enabled; V4 asks each feature only for the
  buttons it uses.
- Each wheel records its confirmed socket, SDK channel, polarity and routing status.
  Two same-side wheels may share one channel only when the board is proven to parallel
  those sockets (see `docs/KNOWN-LIMITATIONS.md`, item 1).
- Drive maths (`mix`, `deadbanded`, `toPwm`) is separate from the output layer. Optional
  acceleration limit and measured PWM floor; safety stops bypass the limit.
- Controller loss is recoverable: wheels stop, automation is cancelled, servos keep their
  last command, and control returns only after a fresh neutral packet. V3 shut down
  until a new START.
- Taking over mid-automation cancels it and waits for neutral. V3 shut the run down.
- RT without stow calibration no longer removes servo PWM by default. Holding or
  removing PWM is now an explicit per-case choice in SafetyConfig.
- Per-servo state keeps requested, commanded and settled-estimate apart. Position is
  never treated as measured.
- New `HD Input Monitor` (moves nothing) and `HD Wheel Check` (one routed wheel, low
  power, dead-man).

## Build and test

```sh
sh tests/run-host-tests.sh
pio run -d RobotFirmware -e robot -t clean
pio run -d RobotFirmware -e robot
shasum -a 256 -c firmware/SHA256SUMS
```

Start with `docs/COMMISSIONING.md`.
