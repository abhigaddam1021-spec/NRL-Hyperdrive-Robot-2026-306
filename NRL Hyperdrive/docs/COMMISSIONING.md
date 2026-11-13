# Commissioning — from disabled software to a working robot

Each stage is enabled only by entering its own measured values in
`RobotFirmware/opmodes/HyperdriveConfig.h`. The build refuses any stage whose
evidence is missing. A stage never requires a later stage.

To enter values, replace `inline constexpr Robot configuration{};` with a function
that fills the fields, for example (blanks are yours to measure, not suggestions):

```cpp
inline constexpr Robot configuration = [] {
    Robot r;
    r.drive.wheel[FrontLeft] = {3, Channel::___, ___, Status::Verified};
    // ...
    return r;
}();
```

After every edit: `sh tests/run-host-tests.sh`, then a clean build.

## Before anything: the upload decision

Read `FACTORY-RESTORATION.md`. Uploading replaces the factory application and no
restoration route has been proven for this single hub. That decision is separate from
everything below and is yours.

## Stage A — read-only (no actuator can move)

Without flashing:
1. Record hub and motor-board markings, board revision, team number and field channel.
2. Chassis on blocks. Using the existing factory application, push forward, then steer.
   Record for each wheel: turns or not, and which way. **If all four wheels turn**, see
   `KNOWN-LIMITATIONS.md` item 1: the SDK's factory programs create only two motor
   outputs, which points to the four sockets being fed in pairs.
3. Confirm sockets 3–4 sit under MOTOR 1 (see `MOTOR-GROUPS.md`). With power off,
   check continuity between the motor pins of sockets 1–2 and of
   sockets 3–4. Continuity proves paired sockets. Trace each socket (or pair) back to
   its driver output and GPIO pair.

After upload is authorised: run **HD Input Monitor**. It creates no motor or servo
object. Press R1, R2, R3, R4, L1, L3, L4 one at a time and record the mask shown for
each, twice. Enter each in `map.slot[...]` with `Status::Verified` and where the
evidence came from.

## Stage B — drivetrain (chassis securely supported)

1. Set each `groupObserved` to Verified once seen. Enter each wheel's traced `channel` and set `routing = Status::Verified`. Leave
   untraced wheels alone. Set `pairedSocketsVerified` only on the continuity result.
   Channel M4 additionally needs `motor4PinsFreeVerified` (GPIO 15/16 are shared with
   the line array).
2. Set `wheelCheckEnabled = true`, `wheelCheckPwm` between 11 and 64. Build, upload.
3. Run **HD Wheel Check**. RB selects the next routed wheel. Holding LB alone spins it
   with a positive raw command; release stops it. Record `sign = +1` if it rolled
   forward, `-1` if backward.
4. Confirmed under the factory application: right stick swings the nose right. That
   does not set V4.1's `steeringSign`, which depends on the label side and group
   polarities (see "Steering observation update" in `HARDWARE-WORKSHEET.md`). Enter
   it last, by checking on blocks that right stick reproduces the factory wheel pattern.
   Turn the wheel check off. Enter signs, `throttleSign`, `steeringSign`, `deadband`,
   `maxPwm`; set `driverCapacityVerified` once four-motor current is known to be within
   the driver's rating. Set `drive.enabled = true`.
5. Still on blocks, verify forward, reverse, both turns, RT, LT and switching the
   controller off.

## Stage C — servos (mechanisms unloaded and supported)

For each servo enter: port, 0° and 180° pulse widths, verified safe `low`/`high`, both
endpoints, staged `start` pose, worst-case `settleMs`.
- Claw: 85° open / 105° closed are fixed by the configuration check.
- Crane: 50° raised. Enter `crane.second` only as a physically verified lowered
  endpoint, never above 125°, and set `craneClearanceVerified` only after the whole
  path is clear. Do not drive through the resistance near 100°.
- Payload: SERVO_3 or SERVO_4 as actually wired, endpoints, `parkedPayload`,
  `payloadPathAndPowerVerified`. Needs R1 and R4 verified.
- `safety.responsesVerified`: for link loss, RT-without-stow and faults, decide per
  case whether servos hold or lose PWM, by testing what each does under load.

## Stage D — manual operation

Drive, steer, claw, crane, payload. Check interlocks, held buttons, conflicting
buttons, link loss, STOP.

## Stage E — automation

- Pickups: `pickup.enabled`, `timingAndGeometryVerified`, `timeoutMs` greater than
  500 + 2 × crane settle + grip settle. Needs R2 and R3 verified.
- Turns: set `turns.rightTurnSign` (+1 if the L-labelled wheels must run forward for
  the robot to turn right, −1 if backward), then confirm on blocks that L3 spins the
  robot right and L4 left. Then `TurnMode::Heading` with gyro sign and tolerance, or `TurnMode::Timed` with
  three separately measured durations (approximate). Needs L1, L3, L4 verified.
- Safe stow: rest angles for each mechanism, an external health input on D1/D2/D3,
  and `pathsAndRetentionVerified`. Until then RT cancels and inhibits.
