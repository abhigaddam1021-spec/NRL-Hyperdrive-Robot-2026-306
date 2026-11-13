# Motor-driver architecture — V4.1 finding

## Physical arrangement (team's labels, assigned looking at the robot from its front)

| Socket | Wheel | Board group | Evidence |
|---|---|---|---|
| 1 (leftmost) | RB — `wheel[RearRight]` | MOTOR 2 | seen directly |
| 2 | RF — `wheel[FrontRight]` | MOTOR 2 | seen directly |
| 3 | LF — `wheel[FrontLeft]` | MOTOR 1 | inferred from layout |
| 4 (rightmost) | LB — `wheel[RearLeft]` | MOTOR 1 | inferred from layout |

The labels are kept exactly. They are not a statement about the robot's own left and
right; `steeringSign`, measured on the robot, decides which way "stick right" turns it.

## What the SDK shows

Every program in `libHexaNRL_static.a` (SDK 1.5.0) that drives motors was
disassembled: the seven motor examples, the match runner and the calibration MOTOR
TEST path. Each creates exactly two motor objects, with DIR/PWM 5/10 and 11/12
(`MOTOR_L`, `MOTOR_R`; the examples flip the second). GPIO 1/2 and 15/16 are never
used as motor outputs anywhere in the library, and `BoardPins.h` calls them
bench-only, needing external driver electronics.

## Conclusion

- **Architecture B (two groups, two wheels each) is strongly indicated, not verified.**
  A factory application with only two motor outputs cannot turn four independently
  wired wheels. Since the factory application drives the robot, and the board prints
  two groups, the consistent explanation is one driver output per group.
- It becomes verified by either of: all four wheels turning under the factory
  application, or continuity between the two sockets of a group with power off.
- **Which group is `MOTOR_L` and which is `MOTOR_R` cannot be determined from
  software.** Nothing in the kit names "MOTOR 1" or "MOTOR 2". It stays unverified.
- If B is confirmed, V4 needs **two** logical motor outputs. If the sockets turn out
  to be separate, it needs four, and two of them would have to be the bench-only
  breakout channels.

## How V4.1 handles it

- All four wheel positions stay in the configuration, each with socket and group.
- `pairedSocketsVerified = true` selects architecture B: both wheels of a group name
  the same channel and share one polarity. The firmware creates one motor object per
  channel, so no GPIO pair is ever owned twice, and writes each output once per tick.
- The check rejects: a group label that is only inferred, a paired group split across
  two channels, both groups on one channel, a shared channel without the paired flag,
  and paired wheels with different polarity.
- Left/right polarity remains independent per group. A single wheel that runs
  backwards inside a group must be corrected at its plug.

## Team observations received

Under the factory application all four wheels drive forward and all four take part in
in-place turns, moving as an R-labelled pair and an L-labelled pair. With the
two-output SDK finding above, this supports architecture B. It does not identify the
SDK output behind each group, and it does not calibrate the steering sign: see the
observation section of `HARDWARE-WORKSHEET.md` for the heading check that must come
first.

## Factory MOTOR TEST result

M1 at +25 moved only LF/LB (backward); M2 at +25 moved only RF/RB (forward).
Architecture B is physically confirmed. In the SDK library the test's slot 0 is
`MOTOR_L` and slot 1 is `MOTOR_R`; that screen M1 is slot 0 is corroborated but not
provable without controller source. Details and the remaining checks are in
`HARDWARE-WORKSHEET.md`.

## To finish (no upload needed for 1–3)

1. Look at sockets 3–4 and confirm they sit under MOTOR 1.
2. Power off: continuity between the motor pins of sockets 1 and 2, and of 3 and 4.
3. Done by the team: all four wheels turn under the factory application. Still to
   record: which physical side the R-labelled wheels are on in the robot's own
   frame. Chassis heading is now confirmed: right stick turns the nose right.
4. Identify which group each of `MOTOR_L` / `MOTOR_R` drives: trace the board, or after
   an authorised upload use **HD Wheel Check**, which spins one output at a time.
5. Confirm the driver's current rating covers two motors per output.

Then, in `HyperdriveConfig.h`: set all four `groupObserved` to Verified, set
`pairedSocketsVerified`, give both wheels of each group their channel with
`Status::Verified`, enter the two polarities, and enable the drive as in
`COMMISSIONING.md` stage B.
