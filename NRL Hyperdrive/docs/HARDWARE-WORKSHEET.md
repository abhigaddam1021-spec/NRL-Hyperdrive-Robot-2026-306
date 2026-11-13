# Hardware calibration worksheet

Known = physically observed by the team. Blank = must be measured; nothing here is
assumed by the software.

| Item | Known | To record |
|---|---|---|
| Hub | one HEXA Command Hub, USB 303A:1001 | board revision ___; installed SDK version ___ |
| Team / field channel | build uses placeholder 001 "Bench", channel 1 | real team ___ / name ___ / channel ___ |
| Motor board | one board, four sockets | part / revision ___; continuous current rating ___ |
| Socket 1 — Right Back — `wheel[RearRight]` | position confirmed; under MOTOR 2 (seen) | channel ___ / DIR,PWM ___ / sign ___ |
| Socket 2 — Right Front — `wheel[FrontRight]` | position confirmed; under MOTOR 2 (seen) | channel ___ / DIR,PWM ___ / sign ___ |
| Socket 3 — Left Front — `wheel[FrontLeft]` | position confirmed; MOTOR 1 inferred — confirm ___ | channel ___ / DIR,PWM ___ / sign ___ |
| Socket 4 — Left Back — `wheel[RearLeft]` | position confirmed; MOTOR 1 inferred — confirm ___ | channel ___ / DIR,PWM ___ / sign ___ |
| Sockets paralleled? | SDK factory programs use only two outputs (5/10, 11/12) | 1–2 continuity ___ / 3–4 continuity ___ / all four wheels turn under factory app ___ |
| Group to SDK output | not determinable from software | MOTOR 1 = MOTOR_L or MOTOR_R ___ / MOTOR 2 = ___ / current rating per output ___ |
| Wheel labels | assigned looking at the robot from its front; kept as labelled | — |
| Drive | — | leftY forward sign ___ / rightX right sign ___ / deadband ___ / max PWM ___ / min PWM ___ / four-motor current ___ |
| Servo 1 claw | SERVO_1, open 85°, closed 105°, smooth under factory app | model ___ / pulse at 0° and 180° ___ / safe low, high ___ / staged start ___ / settle ms ___ |
| Servo 2 crane | SERVO_2, raised 50°, reference 90°, larger angle lowers, resistance near 100°, 125° requested | cause of resistance ___ / verified lowered endpoint ___ / full-path clearance ___ / pulses ___ / settle ms ___ |
| Servo 3 payload | exists | socket ___ / pulses ___ / raised ___ / lowered ___ / parked ___ / settle ms ___ |
| Retention | — | claw, crane, payload behaviour on PWM removal ___ / on link loss ___ / on power loss ___ |
| R2 / R3 | 500 ms dwell after lowering | worst-case lower, grip, raise times ___ / total timeout ___ |
| Turns | — | gyro present and sign ___ / tolerance ___ / turn PWM ___ / or timed 180 ___ ms, right 90 ___ ms, left 90 ___ ms |
| Safe stow | — | rest angles claw ___ crane ___ payload ___ / health input pin ___ / both stow orders tested ___ |

## Drivetrain observations under the factory application (reported by the team, 12 November 2026)

| # | Observation | What it establishes | What it does not establish |
|---|---|---|---|
| 1 | All four wheels operate during forward driving | every socket is driven by the factory application | — |
| 2 | All four wheels take part in in-place turning | the two sides are driven independently of each other | — |
| 3 | Right-stick turn command: RF and RB rotate forward, LF and LB rotate backward | the R-labelled pair moves together and opposite to the L-labelled pair | which way the chassis turns; the steering sign |
| 4 | RB and RF sit under the board's MOTOR 2 group | group of sockets 1–2 | which SDK output feeds MOTOR 2 |
| 5 | LF and LB strongly inferred under MOTOR 1 | nothing yet — still to be seen | — |

**Architecture.** The SDK 1.5.0 factory programs create only two motor outputs
(`MOTOR_L` 5/10, `MOTOR_R` 11/12). Observations 1–3 show four wheels moving as two
pairs. Together these support the two-group arrangement (two wheels per driver output),
which is the implementation V4.1 keeps. Still to record before `pairedSocketsVerified`
is set: observation 5 seen directly, and power-off continuity within each socket pair
(1–2 ___ / 3–4 ___). The SDK version installed on the hub is also unconfirmed ___.

**Steering sign — NOT calibrated.** With the robot's own right side running forward and
its own left side running backward, a differential chassis turns left, not right. So
observation 3 means one of the following, and only watching the robot can say which:

- the wheel labels were assigned looking at the robot from its front, so the
  R-labelled wheels may be on the robot's own left side, in which case observation 3
  is an ordinary right turn; or
- "forward" was judged per wheel rather than as the direction that moves the robot
  forward; or
- the factory application really turns the chassis left for a right-stick command.

Record on the robot, on the floor or on blocks with a heading mark:

| Check | Record |
|---|---|
| Right-stick command: chassis nose swings toward the driver's right or left? | ___ |
| Seen from behind the robot (robot's own frame), the R-labelled wheels are on its left or right? | ___ |
| "Forward" for a wheel = the rotation that moves the robot forward? | ___ |
| Stick fully forward: does every wheel roll the robot forward? | ___ |

`steeringSign` and `throttleSign` stay unset (0) until these are filled in.

**MOTOR_L / MOTOR_R correspondence — unverified.** Wheel motion cannot identify it:
the factory application may flip either output in software, so a given wheel direction
is consistent with either GPIO pair. It needs a board trace, or a single-output test
(`HD Wheel Check` after an authorised upload, or the factory MOTOR TEST screen if it
drives one output at a time).

| Group | SDK output | Evidence |
|---|---|---|
| MOTOR 1 | ___ | ___ |
| MOTOR 2 | ___ | ___ |

No configuration value was changed on the strength of these observations; every
drive output remains disabled.

## Steering observation update (reported by the team, 12 November 2026)

| # | Observation under the factory application | Status |
|---|---|---|
| 6 | Right joystick pushed RIGHT: the robot's front rotates RIGHT | confirmed |
| 7 | All four wheels take part in the rotation, operating as two pairs | confirmed |
| 8 | Socket order remains RB, RF, LF, LB | confirmed, unchanged |

**Chassis direction — resolved.** The earlier open question "does the nose swing right
or left on a right-stick command?" is answered: right. The factory application steers
conventionally.

**What follows for the wheel labels.** Observation 3 recorded RF and RB rotating
forward and LF and LB backward during that same right turn. A chassis only turns right
when the wheels on its own left side drive forward. So, taking observation 3 as
recorded, the R-labelled wheels are on the robot's own left side and the L-labelled
wheels on its own right. That matches how the labels were assigned (looking at the
robot from its front). The labels are kept exactly as they are; one direct look
settles it:

| Check | Record |
|---|---|
| Standing behind the robot, facing the way it drives forward: the R-labelled wheels (RB, RF) are on its left or right? | ___ |
| In observation 3, "forward" meant the rotation that moves the robot forward? | ___ |

**V4.1 steering polarity — still unset, on purpose.** The factory result does not carry
over directly, for two reasons:

1. V4.1's mixer sends throttle + steering to the L-labelled pair and throttle −
   steering to the R-labelled pair. If the L-labelled pair is on the robot's own right
   side, a positive steering value turns the robot left, so `steeringSign` would need to
   be −1 (assuming rightX reads positive to the right and both group polarities are
   entered correctly). That depends on the two checks above and on the group
   polarities, none of which are recorded yet.
2. Which SDK output drives which group is still unknown, and so are the polarities.

`steeringSign` is therefore calibrated on V4.1 itself, last: with routing and both
group polarities entered and the robot on blocks, right stick must give the wheel
pattern of observation 3; if it gives the opposite, `steeringSign` is flipped.

**Automatic turns need their own check.** In `HyperdriveTeleOp.cpp`, `beginTurn`
commands the L-labelled pair forward and the R-labelled pair backward for a "right"
turn. It does not pass through `steeringSign`. If the L-labelled pair is on the
robot's own right side, L3 ("right 90") would turn the robot left and L4 would turn it
right, with L1's direction set by `halfTurnSign`. Firmware behaviour was not changed in
this update. Turns stay disabled; this must be resolved (a configuration field or a
confirmed label side) before `TurnConfig` is enabled.

**MOTOR_L / MOTOR_R** — unchanged: not inferred from this observation, still unverified.

## Remaining drivetrain calibration, in order

| # | Requirement | Needs upload? | Record |
|---|---|---|---|
| 1 | LF and LB seen under MOTOR 1 | no | ___ |
| 2 | Continuity within sockets 1–2 and within 3–4, power off | no | ___ / ___ |
| 3 | Which side of the robot (own frame) the R-labelled wheels are on | no | ___ |
| 4 | SDK output behind MOTOR 1 and behind MOTOR 2 (board trace or single-output test) | trace: no | ___ / ___ |
| 5 | Driver current rating against two motors per output | no | ___ |
| 6 | Polarity of each group (`HD Wheel Check`) | yes | ___ / ___ |
| 7 | `throttleSign`: stick forward drives the robot forward | yes | ___ |
| 8 | `steeringSign`: right stick swings the nose right | yes | ___ |
| 9 | Deadband, maximum PWM, optional minimum PWM and acceleration limit | yes | ___ |
| 10 | Automatic-turn direction for L3, L4, L1, then turn calibration | yes | ___ |
| 11 | RT, LT and controller-off stop the wheels, on blocks | yes | ___ |

## V4.2 note — automatic-turn direction

The mirrored-turn risk described above is closed in software: `turns.rightTurnSign`
states which way the L-labelled pair runs for a robot-right turn, and the build refuses
to enable turns without it. It is unset.

| Value | Record |
|---|---|
| `rightTurnSign` (+1 L-labelled pair forward, −1 backward, for the robot to turn right) | ___ |
| Evidence (which side the L-labelled wheels are on; pattern seen on blocks) | ___ |
| L3 turns robot right ___ / L4 turns robot left ___ / L1 direction ___ | |

## Factory MOTOR TEST results (performed by the team, wheels raised, real controller)

| Test | Wheels that moved | Direction (team's convention) | Wheels that stayed still |
|---|---|---|---|
| M1 at +25 | LF and LB | backward | RF, RB |
| M2 at +25 | RF and RB | forward | LF, LB |

**Established by this test**
- Two-group arrangement: each test entry drives exactly one same-side pair and nothing
  else. With the earlier driving observations and the SDK's two-output factory code,
  the paired-socket architecture is physically confirmed.
- Screen M1 drives the group holding LF/LB (board MOTOR 1, by inference from layout);
  screen M2 drives the group holding RF/RB (board MOTOR 2, seen).

**What the SDK source evidence shows about M1 / M2**
From disassembly of `NRLComms.o` in the SDK 1.5.0 library (the robot side of MOTOR TEST):
- the test owns two motor slots; slot 0 is built with DIR/PWM 5/10 (`MOTOR_L`), slot 1
  with 11/12 (`MOTOR_R`); neither is flipped;
- commands are held per slot and applied slot 0 first, then slot 1; speeds are reported
  back to the controller in the same order.

Not available: the controller's source. The step "screen label M1 sends slot 0" lives in
the controller binary and cannot be read from this kit. It is the natural numbering,
and the servo test offers the same pattern to cross-check (slot 0 there is GPIO 39,
`SERVO_1`), but it is corroboration, not proof.

| Link | Status |
|---|---|
| slot 0 = `MOTOR_L` (5/10), slot 1 = `MOTOR_R` (11/12), unflipped | proven from SDK library |
| M1 moves LF/LB only, M2 moves RF/RB only | proven by the team's test |
| screen M1 = slot 0, screen M2 = slot 1 | **corroborated, not proven** |
| therefore LF/LB on `MOTOR_L`, RF/RB on `MOTOR_R` | **provisional** — awaiting team acceptance or a direct check |

Direct checks that would close the last link (any one): the controller's SERVO TEST
entry "S1" moves the servo plugged into the port marked SERVO 1 ___; or a continuity
trace from GPIO 10 to one group's driver input ___.

**Polarity, as observed (raw, unflipped positive command)**

| Group | Raw + command gives | `sign` in V4.2 |
|---|---|---|
| LF / LB (screen M1) | wheels rotate "backward" | unset |
| RF / RB (screen M2) | wheels rotate "forward" | unset |

`sign` is the command sign that drives the ROBOT forward. The test's "+" is not assumed
to mean forward travel, and the team's "forward/backward" for a wheel has not been tied
to robot travel. One recorded fact turns the table into signs:

| Check | Record |
|---|---|
| The wheel rotation the team calls "forward" moves the robot forward (toward its front)? | ___ |

If yes: LF/LB pair `sign = -1`, RF/RB pair `sign = +1`. If no: the opposite. Not entered.

No configuration value was changed. Every output remains disabled.

## Routing entered in the configuration (team decision, 12 November 2026)

| Wheel | Socket | Group | SDK output | DIR / PWM | Routing | Polarity |
|---|---|---|---|---|---|---|
| LF `wheel[FrontLeft]` | 3 | screen M1 | `MOTOR_L` | 5 / 10 | entered | unset |
| LB `wheel[RearLeft]` | 4 | screen M1 | `MOTOR_L` | 5 / 10 | entered | unset |
| RF `wheel[FrontRight]` | 2 | screen M2, board MOTOR 2 | `MOTOR_R` | 11 / 12 | entered | unset |
| RB `wheel[RearRight]` | 1 | screen M2, board MOTOR 2 | `MOTOR_R` | 11 / 12 | entered | unset |

Two logical motor outputs. The screen-label link is accepted by the team, not proven
from source; the first wheel check on blocks confirms it (if the opposite pair moves,
swap the two channels). `drive.enabled` and `wheelCheckEnabled` remain false.

Still needed before the drive can be enabled:

| # | Value | Record |
|---|---|---|
| 1 | "Forward" wheel rotation moves the robot toward its front? (fixes both signs) | ___ |
| 2 | Driver current rating vs two motors per output (`driverCapacityVerified`) | ___ |
| 3 | `maxPwm` starting ceiling and `deadband` | ___ / ___ |
| 4 | `throttleSign`, `steeringSign` (on blocks, after upload) | ___ / ___ |

## Provisional polarities entered (12 November 2026)

Team confirmation: a wheel rotating "forward" means its rotation would move the robot
toward its front when on the ground.

| Group | Factory MOTOR TEST | Entered `sign` | Status |
|---|---|---|---|
| LF / LB on `MOTOR_L` | M1 at +25: backward | −1 | provisional |
| RF / RB on `MOTOR_R` | M2 at +25: forward | +1 | provisional |

Derived from the factory test and entered together with the provisional channel mapping
(screen M1 = `MOTOR_L`, M2 = `MOTOR_R`). Both test outputs are unflipped in the SDK
library, so each sign belongs to its wheel pair whichever of the two outputs feeds it;
what the mapping decides is the channel, and that is what the wheel check confirms.
Neither has been seen on V4.2 itself.

Not entered, not invented: PWM ceiling, deadband, current rating, throttle and steering
signs. `drive.enabled` and `wheelCheckEnabled` remain false.

## Requirements for a controlled wheel-check build

The wheel check spins one group at a time while LB alone is held, at one fixed low PWM.
It needs no stick signs, no deadband and no drive enable.

| # | Requirement | Record |
|---|---|---|
| 1 | `wheelCheckPwm`, the team's choice between 11 and 64 of 255 | ___ |
| 2 | Motor-driver chip part number / current rating (the factory test already ran each group at +25) | ___ |
| 3 | Recovery decision for the factory firmware accepted | ___ |
| 4 | Real team number, name and field channel stamped | ___ |
| 5 | Robot on blocks, wheels clear; controller available; explicit go-ahead to upload | ___ |

Expected result on blocks, to be recorded:

| Selected wheel | Pair that should move | Expected direction with LB held | Seen |
|---|---|---|---|
| FrontLeft or RearLeft | LF and LB only | backward (raw + command) | ___ |
| FrontRight or RearRight | RF and RB only | forward (raw + command) | ___ |

If the opposite pair moves, swap the two channels. If a direction differs, flip that
group's sign. Also check on blocks: release LB stops the wheels; RT, LT and switching
the controller off each stop them.

## Radio identity (12 November 2026)

| Item | Value | Status |
|---|---|---|
| Radio channel | 1 | verified: the handheld displays RADIO CHANNEL 1 |
| Robot name | HYPERDRIVE | stamped in `platformio.ini` and `nrl-project.json` |
| Team number | 001 | PLACEHOLDER, not our registered number ___ |

The SDK headers describe the team number as identity for the hub display and status
only; the project wizard also uses it to pick a default channel, which is already set
explicitly here. Pairing is by controller address and a 4-digit code stored on the hub.
Remaining: registered team number ___; field channel assigned at the event (1, 6 or
11) ___; pairing with this handheld confirmed after any upload ___.

## HD Wheel Check prepared (12 November 2026)

**What "+25" on the factory MOTOR TEST was.** Traced in the SDK 1.5.0 library
(`NRLComms.o`): the test value is copied from the radio packet as a 16-bit number,
with no multiply, divide or float conversion anywhere in the receive handler, then
clamped to ±255 and passed to `HexaDCMotor::setSpeed`, whose range is ±255 (8-bit PWM,
20 kHz). So on the robot side the unit is PWM counts out of 255. Whether the
controller sends the number it displays or converts a percentage first is inside the
controller's binary and cannot be read.

| If the screen shows… | PWM applied at "+25" |
|---|---|
| raw PWM counts | 25 of 255 |
| percent | about 64 of 255 |

**Wheel-check value entered: 25 of 255.** It does not exceed what each group has
already run at under the factory test in either reading. No motor or driver rating is
documented, so nothing higher is justified by evidence.

| Observation on blocks | Meaning | Record |
|---|---|---|
| Wheels turn at 25, similar speed to the factory "+25" | screen shows raw PWM | ___ |
| Wheels hum or do not turn at 25 | screen shows percent; raise in small steps, never above 64 | ___ |

`wheelCheckEnabled = true`, `wheelCheckPwm = 25`. The drive, claw, crane, payload,
pickups, turns and stow remain disabled; the match program still moves nothing.

## Physical calibration results (factory controller, reported by the team, 12 November 2026)

These supersede earlier entries in this worksheet where they differ.

| Item | Measured | Entered in configuration |
|---|---|---|
| MOTOR TEST range / step | −255 to +255, steps of 25 | — |
| M1 at +25 | LF and LB rotate smoothly backward | `MOTOR_L`, sign −1 (provisional) |
| M2 at +25 | RF and RB rotate smoothly forward | `MOTOR_R`, sign +1 (provisional) |
| Motors | no abnormal behaviour | — |
| Claw open / closed | 85° / 150°, smooth | 85 / 150, travel limited to 85–150 |
| Claw at 160° | extra tightness | cannot be configured |
| Crane raised / lowered | 30° / 120°, full travel, no noise or binding | 30 / 120, travel limited to 30–120, clearance verified |
| Servo pulse range | — | 500–2500 µs, the factory SERVO TEST's own range in the SDK library |
| Radio | channel 1 | channel 1, verified |

Earlier values no longer used: claw closed 105°, crane raised 50°, crane requested 125°.

**MOTOR TEST units.** The screen's range (±255) equals the motor command range, and the
SDK library passes the received value to the motor with no scaling. Both point to the
screen number being the PWM command (25 = PWM 25 of 255). The controller's own code
cannot be read, so this is strong evidence, not proof. The wheel check stays at 25.

**Status: software-ready vs physically verified**

| Function | Configured | Host-tested | Enabled in image | Seen on V4.2 on the robot |
|---|---|---|---|---|
| Wheel check (one group, PWM 25, LB held) | yes | yes | **yes** | no |
| Manual driving | routing and polarity only | yes (synthetic values) | no | no |
| Claw (LB) | endpoints, limits, pulses | yes (synthetic timing) | no | no |
| Crane (RB) | endpoints, limits, pulses, clearance | yes (synthetic timing) | no | no |
| Payload, pickups, turns, stow | no | yes (synthetic values) | no | no |

**Still missing before each can be enabled**

| Function | Missing value | How to get it |
|---|---|---|
| Driving | `maxPwm` | highest MOTOR TEST value run without trouble ___ |
| Driving | motor-driver current rating | chip part number off the board ___ |
| Driving | `deadband`, `throttleSign`, `steeringSign` | on blocks after upload ___ |
| Claw | travel-and-settle time 85° ↔ 150° | time it on SERVO TEST ___ ms |
| Claw | start pose (open or closed) | team decision ___ |
| Crane | travel-and-settle time 30° ↔ 120°, with the claw's load | time it on SERVO TEST ___ ms |
| Crane | start pose (raised or lowered) | team decision ___ |
| Both servos | what each does when its signal stops, loaded, with a catch | leave SERVO TEST / power off and watch ___ |

## Latest servo calibration (factory SERVO TEST, reported by the team, 12 November 2026)

Supersedes the previous section where it differs (claw closed is now 120°, not 150°).

| Item | Measured | Entered |
|---|---|---|
| Claw open / closed | 85° / 120°; at 120° it holds the CORE without slipping | 85 / 120, travel limited to 85–120 |
| Claw 150°, 160° | superseded / extra tightness | cannot be configured |
| Claw closing / opening time | 2.13 s / 2.08 s | wait 2700 ms |
| Crane raised / lowered | 30° / 120°, smooth, no binding | 30 / 120, travel limited to 30–120 |
| Crane lowering / raising time | 4.66 s / 4.80 s | wait 6000 ms |
| Start poses | claw open 85°, crane raised 30° | `grip.start = 85`, `crane.start = 30` |

**About the times.** They were taken while stepping the factory test with buttons, so
they are upper bounds on travel, not servo speeds. The entered waits are the slower
direction plus 25 %, rounded up; the 25 % is a margin chosen in software.

**Two behaviours of V4.2 that differ from the factory test — check before enabling**

1. *Moves are single commands.* The factory test walked each servo there in steps over
   seconds. V4.2 sends the end angle at once, so the servo travels at its own full
   speed. The crane's 90° swing under load has not been seen at that speed.
2. *Waits block other actions.* After LB the coordinator accepts nothing else for
   2.7 s, and after RB for 6 s; pushing a stick in that window cancels and restarts the
   wait. With the stepped timings above the robot cannot drive for 6 s after each crane
   toggle. A direct timing of one full-speed move (video is enough) would let these be
   shortened honestly: claw ___ ms, crane ___ ms.

**Start-up.** On the first neutral packet after START, V4.2 commands the start poses
(85°, 30°). If the claw is open and the crane raised when START is pressed, nothing
visibly moves. The servos stay disabled, so no start-up command exists in the current
image; staging and this behaviour are to be validated when they are first enabled.

**Still missing**

| Function | Missing | How |
|---|---|---|
| Claw and crane | behaviour when the signal stops, loaded, with a catch (`safety.responsesVerified`) | leave SERVO TEST or power off and watch ___ |
| Driving | `maxPwm` | highest MOTOR TEST value run without trouble ___ |
| Driving | motor-driver current rating | chip part number ___ |
| Driving | `deadband`, `throttleSign`, `steeringSign` | on blocks after upload ___ |

## Manual-drive candidate (13 November 2026)

New evidence: both motor groups ran at +25 and +50 on the factory MOTOR TEST with no
abnormal behaviour. The motor-driver chip could not be read through the cover and is
not documented in the SDK kit.

| Setting | Value | Basis | Seen on V4.2 |
|---|---|---|---|
| `drive.enabled` | true | — | no |
| `maxPwm` | 50 of 255 | highest value physically run on both groups | no |
| `driverCapacityVerified` | true, for outputs up to 50 only | that same test | — |
| `steeringSign` | −1 | factory app: right stick = RF/RB forward, LF/LB backward, nose right; assumes rightX positive to the right | no |
| `throttleSign` | +1 | SDK convention only (leftY positive forward) | **no — check first** |
| `deadband` | 0.10 | software starting value, not a measurement | no |
| Wheel check | off | never built together with the drive | — |

Claw, crane, payload, pickups, turns and stow remain disabled.

**First run, on blocks, in this order**

| # | Do | Expect | Seen |
|---|---|---|---|
| 1 | START with sticks centred, nothing pressed | no wheel moves | ___ |
| 2 | Left stick gently forward | all four wheels roll forward (slowly; ceiling is 50) | ___ |
| 3 | Left stick back | all four roll backward | ___ |
| 4 | Right stick right | RF/RB forward, LF/LB backward | ___ |
| 5 | Flip RT while driving | wheels stop and stay stopped | ___ |
| 6 | Restart; flip LT while driving | wheels stop, program exits | ___ |
| 7 | Restart; switch the controller off while driving | wheels stop | ___ |

If step 2 runs backward: `throttleSign = -1`. If only one side is wrong: that group's
`sign`. If step 4 is mirrored: `steeringSign = +1`. If the wrong pair responds to
steering in a way that suggests swapped outputs: swap the two channels.

## Loaded CORE test and basic-control candidate (13 November 2026)

**Team's result (factory SERVO TEST, CORE in the claw)**
- Claw closed at 120° held the CORE securely.
- Crane raised at 30° stayed raised.
- After leaving SERVO TEST (left toggle and L1) the crane stayed raised and the claw
  kept the CORE. No unexpected movement.
- This was an exit-from-test observation. It was not a radio-loss or power-loss test.

**What that exit does, from the SDK library.** Leaving the servo test runs the SDK's
release routine, which detaches the servos (signal removed). So the observation is the
signal-removed case under load, the same state the framework produces at STOP.

**How V4.2 uses it**

| Event | V4.2 response | Supported by |
|---|---|---|
| Radio lost | wheels stop; servos keep their last command (Hold) | held state seen during the test |
| RT, no stow route | wheels stop; servos Hold; latched until restart | same |
| Fault | wheels stop; servos Hold; needs a new START | same |
| LT / STOP | wheels stop; framework detaches servos | the exit observation |
| Power lost | nothing software can do | **not tested** |

Not proven by this test: an actual radio loss, a power loss, or holding over a longer
time or under shock. Those are the first checks when the image is commissioned.

**Enabled in this candidate**

| Function | Setting |
|---|---|
| Driving | two groups, ceiling PWM 50, deadband 0.10 |
| LB claw | 85° ↔ 120°, limited to 85–120, 2.7 s wait |
| RB crane | 30° ↔ 120°, limited to 30–120, 6 s wait |
| LT, RT, link-loss stop | active |

Disabled: payload, R2/R3 pickups, automatic turns, safe stow, all seven round buttons,
wheel check.

**Start-up.** INIT powers nothing. After START, the first packet with sticks centred
and no button pressed powers the claw at 85° and the crane at 30°, then nothing is
accepted for 6 s. The software cannot sense where the mechanisms are: if the claw is
closed or the crane lowered at that moment, they will move to open and raised. Always
stage open and raised, with no CORE in the claw, before pressing START.

**First run with servos, after the driving checks, catch in place**

| # | Do | Expect | Seen |
|---|---|---|---|
| 1 | Stage claw open, crane raised, no CORE. START | no visible servo movement | ___ |
| 2 | Wait 6 s, press LB | claw closes to 120°; nothing else accepted for 2.7 s | ___ |
| 3 | Press LB | claw opens to 85° | ___ |
| 4 | Press RB, unloaded | crane swings to 120° in one move, at servo speed | ___ |
| 5 | Press RB | crane returns to 30° | ___ |
| 6 | Grip a CORE, raise, switch the controller off | wheels stopped; crane stays up; CORE held | ___ |
| 7 | Controller on, sticks centred | control returns after 6 s | ___ |
| 8 | Grip a CORE, raise, flip LT | crane stays up; CORE held | ___ |
