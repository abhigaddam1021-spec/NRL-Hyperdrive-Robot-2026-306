# Test report — HYPERDRIVE V4.2 — 12 November 2026

Host tests only. They run the real OpMode source against doubles of the SDK on macOS
(clang, C++17, `-Wall -Wextra -Werror -pedantic`, undefined-behaviour sanitiser).
They prove logic, not hardware.

| Suite | Result |
|---|---|
| `hyperdrive_test.cpp` (V2 regression sources in `tests/v2`) | PASS |
| `lift_jog_test.cpp` | PASS |
| `opmode_test.cpp` | PASS |
| `servo_lift_test.cpp` | PASS |
| `v4_test.cpp` — 96 directive cases + 11 extra | PASS, 107 of 107 |

Baseline before any change: the five V3 suites passed on the unmodified copy.
`v3_test.cpp` was replaced by `v4_test.cpp` because link-loss, takeover and
RT-without-stow behaviour changed by design; the V3 file remains in the V3 project.

During development one case (T95) failed once because the test left two simulated
fault flags set between sub-cases; the test was corrected with no firmware change.
The suite exposed no firmware defect.

Mutation check (scratch copies, not shipped): shortening the dwell by 1 ms failed T42;
disabling link-loss handling failed T13; letting stow move a closed claw failed T72.

V4.1: one existing assertion in T10 changed its expected reason (a board flagged as
paired but configured on four outputs is now rejected as a group error), and X05 was
added for the motor groups.

V4.2: X06 added for turn direction. No existing expectation changed.

Routing entry (12 Nov): X05 now also checks the delivered routing and that the drive
cannot be enabled without polarity.

Polarity entry (12 Nov): X05 checks the provisional signs and that neither the drive
nor the wheel check can be enabled as delivered.

Radio entry (12 Nov): X07 added for the channel and team placeholder. Final run: the
clean rebuild reproduced the archived image byte for byte.

Wheel-check preparation (12 Nov): X08 added for the delivered wheel check; T93 now
covers the match program and input monitor only.

Servo calibration (12 Nov): synthetic test robot moved to the measured endpoints (claw
85/150, crane 30/120); X09 added. One stale angle list in T20 was corrected.

Claw 120 and timings (12 Nov): test robot moved to claw 85/120; X09 updated; X10 added,
running the claw and crane with the recorded values (the untested response flag is
forced in that host-only case).

Manual-drive candidate (13 Nov): X11 added for the delivered image; T93 and X08 adjusted
because the delivered image now drives and the wheel check is off.

Basic-control candidate (13 Nov): X11 rewritten for drive + claw + crane in the delivered
image; T93 and X09 updated. One stale assertion (crane expected disabled) corrected.

## Cases executed

- T01 PASS  neutral input gives zero output
- T02 PASS  forward
- T03 PASS  reverse
- T04 PASS  left steering
- T05 PASS  right steering
- T06 PASS  combined throttle and steering
- T07 PASS  command normalisation
- T08 PASS  independent wheel polarities
- T09 PASS  invalid motor routing rejected
- T10 PASS  duplicate motor channels rejected; proven paired sockets allowed
- T11 PASS  PWM saturation
- T12 PASS  deadband
- T13 PASS  controller disconnect stops the drive
- T14 PASS  STOP while driving
- T15 PASS  RT while driving
- T16 PASS  initialisation failure at each motor
- T17 PASS  no output from disabled channels
- T18 PASS  claw open command
- T19 PASS  claw close command
- T20 PASS  claw toggles once per press
- T21 PASS  held LB does not repeat
- T22 PASS  LB re-pressed before settling does not re-toggle
- T23 PASS  LB during automation cancels instead of fighting it
- T24 PASS  closed-claw command kept through the lift
- T25 PASS  crane raise
- T26 PASS  crane lower
- T27 PASS  held RB does not repeat
- T28 PASS  missing lowered endpoint rejected
- T29 PASS  invalid travel limits rejected
- T30 PASS  unverified clearance rejected
- T31 PASS  crane initialisation failure
- T32 PASS  settling interval gates the next command
- T33 PASS  R1 raises payload
- T34 PASS  R4 lowers payload
- T35 PASS  opposing payload commands rejected
- T36 PASS  invalid payload port rejected
- T37 PASS  missing payload endpoints rejected
- T38 PASS  crane/payload collision interlock
- T39 PASS  R2 valid start
- T40 PASS  R2 invalid starts rejected
- T41 PASS  R2 lowering stage waits the calibrated interval
- T42 PASS  R2 dwell: claw still open 499 ms after lowering completed
- T43 PASS  R2 dwell: claw closes at 500 ms
- T44 PASS  R2 claw closes with crane still lowered
- T45 PASS  R2 grip settling before the lift
- T46 PASS  R2 raising stage and completion
- T47 PASS  R2 cancellation at every stage (RT and stick)
- T48 PASS  R2 total timeout
- T49 PASS  controller loss during R2
- T50 PASS  STOP during R2 at every stage
- T51 PASS  R3 valid lowered start
- T52 PASS  R3 rejected with crane raised
- T53 PASS  R3 rejected while crane state is unsettled
- T54 PASS  R3 closes the claw at once, no dwell
- T55 PASS  R3 grip settling
- T56 PASS  R3 raises and completes
- T57 PASS  R3 cancellation keeps the claw closed
- T58 PASS  R3 timeout
- T59 PASS  L1 180-degree request
- T60 PASS  L3 right 90 request
- T61 PASS  L4 left 90 request uses its own calibration
- T62 PASS  missing turn calibration
- T63 PASS  heading wraparound
- T64 PASS  heading loss stops the turn
- T65 PASS  turn timeout
- T66 PASS  manual override cancels a turn
- T67 PASS  RT cancels a turn
- T68 PASS  STOP during a turn
- T69 PASS  RT while idle stows crane, then payload, then claw
- T70 PASS  RT during manual driving
- T71 PASS  RT during R2
- T72 PASS  RT during R3 leaves the closed claw alone
- T73 PASS  RT during turning
- T74 PASS  RT during payload movement retargets the payload first
- T75 PASS  RT held continuously does not retrigger
- T76 PASS  repeated RT
- T77 PASS  missing stow calibration: inhibit, no servo travel
- T78 PASS  invalid stow targets rejected
- T79 PASS  inputs during stow cannot interrupt or redirect it
- T80 PASS  controller loss during stow
- T81 PASS  STOP during stow
- T82 PASS  neutral re-arming after stow
- T83 PASS  no unintended claw release
- T84 PASS  no automatic restart after interruption
- T85 PASS  duplicate and unsupported button masks rejected
- T86 PASS  unassigned and unverified buttons never act
- T87 PASS  invalid configurations rejected with a specific reason
- T88 PASS  timer rollover
- T89 PASS  stale controller packets
- T90 PASS  conflicting actions rejected
- T91 PASS  failed initialisation enables nothing
- T92 PASS  partial initialisation is rolled back
- T93 PASS  delivered program: payload, pickups, turns and round buttons never act
- T94 PASS  recovery from link loss, repeatedly, only via fresh neutral
- T95 PASS  safety priority ordering within one tick
- T96 PASS  safe shutdown from every state
- X01 PASS  manual acceleration limit; safety stops are immediate
- X02 PASS  measured PWM floor mapping
- X03 PASS  wheel check spins one routed wheel under a dead-man
- X05 PASS  board motor groups: two paired outputs or four separate ones, never a mix
- X06 PASS  turn direction follows the robot, not the wheel labels
- X07 PASS  radio identity: channel 1 verified, team number still a placeholder
- X08 PASS  wheel-check variant of the delivered routing: one group at PWM 25 under a dead-man
- X09 PASS  measured servo calibration: poses, limits, waits and responses as recorded
- X10 PASS  claw and crane with the recorded values: staged start, toggles, waits, holds on link loss
- X11 PASS  DELIVERED IMAGE: drive capped at 50 plus LB claw and RB crane, all stops intact
- X04 PASS  confirmed socket order is fixed in the configuration
