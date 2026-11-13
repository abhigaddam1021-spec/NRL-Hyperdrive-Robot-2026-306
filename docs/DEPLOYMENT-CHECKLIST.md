# Deployment checklist — HYPERDRIVE V4.2 (state of 12 November 2026)

Upload is **not authorised**. The delivered image is the basic-control candidate:
stick driving capped at PWM 50, LB claw, RB crane; everything else off. First-run
steps are in the last two sections of `HARDWARE-WORKSHEET.md`; Gates 1 to 4 below are
folded into those runs. Stage the claw open and the crane raised before every START.

## Already configured

- [x] Name HYPERDRIVE; radio channel 1 (matches the handheld's display)
- [x] Two-group drivetrain: LF/LB on `MOTOR_L`, RF/RB on `MOTOR_R` (provisional mapping)
- [x] Polarity LF/LB −1, RF/RB +1 (provisional, from the factory MOTOR TEST)
- [x] Claw 85° open / 120° closed on `SERVO_1`; crane 30° raised / 120° lowered on `SERVO_2`;
      travel limits, 500–2500 µs pulses, start poses (open, raised), waits (2.7 s, 6 s)
      and crane clearance entered (outputs disabled)
- [x] LT STOP, RT cancel, link-loss stop: built into the coordinator, host-tested

## Gate 0 — before any upload

- [ ] Recovery route for the factory application accepted by the team. No supported
      restoration procedure exists in the kit (see `FACTORY-RESTORATION.md`).
- [ ] Team's explicit go-ahead to replace the factory application.
- [ ] Registered team number entered, or the 001 placeholder knowingly accepted.

## Gate 1 — wheel-check build (first upload)

- [x] `wheelCheckPwm = 25`, `wheelCheckEnabled = true`: no more than the factory test
      already applied (see the worksheet). Tests and clean build done.
- [ ] Motor-driver chip part number read off the board (for the record and for Gate 2).
- [ ] Robot on blocks, all four wheels clear; battery charged; servos need no staging
      (none is powered by this image).

Then upload with
`pio run -d RobotFirmware -e robot -t upload --upload-port PORT`
(never erase flash, never write a merged image at 0x0).

On blocks, record:
- [ ] Hub boots, display alive, pairs with the handheld on channel 1.
- [ ] **HD Input Monitor**: neutral reads 0x000; record the mask of each round button.
- [ ] **HD Wheel Check** (START with no button held; RB steps FrontLeft → RearLeft →
      FrontRight → RearRight). Left wheel selected, LB held: only LF/LB move, backward.
- [ ] Do the wheels turn at 25? Record it: it tells whether the factory screen is raw PWM or percent.
- [ ] Right wheel selected, LB held: only RF/RB move, forward.
- [ ] Releasing LB, RT, LT and switching the controller off each stop the wheels.

## Gate 2 — manual drive build

- [ ] Driver current rating covers two motors per output (`driverCapacityVerified`).
- [ ] `maxPwm` and `deadband` chosen by the team, starting low.
- [ ] `throttleSign`, `steeringSign` checked on blocks: stick forward drives forward,
      right stick swings the nose right.
- [ ] RT, LT, controller-off stop the robot while driving on blocks, then on the floor.

## Gate 3 — claw (independent of the drive)

- [x] Start pose open (85°); wait 2700 ms from the timed 2.13 s.
- [ ] Claw open and staged before START; confirm nothing jumps at START.
- [ ] What the claw does when PWM is removed and on link loss, with a CORE held.

## Gate 4 — crane (after the claw)

- [x] Replacement servo ran 30°–120° with no noise, binding or resistance.
- [x] Start pose raised (30°); wait 6000 ms from the timed 4.80 s.
- [ ] Crane raised and staged before START; first full-speed move watched with a catch in place.
- [ ] Optional: direct timing of one full-speed move, to shorten the 6 s wait; behaviour on PWM removal with a load, with a catch in place.

## Later — optional

Payload (port, endpoints), R2/R3 pickups, automatic turns (`rightTurnSign`), safe stow.
Each needs its round buttons from the Input Monitor record.
