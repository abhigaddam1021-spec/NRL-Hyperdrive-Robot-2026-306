# Known limitations and open evidence

1. **Motor routing is the main open question; see `MOTOR-GROUPS.md` for the V4.1 finding.**
   Disassembly of `libHexaNRL_static.a` (SDK 1.5.0) shows every built-in program that
   drives motors creates exactly two `HexaDCMotor` objects; the calibration MOTOR TEST
   path also creates two. `MotorRampDemo` uses DIR/PWM 5/10 and 11/12 (second one
   flipped), i.e. MOTOR_L and MOTOR_R. `BoardPins.h` calls MOTOR_3/4 bench-only.
   So if the factory application turns all four wheels, the four sockets are almost
   certainly driven as two pairs from MOTOR_L and MOTOR_R. This is inference from
   software, not a traced circuit: the configuration stays unrouted. V4 supports both
   outcomes — four separate channels, or proven paired sockets
   (`pairedSocketsVerified`), in which case paired wheels share one polarity in
   software and any single reversed wheel must be fixed at its plug.
1a. Automatic-turn direction is set by `turns.rightTurnSign` (V4.2), separately from
   `steeringSign`. It is unset; turns cannot be enabled until it is observed on blocks.
2. No servo position feedback and no CORE sensor. "Settled" is elapsed calibrated
   time. A completed pickup means the sequence ran, not that a CORE was acquired.
3. The crane's resistance near 100° was reviewed against the code path: angles pass
   unchanged to `HexaServo::setPosition`, port and bounds are validated, and no
   conflicting writer exists. No software defect was found; it remains a mechanical
   question.
4. SDK STOP detaches every servo in the framework after `stop()`. A raised load needs
   mechanical support through STOP. Software cannot change that.
5. A stalled CPU cannot enforce a software deadline; all timeouts run on the 100 Hz tick.
6. Timed turns are approximate by nature. Heading turns need the hub's gyro confirmed.
7. Safe stow needs an external health input; none is fitted or recorded.
8. Team 001 "Bench", channel 1 are placeholders. Pairing is unverified.
9. Button positions are unknown; no factory button-test view was found.
10. `platformio.ini` allows dependency ranges; this build used Espressif32 6.13.0,
    Adafruit NeoPixel 1.15.5, LSM6DS 4.7.4, Unified Sensor 1.1.15, BusIO 1.17.4.
11. The build toolchain lives under `/private/tmp` and may not survive a reboot.
12. Game scoring and Starburst activation are not addressed or claimed.
