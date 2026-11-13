# Release manifest — NRL HYPERDRIVE V4.2 (turn direction)

- Release: NRL-HYPERDRIVE-V4.2-TURN-DIRECTION, packaged 20261113-095951 IST (basic-control candidate: drive at PWM ceiling 50, LB claw, RB crane; all other outputs disabled)
- Classification: SOFTWARE COMPLETE — HARDWARE COMMISSIONING PENDING
- Firmware label: NOT READY FOR HARDWARE UPLOAD
- Base: NRL-HYPERDRIVE-V4.1-MOTOR-GROUPS, preserved unchanged with its archives
- Source changes: `HyperdriveConfig.h` (new `turns.rightTurnSign` and its check), `HyperdriveTeleOp.cpp` (one statement in `beginTurn`), `tests/v4_test.cpp`
- SDK: NRL 1.5.0 (libraries and `RobotMain.ino` unmodified)
- Firmware SHA-256: c0ec62d4f4678cbc7ea772140e4ed9e8abb11c58d72b62ab58f20cc1d777a267
- Per-file checksums: `PROJECT-SHA256SUMS`
- Hardware: nothing flashed, erased, reset or connected to
