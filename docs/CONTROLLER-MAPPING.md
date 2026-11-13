# Controller mapping worksheet — SDK 1.5.0

The table lives in `ControllerMap` in `RobotFirmware/opmodes/HyperdriveConfig.h`. A
binding acts only when it has a mask **and** `Status::Verified`.

| Position | Action | SDK mask | Status | Evidence |
|---|---|---|---|---|
| R1 | raise payload | ___ | unassigned | ___ |
| R2 | automatic CORE collection | ___ | unassigned | ___ |
| R3 | quick pickup and lift | ___ | unassigned | ___ |
| R4 | lower payload | ___ | unassigned | ___ |
| L1 | rotate 180° | ___ | unassigned | ___ |
| L3 | rotate 90° right | ___ | unassigned | ___ |
| L4 | rotate 90° left | ___ | unassigned | ___ |
| LB | toggle claw | 0x200 | from `NRLGamepad.h` | confirm on the handheld |
| RB | toggle crane | 0x100 | from `NRLGamepad.h` | confirm on the handheld |
| LT | STOP / exit | 0x400 | SDK | unchanged |
| RT | cancel / stow | 0x800 | SDK | — |

Candidates for the seven round buttons: X 0x001, A 0x002, Y 0x008, Right 0x010,
Down 0x020, Up 0x040, Left 0x080. There is no B bit.

## What was searched

The kit contains no controller schematic, button-scan source, GPIO matrix or labelled
diagram. The controller ships as a hardened binary. Its readable screen names are
IMU TEST, LINE TEST, MOTOR TEST and SERVO TEST; no button-test screen was found. The
mask definitions alone do not say where a button sits, so all seven stay unassigned.

## Discovery

- Without replacing the factory application: only an official labelled diagram for
  this handheld revision from NRL would establish the layout. None is in the kit.
- After an authorised upload: **HD Input Monitor** shows the raw mask and moves
  nothing. Neutral must read 0x000. Press one position, record one bit, release back
  to 0x000. Do all seven, then repeat in reverse order. A duplicate, a multi-bit value
  or a non-zero neutral leaves that position unverified.
