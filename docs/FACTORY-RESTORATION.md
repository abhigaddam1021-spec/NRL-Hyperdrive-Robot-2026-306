# Factory firmware preservation and restoration requirements

Nothing in this release touched the hub or the handheld.

## What the official kit establishes

- `ControllerFirmware/flash_controller.py` states that the robot's NVS partition
  (0x9000–0xE000) holds the board's licence (namespace `nrl`, key `lic`), that only the
  NRL Flash Station can reissue it, and that a robot with blank NVS boots dark
  (blank OLED, no LED, `ST:E27` on serial) even after correct firmware is uploaded.
- A standard PlatformIO upload writes the bootloader (0x0), partition table (0x8000),
  boot selector (0xE000) and application (0x10000). It does not write NVS.
- The SDK's example programs and calibration screens are inside
  `libHexaNRL_static.a`, so any firmware built from this kit, including V4, contains
  them. Whether the hub's current application was built from this same SDK version is
  not known.
- `controller_merged.bin` is controller firmware. It is not a robot restoration image.

## What is not established

- No official procedure for restoring this hub's factory application was found in
  the kit.
- The SDK version and licence key generation on this hub are unknown. If they differ
  from this kit's, a new upload could fail the licence check.
- No restore has been demonstrated on this or a matching hub.

**Result: deployment remains blocked by recovery uncertainty.**

## Ways to close it (team's choice)

1. Ask NRL for the restoration image/procedure for this hub and confirmation that an
   SDK 1.5.0 project build is accepted by its licence.
2. Read the SDK version the hub prints at boot (`[NRL] SDK vX.Y.Z` on USB serial) and
   compare with 1.5.0.
3. A full flash read-back before any upload gives a candidate restoration image. It
   only counts as a restoration route once writing it back has been shown to work,
   ideally on a second hub. Taking it puts the hub into its bootloader, so it is not
   done here.
