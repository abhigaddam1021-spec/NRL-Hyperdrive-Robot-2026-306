# Build report — HYPERDRIVE V4.2 — 12 November 2026

| Item | Result |
|---|---|
| Tool | PlatformIO Core 6.2.0 at `/private/tmp/nrl-corelift-tools/bin/pio` (verified present), core dir `/private/tmp/nrl-corelift-pio` |
| Clean | `pio run -d RobotFirmware -e robot -t clean` — exit 0 |
| Build | `pio run -d RobotFirmware -e robot` — exit 0 |
| Target | Espressif32 6.13.0, board esp32-s3-devkitc-1, ESP32-S3 240 MHz, Arduino framework 3.20017, toolchain xtensa-esp32s3 8.4.0, esptool 4.11.0 |
| SDK | NRL SDK 1.5.0 prebuilt libraries, unmodified; ABI marker linked |
| Warnings / errors | 0 / 0 |
| RAM | 56,036 of 327,680 bytes (17.1%) |
| Flash | 837,061 of 3,145,728 bytes (26.6%) |
| ELF | `firmware/firmware.elf` — 32-bit Xtensa executable |
| Image | `firmware/NOT-READY-FOR-HARDWARE-UPLOAD.bin` |
| Image SHA-256 | `c0ec62d4f4678cbc7ea772140e4ed9e8abb11c58d72b62ab58f20cc1d777a267` |
| ELF SHA-256 | `6a1be791700acdd98cdef5b9c22a6ae4660c7e926fb7b9c80f4fff69916b9710` |
| OpModes found in image | Hyperdrive TeleOp, HD Input Monitor, HD Wheel Check, CORE Lift Bench, CORE Servo Lift |
| Identity compiled in | name HYPERDRIVE, channel 1 (verified), team number 001 (placeholder) |
| Hardware | not flashed, not connected to, not tested |

Full output: `build-logs/firmware-build.log`. This image is the basic-control candidate: `Hyperdrive TeleOp` drives the two motor
groups from the sticks capped at PWM 50, with LB claw and RB crane (host test X11).
Payload, pickups, turns, round buttons and the wheel check are off (host test T93).
Upload is not authorised.
