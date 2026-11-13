#include "../RobotFirmware/opmodes/LiftJog.h"
#include <cassert>
#include <cstdio>

int main() {
    LiftJog jog;
    // Synthetic commands and switch values, solely for host tests.
    auto tick = [&](uint32_t t, bool up, bool down, bool top = false,
                    bool bottom = false, bool valid = true) {
        return jog.update(t, valid, up, down, top, bottom, 30, -40, 20);
    };
    assert(tick(0, true, false) == 0); // held across start
    assert(tick(1, false, false) == 0);
    assert(tick(2, true, false) == -40);
    assert(tick(3, true, true) == 0);
    assert(tick(4, false, true) == 0);
    assert(tick(5, false, false) == 0);
    assert(tick(6, false, true) == 20);
    assert(tick(7, true, false) == 0); // reversal requires release
    assert(tick(8, false, false) == 0);
    assert(tick(9, true, false, false, true) == -40);
    assert(tick(10, true, false, true) == 0);
    assert(tick(11, true, false) == 0); // switch bounce
    assert(tick(12, false, false) == 0);
    assert(tick(13, false, true, true) == 20);
    assert(tick(14, false, true, false, true) == 0);
    assert(tick(15, false, false) == 0);
    assert(tick(16, true, false, true, true) == 0);
    assert(tick(17, true, false) == 0);
    assert(tick(18, false, false) == 0);
    assert(tick(19, true, false) == -40);
    assert(tick(20, true, false, false, false, false) == 0);
    assert(tick(21, true, false) == 0);
    assert(tick(22, false, false) == 0);
    assert(tick(23, true, false) == -40);
    assert(tick(24, false, false) == 0); // early release
    assert(tick(UINT32_MAX - 10, true, false) == -40);
    assert(tick(18, true, false) == -40);
    assert(tick(19, true, false) == 0);
    assert(tick(20, true, false) == 0);
    puts("PASS: lift jog state transitions");
}
