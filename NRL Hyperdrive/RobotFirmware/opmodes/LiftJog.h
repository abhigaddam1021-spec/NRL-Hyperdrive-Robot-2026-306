#pragma once
#include <stdint.h>

// One pulse per neutral-to-pressed transition. Account for unsigned clock wrap.
class LiftJog {
    bool armed = false;
    int direction = 0;
    uint32_t started = 0;

public:
    int update(uint32_t now, bool valid, bool up, bool down, bool top,
               bool bottom, uint32_t pulseMs, int raise, int lower,
               bool freshPacket = true) {
        if (!valid || (top && bottom) || (up && down)) {
            armed = false;
            direction = 0;
            return 0;
        }
        if (!up && !down) {
            // The SDK may clear its snapshot without receiving a packet.
            // Only a real neutral packet can authorize another jog.
            if (freshPacket) armed = true;
            else if (direction != 0) armed = false;
            direction = 0;
            return 0;
        }

        const int requested = up ? 1 : -1;
        if (armed) {
            started = now;
            direction = requested;
            armed = false;
        }
        if (direction != requested || (up && top) || (down && bottom)
            || now - started >= pulseMs) {
            direction = 0;
            return 0;
        }
        return up ? raise : lower;
    }
};
