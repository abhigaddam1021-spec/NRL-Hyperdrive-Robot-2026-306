// Host-only doubles for the SDK calls used by CoreLiftJog.cpp.
// PlatformIO never includes this directory.
#pragma once
#include "BoardPins.h"
#include <cassert>
#include <cstdint>

namespace fixture {
inline uint32_t timeMs, sequence;
inline uint16_t buttons;
inline float axes[4];
inline int pwmByPin[64], failMotorPin = -1;
inline bool stale, upperOpen, lowerOpen, initOK = true;
inline int begins, stops, writes, lastPwm, inputs;
}
constexpr unsigned BTN_X=1, BTN_A=2, BTN_Y=8, BTN_DPAD_RIGHT=16, BTN_DPAD_DOWN=32,
    BTN_DPAD_UP=64, BTN_DPAD_LEFT=128, BTN_RB=256, BTN_LB=512, BTN_LT=1024, BTN_RT=2048;
constexpr int HIGH = 1, INPUT_PULLUP = 2;
inline uint32_t millis() { return fixture::timeMs; }
inline void pinMode(int pin, int mode) {
    assert((pin == DIGITAL_D1 || pin == DIGITAL_D2) && mode == INPUT_PULLUP);
    ++fixture::inputs;
}
inline int digitalRead(int pin) {
    assert(pin == DIGITAL_D1 || pin == DIGITAL_D2);
    return pin == DIGITAL_D1 ? fixture::upperOpen : fixture::lowerOpen;
}
namespace NRLGamepad {
inline void read(float& lx, float& ly, float& rx, float& ry, uint16_t& buttons, uint32_t& seq) {
    lx = fixture::axes[0]; ly = fixture::axes[1];
    rx = fixture::axes[2]; ry = fixture::axes[3];
    buttons = fixture::buttons;
    seq = fixture::sequence;
}
}
struct {
    bool low=false;
    bool isBatteryLow() const { return low; }
} inline power;
namespace NRLComms {
inline bool headingReady = true;
inline float heading;
inline bool isHeadingReady() { return headingReady; }
inline float getHeadingFree() { return heading; }
inline bool isInputStale() { return fixture::stale; }
}
struct HexaDCMotor {
    struct Pins { uint8_t dir, pwm; };
    Pins pins;
    explicit HexaDCMotor(Pins p) : pins(p) {}
    bool begin() { ++fixture::begins; return fixture::initOK && pins.dir != fixture::failMotorPin; }
    void stop() { ++fixture::stops; fixture::pwmByPin[pins.dir] = fixture::lastPwm = 0; }
    void setSpeed(int pwm) { ++fixture::writes; fixture::pwmByPin[pins.dir] = fixture::lastPwm = pwm; }
};
struct NRLOpMode {
    struct {
        void addData(const char*, const char*) {}
        void addData(const char*, float) {}
    } telemetry;
    virtual void init() {}
    virtual void start() {}
    virtual void loop() {}
    virtual void stop() {}
};
#define REGISTER_OPMODE(type, name, mode) static_assert(sizeof(type) > 0)
