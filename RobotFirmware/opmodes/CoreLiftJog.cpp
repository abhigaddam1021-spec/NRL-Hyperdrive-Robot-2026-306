#include "NRL.h"
#include "LiftJog.h"

namespace {
// Unloaded DC bench fixture ONLY. Fill from measured wiring and controls.
// Both switches must be normally closed to ground, opening at the limit.
// An unplugged switch then inhibits travel toward its end.
struct BenchConfig {
    static constexpr bool enableBench = false;
    static constexpr int dirPin = -1, pwmPin = -1;
    static constexpr int upperPin = -1, lowerPin = -1;
    static constexpr unsigned upButton = 0, downButton = 0;
    static constexpr int raiseCommand = 0, lowerCommand = 0;
    static constexpr uint32_t pulseMs = 0; // measured jog, 100 ms absolute ceiling
};

constexpr bool limitPin(int pin) {
    return pin == DIGITAL_D1 || pin == DIGITAL_D2 || pin == DIGITAL_D3;
}
constexpr bool motorPair(int dirPin, int pwmPin) {
    return
    (dirPin == MOTOR_L_DIR && pwmPin == MOTOR_L_PWM) ||
    (dirPin == MOTOR_R_DIR && pwmPin == MOTOR_R_PWM) ||
    (dirPin == MOTOR_3_DIR && pwmPin == MOTOR_3_PWM) ||
    (dirPin == MOTOR_4_DIR && pwmPin == MOTOR_4_PWM);
}
constexpr unsigned momentary = BTN_X | BTN_A | BTN_Y | BTN_RB | BTN_LB |
    BTN_DPAD_LEFT | BTN_DPAD_RIGHT | BTN_DPAD_UP | BTN_DPAD_DOWN;
constexpr bool button(unsigned mask) {
    return mask && !(mask & (mask - 1)) && (mask & momentary);
}
constexpr bool benchSpeed(int speed) {
    return (speed >= 11 && speed <= 64) || (speed <= -11 && speed >= -64);
}
template<class C>
class BenchOpMode : public NRLOpMode {
    static_assert(!C::enableBench || (motorPair(C::dirPin, C::pwmPin) && limitPin(C::upperPin) && limitPin(C::lowerPin)
        && C::upperPin != C::lowerPin && button(C::upButton) && button(C::downButton)
        && C::upButton != C::downButton && benchSpeed(C::raiseCommand) && benchSpeed(C::lowerCommand)
        && (C::raiseCommand > 0) != (C::lowerCommand > 0)
        && C::pulseMs > 0 && C::pulseMs <= 100), "Incomplete or invalid lift bench configuration");
    HexaDCMotor motor{{static_cast<uint8_t>(C::dirPin), static_cast<uint8_t>(C::pwmPin)}};
    bool running = false;
    LiftJog jog;
    bool ready = false;
    uint32_t sequence = 0;
    uint32_t lastPacket = 0;
    bool live = false;

    uint32_t sample(uint16_t& buttons) {
        float lx, ly, rx, ry;
        uint32_t seq;
        NRLGamepad::read(lx, ly, rx, ry, buttons, seq);
        return seq;
    }

public:
    void init() override {
        if constexpr (C::enableBench) {
            pinMode(C::upperPin, INPUT_PULLUP);
            pinMode(C::lowerPin, INPUT_PULLUP);
            ready = motor.begin();
            if (ready) motor.stop();
        }
        telemetry.addData("lift", ready ? "ready" : (C::enableBench ? "init failed" : "disabled"));
    }

    void start() override {
        uint16_t buttons;
        sequence = sample(buttons);
        live = false;
        running = true;
        jog = LiftJog{};
    }

    void loop() override {
        if (!running) return;
        uint16_t buttons;
        const uint32_t seq = sample(buttons);
        const uint32_t now = millis();
        // A delayed tick or stale packet stream requires a fresh neutral frame.
        const bool stale = NRLComms::isInputStale();
        if (now - lastPacket >= 50 || stale) {
            live = false;
            jog = LiftJog{};
        }
        const bool newPacket = seq != sequence;
        if (newPacket) {
            sequence = seq;
            lastPacket = now;
            live = !stale;
        }
        const bool valid = live && !(buttons & ~(momentary | BTN_LT | BTN_RT));
        telemetry.addData("buttons", valid ? static_cast<float>(buttons) : -1.0f);
        if (!ready) return;
        const bool top = digitalRead(C::upperPin) == HIGH;
        const bool bottom = digitalRead(C::lowerPin) == HIGH;
        telemetry.addData("top open", static_cast<float>(top));
        telemetry.addData("bottom open", static_cast<float>(bottom));
        const int speed = jog.update(now, valid, buttons & C::upButton, buttons & C::downButton,
            top, bottom, C::pulseMs, C::raiseCommand, C::lowerCommand, newPacket);
        motor.setSpeed(speed);
        telemetry.addData("lift PWM", static_cast<float>(speed));
    }

    void stop() override {
        if (ready) motor.stop(); // SDK coast stop; fixture must support the load
        ready = false;
        running = false;
    }
};
using CoreLiftJog = BenchOpMode<BenchConfig>;
REGISTER_OPMODE(CoreLiftJog, "CORE Lift Bench", TELEOP);
}
