#include "NRL.h"
#include "HexaServo.h"

namespace {
// Positional-servo bench only. Record measured values before enabling.
// Angles use the calibrated 0..180 SDK command scale with zero horn offset.
struct ServoBenchConfig {
    static constexpr bool enabled = false;
    static constexpr int port = -1;
    static constexpr unsigned pulseAt0 = 0, pulseAt180 = 0;
    static constexpr int lowered = -1, raised = -1, startAngle = -1;
    static constexpr int stepDeg = 0;
    static constexpr unsigned up = 0, down = 0;
};

constexpr bool liftButton(unsigned bit) {
    constexpr unsigned keys = BTN_X | BTN_A | BTN_Y | BTN_RB | BTN_LB |
        BTN_DPAD_UP | BTN_DPAD_DOWN | BTN_DPAD_LEFT | BTN_DPAD_RIGHT;
    return (bit & keys) && !(bit & (bit - 1));
}

template<class C>
class ServoLiftBench : public NRLOpMode {
    static constexpr int low = C::lowered < C::raised ? C::lowered : C::raised;
    static constexpr int high = C::lowered > C::raised ? C::lowered : C::raised;
    static_assert(!C::enabled || (
        (C::port == SERVO_1 || C::port == SERVO_2 || C::port == SERVO_3 || C::port == SERVO_4)
        && C::pulseAt0 > 0 && C::pulseAt0 < C::pulseAt180 && C::pulseAt180 < 20000
        && low >= 0 && high <= 180 && low < high && C::startAngle >= low && C::startAngle <= high
        && C::stepDeg > 0 && C::stepDeg <= 5 && liftButton(C::up) && liftButton(C::down)
        && C::up != C::down), "Confirm the servo bench wiring, calibration and buttons");

    HexaServo servo{{static_cast<uint8_t>(C::port), C::pulseAt0, C::pulseAt180,
                     0, 180, 0, static_cast<float>(C::startAngle), 0}};
    int target = C::startAngle;
    bool running = false, attached = false, armed = false;
    uint32_t sequence = 0, lastFrame = 0;

    static uint32_t sample(uint16_t& buttons) {
        float lx, ly, rx, ry;
        uint32_t seq;
        NRLGamepad::read(lx, ly, rx, ry, buttons, seq);
        return seq;
    }

public:
    void init() override {
        telemetry.addData("servo lift", C::enabled ? "await start" : "disabled");
    }

    void start() override {
        uint16_t buttons;
        sequence = sample(buttons);
        lastFrame = millis();
        armed = false;
        running = C::enabled;
    }

    void loop() override {
        if (!running) return;
        uint16_t buttons;
        const uint32_t seq = sample(buttons);
        const uint32_t now = millis();
        const bool fresh = seq != sequence;
        sequence = seq;
        const bool lost = NRLComms::isInputStale() || now - lastFrame >= 50;
        if (fresh) lastFrame = now;
        if (lost) {
            armed = false;
            if (attached) {
                stop(); // fixture must be independently supported
                telemetry.addData("servo lift", "link stop");
            }
            return;
        }
        if (!fresh) return;
        if (buttons == 0) {
            if (!attached) {
                // begin() immediately commands startAngle. No actuation in INIT.
                attached = servo.begin();
                if (!attached) {
                    running = false;
                    telemetry.addData("servo lift", "init failed");
                    return;
                }
                target = C::startAngle;
                telemetry.addData("target deg", static_cast<float>(target));
            }
            armed = true;
            return;
        }

        const bool step = armed && (buttons == C::up || buttons == C::down);
        armed = false;
        if (!step) return;
        const int raiseDelta = C::raised > C::lowered ? C::stepDeg : -C::stepDeg;
        const int requested = target + (buttons == C::up ? raiseDelta : -raiseDelta);
        const int next = requested < low ? low : (requested > high ? high : requested);
        if (next != target) {
            target = next;
            servo.setPosition(static_cast<float>(target));
            telemetry.addData("target deg", static_cast<float>(target));
        }
    }

    void stop() override {
        if (attached) servo.detach();
        running = attached = armed = false;
    }
};

using CoreServoLift = ServoLiftBench<ServoBenchConfig>;
REGISTER_OPMODE(CoreServoLift, "CORE Servo Lift", TELEOP);
}
