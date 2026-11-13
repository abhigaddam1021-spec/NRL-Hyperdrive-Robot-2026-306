#include "HyperdriveConfig.h"
#include "LiftJog.h"
#include <cmath>

namespace hyperdrive {
static HexaServoConfig servoConfig(Pose p) {
    return {static_cast<uint8_t>(p.pin), static_cast<uint16_t>(p.pulse0),
            static_cast<uint16_t>(p.pulse180), 0, 180, 0, static_cast<float>(p.start), 0};
}
static HexaDCMotorConfig motorConfig(MotorPort p) {
    return {static_cast<uint8_t>(p.dir), static_cast<uint8_t>(p.pwm)};
}

// Both held locks the pair until a fresh release. Independent pairs may operate together.
class ButtonPair {
    bool armed = false;
public:
    int read(bool first, bool second) {
        if (!first && !second) { armed = true; return 0; }
        const bool accepted = armed && first != second;
        armed = false;
        return accepted ? (first ? 1 : -1) : 0;
    }
};

template<const Robot& R>
class TeleOp : public NRLOpMode {
    static_assert(valid(R), "Invalid Hyperdrive hardware, calibration or controller mapping");
    HexaDCMotor left{motorConfig(R.drive.left)}, right{motorConfig(R.drive.right)};
    HexaDCMotor liftMotor{motorConfig(R.liftMotor)};
    HexaServo grip{servoConfig(R.grip)}, liftServo{servoConfig(R.liftServo)};
    bool leftReady = false, rightReady = false, motorReady = false;
    bool gripReady = false, servoReady = false, running = false, active = false;
    ButtonPair gripKeys, liftKeys;
    LiftJog jog;
    int angle = R.liftServo.start;
    uint32_t seq = 0, lastFrame = 0;

    bool initializeOutputs() {
        // SDK requires allocating servo PWM channels before motor PWM channels.
        if constexpr (R.gripEnabled) if (!(gripReady = grip.begin())) return false;
        if constexpr (R.lift == LiftType::PositionalServo) {
            if (!(servoReady = liftServo.begin())) return false;
            telemetry.addData("target deg", static_cast<float>(angle));
        }
        if constexpr (R.drive.enabled) {
            if (!(leftReady = left.begin())) return false;
            left.stop();
            if (!(rightReady = right.begin())) return false;
            right.stop();
        }
        if constexpr (R.lift == LiftType::BrushedDC) {
            pinMode(R.upperSwitch, INPUT_PULLUP);
            pinMode(R.lowerSwitch, INPUT_PULLUP);
            if (!(motorReady = liftMotor.begin())) return false;
            liftMotor.stop();
        }
        return true;
    }
    static float axis(const float* axes, int selector) {
        const float v = axes[magnitude(selector) - 1];
        return std::fabs(v) <= R.drive.deadband ? 0 : (selector < 0 ? -v : v);
    }
    void process(float forward, float turn, unsigned buttons, bool fresh, uint32_t now) {
        if constexpr (R.drive.enabled) {
            const float scale = R.drive.maxPwm / std::fmax(1.f, std::fabs(forward) + std::fabs(turn));
            left.setSpeed(std::lround((forward + turn) * scale) * R.drive.leftSign);
            right.setSpeed(std::lround((forward - turn) * scale) * R.drive.rightSign);
        }
        if constexpr (R.gripEnabled) {
            if (fresh) {
                const int cmd = gripKeys.read(buttons & R.open, buttons & R.close);
                if (cmd) grip.setPosition(cmd > 0 ? R.grip.first : R.grip.second);
            }
        }
        if constexpr (R.lift == LiftType::PositionalServo) {
            if (fresh) {
                const int cmd = liftKeys.read(buttons & R.raise, buttons & R.lower);
                if (cmd) {
                    const int end = cmd > 0 ? R.liftServo.second : loweredTarget(R);
                    const int distance = end - angle;
                    angle += magnitude(distance) <= R.stepDeg ? distance : (distance > 0 ? R.stepDeg : -R.stepDeg);
                    liftServo.setPosition(angle);
                    telemetry.addData("target deg", static_cast<float>(angle));
                }
            }
        }
        if constexpr (R.lift == LiftType::BrushedDC) {
            liftMotor.setSpeed(jog.update(now, true, buttons & R.raise, buttons & R.lower,
                digitalRead(R.upperSwitch) == HIGH, digitalRead(R.lowerSwitch) == HIGH,
                R.jogMs, R.raisePwm, R.lowerPwm, fresh));
        }
    }
public:
    void init() override {
        telemetry.addData("drive", R.drive.enabled ? "configured" : "disabled");
        telemetry.addData("gripper", R.gripEnabled ? "configured" : "disabled");
        telemetry.addData("lift", R.lift == LiftType::Disabled ? "disabled" : "configured");
    }
    void start() override {
        float x,y,z,w;
        uint16_t buttons;
        NRLGamepad::read(x,y,z,w,buttons,seq);
        lastFrame = millis();
        running = true;
        telemetry.addData("control", "await neutral");
    }
    void loop() override {
        if (!running) return;
        float axes[4];
        uint16_t buttons;
        uint32_t packet;
        NRLGamepad::read(axes[0], axes[1], axes[2], axes[3], buttons, packet);
        const uint32_t now = millis();
        const bool fresh = packet != seq;
        const bool lost = NRLComms::isInputStale() || now - lastFrame >= 50;
        if (fresh) lastFrame = now;
        seq = packet;
        bool invalid = buttons & ~(keys | BTN_LT | BTN_RT);
        for (float a : axes) invalid |= !(a >= -1 && a <= 1);
        if (lost || invalid) {
            stop();
            telemetry.addData("control", "input fault");
            return;
        }
        if constexpr (!R.drive.enabled && !R.gripEnabled && R.lift == LiftType::Disabled) {
            if (fresh) {
                telemetry.addData("LX", axes[0]);
                telemetry.addData("LY", axes[1]);
                telemetry.addData("RX", axes[2]);
                telemetry.addData("RY", axes[3]);
                telemetry.addData("buttons", static_cast<float>(buttons));
            }
        }
        float forward = 0, turn = 0;
        if constexpr (R.drive.enabled) {
            forward = axis(axes, R.drive.forwardAxis);
            turn = axis(axes, R.drive.turnAxis);
        }
        if (!active) {
            if (!fresh || buttons || forward || turn) return;
            if (!initializeOutputs()) { stop(); telemetry.addData("control", "init failed"); return; }
            active = true;
            telemetry.addData("control", "running");
        }
        // LT belongs to the controller exit UI. If its bit reaches RUN, stop
        // this OpMode before applying other controls from the same frame.
        if (buttons & BTN_LT) {
            stop();
            telemetry.addData("control", "LT stop");
            return;
        }
        process(forward, turn, buttons, fresh, now);
    }
    void stop() override {
        if (leftReady) left.stop();
        if (rightReady) right.stop();
        if (motorReady) liftMotor.stop();
        if (gripReady) grip.detach();
        if (servoReady) liftServo.detach();
        leftReady = rightReady = motorReady = gripReady = servoReady = false;
        running = active = false;
        angle = R.liftServo.start;
        jog = LiftJog{};
        gripKeys = liftKeys = ButtonPair{};
    }
};
using HyperdriveTeleOp = TeleOp<configuration>;
REGISTER_OPMODE(HyperdriveTeleOp, "Hyperdrive TeleOp", TELEOP);
}
