#include "HyperdriveConfig.h"
#include <cstdio>

// ============================================================
//  HYPERDRIVE V4 commissioning OpModes
// ============================================================
//  "HD Input Monitor" (Stage A) - constructs no motor and no servo. Shows the
//      raw button mask and stick values so each physical button can be matched
//      to its SDK bit. Cannot move anything.
//  "HD Wheel Check" (Stage B) - spins ONE wheel output at low power while LB
//      is held, to record which way that wheel turns. Only wheels whose
//      socket->channel routing is already marked Verified can be selected, and
//      the whole OpMode is inert unless drive.wheelCheckEnabled is set.

namespace hyperdrive {

class InputMonitor : public NRLOpMode {
    uint32_t sequence = 0, packets = 0;
public:
    void start() override {
        float a, b, c, d; uint16_t keys;
        NRLGamepad::read(a, b, c, d, keys, sequence);
    }
    void loop() override {
        float lx, ly, rx, ry; uint16_t buttons; uint32_t packet;
        NRLGamepad::read(lx, ly, rx, ry, buttons, packet);
        if (packet != sequence) { sequence = packet; ++packets; }
        char hex[16];
        if (NRLComms::isInputStale()) snprintf(hex, sizeof hex, "no link");
        else snprintf(hex, sizeof hex, "0x%03X", static_cast<unsigned>(buttons));
        telemetry.addData("mask", hex);
        telemetry.addData("LX", lx); telemetry.addData("LY", ly);
        telemetry.addData("RX", rx); telemetry.addData("RY", ry);
        telemetry.addData("outputs", "none");
    }
};
REGISTER_OPMODE(InputMonitor, "HD Input Monitor", TELEOP);

template<const Robot& R>
class WheelCheck : public NRLOpMode {
    static_assert(valid(R), "Invalid HYPERDRIVE V4 configuration");
    HexaDCMotor motors[channelCount] = {
        HexaDCMotor{{static_cast<uint8_t>(MOTOR_L_DIR), static_cast<uint8_t>(MOTOR_L_PWM)}},
        HexaDCMotor{{static_cast<uint8_t>(MOTOR_R_DIR), static_cast<uint8_t>(MOTOR_R_PWM)}},
        HexaDCMotor{{static_cast<uint8_t>(MOTOR_3_DIR), static_cast<uint8_t>(MOTOR_3_PWM)}},
        HexaDCMotor{{static_cast<uint8_t>(MOTOR_4_DIR), static_cast<uint8_t>(MOTOR_4_PWM)}}};
    bool ready[channelCount] = {};
    bool running = false, armed = false, failed = false;
    int selected = -1;
    uint32_t sequence = 0, lastFrame = 0;
    unsigned previous = 0;

    static int index(Channel c) { return static_cast<int>(c) - 1; }
    void halt() {
        for (int c = 0; c < channelCount; ++c) if (ready[c]) motors[c].stop();
    }
    int nextWheel(int from) const {
        for (int n = 1; n <= WheelCount; ++n) {
            const int w = (from + n + WheelCount) % WheelCount;
            if (wheelRouted(R.drive, w)) return w;
        }
        return -1;
    }
public:
    int selectedWheel() const { return selected; }
    void init() override {
        telemetry.addData("wheel", R.drive.wheelCheckEnabled ? "await START" : "disabled");
    }
    void start() override {
        float a, b, c, d; uint16_t keys;
        NRLGamepad::read(a, b, c, d, keys, sequence);
        lastFrame = millis(); previous = 0; armed = false;
        running = R.drive.wheelCheckEnabled;
    }
    void loop() override {
        if (!running || failed) return;
        float lx, ly, rx, ry; uint16_t buttons; uint32_t packet;
        NRLGamepad::read(lx, ly, rx, ry, buttons, packet);
        const uint32_t now = millis();
        if (buttons & (BTN_LT | BTN_RT)) { stop(); return; }
        const bool linkBad = NRLComms::isInputStale() || now - lastFrame >= R.safety.linkTimeoutMs;
        const bool fresh = packet != sequence;
        if (fresh) { sequence = packet; lastFrame = now; }
        if (linkBad) { halt(); armed = false; previous = 0; return; }   // re-arm from neutral
        const unsigned rising = fresh ? buttons & ~previous : 0;
        if (fresh) previous = buttons;
        if (!armed) {
            halt();
            if (!fresh || buttons) return;
            for (int w = 0; w < WheelCount; ++w) {
                if (!wheelRouted(R.drive, w)) continue;
                const int c = index(R.drive.wheel[w].channel);
                if (ready[c]) continue;
                if (!motors[c].begin()) { failed = true; halt(); telemetry.addData("wheel", "init failed"); return; }
                ready[c] = true; motors[c].stop();
            }
            if (selected < 0) selected = nextWheel(-1);
            armed = true;
        }
        if (rising & BTN_RB) { halt(); selected = nextWheel(selected); }
        // Dead-man: exactly LB, nothing else, or the wheel stops.
        const bool spin = buttons == BTN_LB && selected >= 0;
        halt();
        if (spin) motors[index(R.drive.wheel[selected].channel)].setSpeed(R.drive.wheelCheckPwm);
        static const char* const names[WheelCount] = {"FrontLeft", "RearLeft", "FrontRight", "RearRight"};
        telemetry.addData("wheel", selected < 0 ? "none routed" : names[selected]);
        telemetry.addData("socket", static_cast<float>(selected < 0 ? 0 : R.drive.wheel[selected].socket));
        telemetry.addData("command", spin ? "+ raw (LB held)" : "stopped");
    }
    void stop() override {
        halt();
        for (bool& r : ready) r = false;
        running = false; armed = false;
    }
};
using HyperdriveWheelCheck = WheelCheck<configuration>;
REGISTER_OPMODE(HyperdriveWheelCheck, "HD Wheel Check", TELEOP);
}  // namespace hyperdrive
