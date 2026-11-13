#include "../RobotFirmware/opmodes/CoreServoLift.cpp"
#include <cassert>
#include <cstdio>
#include <vector>

// Link the SDK's real public header to a host-only recording actuator.
static int starts, detaches;
static bool beginOK = true;
static std::vector<float> commands;
HexaServo::HexaServo(const HexaServoConfig& config) : m_config(config) {}
bool HexaServo::begin() {
    ++starts;
    if (beginOK) commands.push_back(m_config.startAngle);
    return beginOK;
}
void HexaServo::setPosition(float degrees) { commands.push_back(degrees); }
void HexaServo::detach() { ++detaches; }

// Synthetic narrow travel range verifies partial last steps at both ends.
struct FixtureConfig {
    static constexpr bool enabled = true;
    static constexpr int port = SERVO_1, pulseAt0 = 1000, pulseAt180 = 2000;
    static constexpr int lowered = 80, raised = 87, startAngle = 83, stepDeg = 3;
    static constexpr unsigned up = BTN_X, down = BTN_A;
};
struct Reversed : FixtureConfig { static constexpr int lowered = 87, raised = 80; };

static void packet(NRLOpMode& op, unsigned buttons) {
    fixture::timeMs += 10;
    ++fixture::sequence;
    fixture::buttons = buttons;
    op.loop();
}

template<class Config>
static void travel() {
    ServoLiftBench<Config> op;
    commands.clear();
    op.init();
    op.start();
    packet(op, BTN_X);
    assert(commands.empty());
    packet(op, 0);
    packet(op, 0);
    assert(commands == std::vector<float>{83});
    for (unsigned key : {BTN_X, BTN_A}) {
        const int end = key == BTN_X ? Config::raised : Config::lowered;
        for (int i = 0; i < 5; ++i) {
            const float before = commands.back();
            packet(op, 0);
            packet(op, key);
            const float after = commands.back();
            assert(after >= 80 && after <= 87);
            assert(after >= before - 3 && after <= before + 3);
            assert(end == 87 ? after >= before : after <= before);
            const auto count = commands.size();
            packet(op, key);
            assert(commands.size() == count);
        }
        assert(commands.back() == end);
    }
    packet(op, 0);
    const auto count = commands.size();
    packet(op, BTN_X | BTN_A);
    packet(op, BTN_X);
    packet(op, 0);
    packet(op, 4);
    packet(op, BTN_X);
    assert(commands.size() == count);
    op.stop();
}

int main() {
    CoreServoLift disabled;
    disabled.init();
    disabled.start();
    packet(disabled, 0);
    packet(disabled, BTN_X);
    disabled.stop();
    assert(starts == 0 && commands.empty() && detaches == 0);
    travel<FixtureConfig>();
    travel<Reversed>();
    for (bool radioStale : {true, false}) {
        ServoLiftBench<FixtureConfig> op;
        op.init();
        op.start();
        packet(op, 0);
        packet(op, BTN_X);
        const int before = detaches;
        const auto count = commands.size();
        fixture::stale = radioStale;
        if (!radioStale) fixture::timeMs += 50;
        op.loop();
        assert(detaches == before + 1);
        fixture::stale = false;
        packet(op, 0);
        packet(op, BTN_A);
        op.stop();
        assert(commands.size() == count && detaches == before + 1);
    }
    beginOK = false;
    ServoLiftBench<FixtureConfig> failed;
    failed.init();
    failed.start();
    const auto count = commands.size();
    const int before = starts;
    packet(failed, 0);
    packet(failed, 0);
    packet(failed, BTN_X);
    assert(commands.size() == count && starts == before + 1);
    puts("PASS: servo bounds, both orientations, neutral/rearm, link loss and disabled lifecycle");
}
