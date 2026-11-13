#include "../RobotFirmware/opmodes/CoreLiftJog.cpp"
#include <cstdio>

// Synthetic fixture only. These assignments never enter the robot build.
struct TestConfig {
    static constexpr bool enableBench = true;
    static constexpr int dirPin = MOTOR_L_DIR, pwmPin = MOTOR_L_PWM;
    static constexpr int upperPin = DIGITAL_D1, lowerPin = DIGITAL_D2;
    static constexpr unsigned upButton = BTN_X, downButton = BTN_A;
    static constexpr int raiseCommand = -32, lowerCommand = 18;
    static constexpr uint32_t pulseMs = 100;
};

static void packet(NRLOpMode& op, uint16_t buttons, int expected) {
    ++fixture::sequence;
    fixture::buttons = buttons;
    fixture::timeMs += 10;
    op.loop();
    assert(fixture::lastPwm == expected);
}

int main() {
    using namespace fixture;
    CoreLiftJog disabled;
    disabled.init();
    disabled.start();
    packet(disabled, 0, 0);
    packet(disabled, BTN_X, 0);
    disabled.stop();
    assert(inputs == 0 && begins == 0 && writes == 0 && stops == 0);

    BenchOpMode<TestConfig> op;
    initOK = false;
    op.init();
    op.start();
    packet(op, 0, 0);
    packet(op, BTN_X, 0);
    op.stop();
    assert(begins == 1 && writes == 0);

    initOK = true;
    op.init();
    assert(stops == 1 && lastPwm == 0);
    packet(op, 0, 0);
    packet(op, BTN_X, 0);
    assert(writes == 0); // pre-START callbacks cannot drive
    op.start();
    packet(op, BTN_X, 0);
    packet(op, 0, 0);
    packet(op, BTN_X, -32);
    stale = true;
    op.loop();
    assert(lastPwm == 0);
    stale = false;
    packet(op, BTN_X, 0); // reconnect while held
    packet(op, 0, 0);
    packet(op, BTN_A, 18);
    timeMs += 50; // sequence frozen, SDK still reports connected
    op.loop();
    assert(lastPwm == 0);
    packet(op, BTN_A, 0);
    packet(op, 0, 0);
    packet(op, BTN_A, 18);
    buttons = 0; // SDK clear without a real release packet
    op.loop();
    assert(lastPwm == 0);
    packet(op, BTN_A, 0);
    packet(op, 0, 0);
    packet(op, BTN_X, -32);
    packet(op, BTN_X | 4, 0); // unused controller bit
    packet(op, BTN_X, 0);
    packet(op, 0, 0);
    packet(op, BTN_X | BTN_A, 0);
    packet(op, BTN_A, 0);
    packet(op, 0, 0);
    packet(op, BTN_X, -32);
    upperOpen = true;
    packet(op, BTN_X, 0);
    packet(op, 0, 0);
    packet(op, BTN_A, 18);
    lowerOpen = true;
    packet(op, BTN_A, 0);
    upperOpen = lowerOpen = false;
    packet(op, 0, 0);
    packet(op, BTN_X, -32);
    op.stop(); // invoke SDK STOP callback during a moving jog
    assert(stops == 2 && lastPwm == 0);
    const int writesAtStop = writes;
    packet(op, 0, 0);
    packet(op, BTN_X, 0);
    op.stop();
    assert(writes == writesAtStop && stops == 2);
    puts("PASS: disabled and configured OpMode lifecycle, inputs and motor outputs");
}
