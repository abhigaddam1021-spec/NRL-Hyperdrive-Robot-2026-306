#include "NRL.h"
#include "HexaServo.h"
using HexaDCMotorConfig = HexaDCMotor::Pins;
#include "v2/HyperdriveTeleOp.cpp"
#include <limits>
#include <cstdio>

using namespace hyperdrive;
static int position[64], attached, servoWrites;
static int failingServoPin = -1;
HexaServo::HexaServo(const HexaServoConfig& config) : m_config(config) {}
bool HexaServo::begin() {
    assert(m_config.settleMs == 0); // idle detach would release the supported target
    if (m_config.signalPin == failingServoPin) return false;
    ++attached;
    position[m_config.signalPin] = m_config.startAngle;
    return true;
}
void HexaServo::detach() { --attached; }
void HexaServo::setPosition(float value) { ++servoWrites; position[m_config.signalPin] = value; }

// All mappings below are synthetic and excluded from the robot project.
constexpr Robot servoFixture() {
    Robot r;
    r.controllerLayoutConfirmed = true; // host fixture only
    r.drive = {true, {MOTOR_L_DIR,MOTOR_L_PWM}, {MOTOR_R_DIR,MOTOR_R_PWM}, 1,-1,2,-3,100,0.1f};
    r.gripEnabled = true;
    r.grip = {SERVO_1,1000,2000,20,70,70};
    r.liftServo = {SERVO_2,1000,2000,85,80,85};
    r.open = BTN_X; r.close = BTN_A; r.raise = BTN_RB; r.lower = BTN_LB;
    r.lift = LiftType::PositionalServo; r.stepDeg = 3;
    return r;
}
constexpr Robot dcFixture() {
    Robot r = servoFixture();
    r.drive.enabled = false;
    r.lift = LiftType::BrushedDC;
    r.liftMotor = r.drive.left;
    r.raisePwm = -35; r.lowerPwm = 20; r.jogMs = 30;
    r.upperSwitch = DIGITAL_D1; r.lowerSwitch = DIGITAL_D2;
    return r;
}
inline constexpr Robot servoRobot = servoFixture(), dcRobot = dcFixture();
inline constexpr Robot disabledRobot = [] {
    Robot r = configuration;
    r.drive.enabled = false;
    r.gripEnabled = false;
    r.lift = LiftType::Disabled;
    return r;
}();
static_assert(!valid([]{ auto r = servoRobot; r.raise = r.open; return r; }()));
static_assert(!valid([]{ auto r = servoRobot; r.liftServo.pin = r.grip.pin; return r; }()));
static_assert(!valid([]{ auto r = dcRobot; r.drive.enabled = true; return r; }()));
static_assert(!valid([]{ auto r = servoRobot; r.liftServo.start = 95; return r; }()));

static_assert(valid(Robot{}));
static_assert(!valid([]{ auto r = servoRobot; r.lift = static_cast<LiftType>(99); return r; }()));
static_assert(!valid([]{ auto r = servoRobot; r.drive.right = r.drive.left; return r; }()));
static_assert(!valid([]{ auto r = servoRobot; r.drive.leftSign = 0; return r; }()));
static_assert(!valid([]{ auto r = servoRobot; r.drive.turnAxis = -r.drive.forwardAxis; return r; }()));
static_assert(!valid([]{ auto r = servoRobot; r.open = 0; return r; }()));
static_assert(!valid([]{ auto r = servoRobot; r.open = BTN_LT; return r; }()));
static_assert(!valid([]{ auto r = servoRobot; r.close = BTN_X | BTN_A; return r; }()));
static_assert(!valid([]{ auto r = servoRobot; r.liftServo.second = -1; return r; }()));
static_assert(!valid([]{ auto r = servoRobot; r.grip.first = 181; return r; }()));
static_assert(!valid([]{ auto r = servoRobot; r.grip.second = r.grip.first; return r; }()));
static_assert(!valid([]{ auto r = servoRobot; r.grip.pulse0 = r.grip.pulse180; return r; }()));
static_assert(!valid([]{ auto r = dcRobot; r.lowerSwitch = -1; return r; }()));
static_assert(!valid([]{ auto r = dcRobot; r.lowerSwitch = r.upperSwitch; return r; }()));
static_assert(!valid([]{ auto r = dcRobot; r.lowerPwm = r.raisePwm; return r; }()));
static_assert(!valid([]{ auto r = dcRobot; r.raisePwm = std::numeric_limits<int>::min(); return r; }()));
static_assert(!valid([]{ auto r = dcRobot; r.jogMs = 0; return r; }()));

constexpr Robot extendedFixture(bool increasingDown) {
    Robot r = servoFixture();
    r.liftServo.first = increasingDown ? 85 : 80;
    r.liftServo.second = increasingDown ? 80 : 85;
    r.liftServo.start = r.liftServo.first;
    r.lowerExtensionDeg = 5;
    r.liftSafeMinDeg = increasingDown ? 80 : 75;
    r.liftSafeMaxDeg = increasingDown ? 90 : 85;
    return r;
}
inline constexpr Robot downPlus = extendedFixture(true), downMinus = extendedFixture(false);
static_assert(loweredTarget(servoRobot) == 85);
static_assert(valid(downPlus) && loweredTarget(downPlus) == 90);
static_assert(valid(downMinus) && loweredTarget(downMinus) == 75);
static_assert(!valid([]{ auto r = downPlus; r.liftSafeMaxDeg = 89; return r; }()));
static_assert(!valid([]{ auto r = downMinus; r.liftSafeMinDeg = 76; return r; }()));
static_assert(!valid([]{ auto r = servoRobot; r.lowerExtensionDeg = 5; return r; }()));
static_assert(!valid([]{ auto r = downPlus; r.lowerExtensionDeg = 10; return r; }()));

constexpr Robot servo2Trial() {
    Robot r = servoRobot;
    r.liftServo.second = configuration.liftServo.second;
    r.liftServo.first = configuration.liftServo.first;
    r.liftSafeMinDeg = r.liftServo.second;
    r.liftSafeMaxDeg = 125;
    r.craneClearanceVerified = true; // synthetic fixture only
    r.grip.first = configuration.grip.first;
    r.grip.start = r.grip.first; // host fixture startup only
    r.grip.second = configuration.grip.second;
    r.gripSafeMinDeg = r.grip.first;
    r.gripSafeMaxDeg = 105;
    return r;
}
inline constexpr Robot trial125 = servo2Trial();
static_assert(valid(trial125) && teamServoCalibrationOK(trial125) && loweredTarget(trial125) == 125);
static_assert(!teamServoCalibrationOK([]{ auto r = trial125; r.lowerExtensionDeg = 5; return r; }()));
static_assert(!teamServoCalibrationOK([]{ auto r = trial125; r.liftSafeMaxDeg = 119; return r; }()));
static_assert(!teamServoCalibrationOK([]{ auto r = trial125; r.liftSafeMinDeg = -1; return r; }()));

static_assert(!teamServoCalibrationOK([]{ auto r = trial125; r.craneClearanceVerified = false; return r; }()));
static_assert(configuration.grip.pin == SERVO_1 && configuration.grip.second == 105);
static_assert(configuration.liftServo.pin == SERVO_2 && configuration.liftServo.first == 125);
static_assert(configuration.lowerExtensionDeg == 0);
static_assert(configuration.grip.first == 85 && configuration.liftServo.second == 50);
static_assert(!teamServoCalibrationOK([]{ auto r = trial125; r.gripSafeMaxDeg = 104; return r; }()));
static_assert(!teamServoCalibrationOK([]{ auto r = trial125; r.liftSafeMaxDeg = 100; return r; }()));
static_assert(!teamServoCalibrationOK([]{ auto r = trial125; r.gripSafeMinDeg = -1; return r; }()));

// Exercise the proposed team bindings on a calibrated synthetic fixture.
inline constexpr Robot mapped = [] {
    Robot r = trial125;
    r.drive.forwardAxis = configuration.drive.forwardAxis;
    r.drive.turnAxis = configuration.drive.turnAxis;
    r.open = configuration.open; r.close = configuration.close;
    r.raise = configuration.raise; r.lower = configuration.lower;
    return r;
}();
static_assert(valid(mapped));
static_assert(mapped.open == BTN_A && mapped.close == BTN_RB);
static_assert(mapped.raise == BTN_RT && mapped.lower == BTN_LB);
static_assert(!valid([]{ auto r=mapped; r.lower=BTN_LT; return r; }()));
static_assert(!valid([]{ auto r=mapped; r.close=4; return r; }())); // absent B bit
static_assert(!valid([]{ auto r=mapped; r.controllerLayoutConfirmed=false; return r; }()));
static_assert(!valid([]{ auto r=mapped; r.close=0; return r; }()));
static_assert(!valid([]{ auto r=mapped; r.lower=0; return r; }()));

static void frame(NRLOpMode& op, int keys = 0, float forward = 0, float turn = 0) {
    ++fixture::sequence;
    fixture::timeMs += 10;
    fixture::buttons = keys;
    fixture::axes[1] = forward;
    fixture::axes[2] = turn;
    op.loop();
}
static void wheels(int l, int r) {
    assert(fixture::pwmByPin[MOTOR_L_DIR] == l && fixture::pwmByPin[MOTOR_R_DIR] == r);
}
template<const Robot& R>
static void exerciseExtension(int firstDown, int expectedLowered, int expectedRaised) {
    TeleOp<R> op;
    op.init(); op.start(); frame(op);
    assert(position[SERVO_2] == R.liftServo.first);
    frame(op, R.lower | R.open);
    assert(position[SERVO_2] == firstDown && position[SERVO_1] == 20);
    for (int i = 0; i < 4; ++i) {
        frame(op); frame(op, R.lower);
        assert(position[SERVO_2] == expectedLowered && position[SERVO_1] == 20);
    }
    for (int i = 0; i < 6; ++i) {
        frame(op); frame(op, R.raise | R.close);
        assert(position[SERVO_2] >= R.liftSafeMinDeg && position[SERVO_2] <= R.liftSafeMaxDeg);
        assert(position[SERVO_1] == 70);
    }
    assert(position[SERVO_2] == expectedRaised);
    op.stop();
}
int main() {
    TeleOp<disabledRobot> disabled;
    disabled.init(); disabled.start(); frame(disabled); frame(disabled,BTN_X,1); disabled.stop();
    assert(!attached && !fixture::begins && !fixture::writes && !fixture::inputs && !servoWrites);
    // After START, losing the link while waiting for neutral must cancel
    // this run too. An idle frame on reconnect must not move servos to start.
    TeleOp<servoRobot> interruptedStart;
    interruptedStart.init();
    interruptedStart.start();
    frame(interruptedStart, BTN_X);
    fixture::stale = true;
    interruptedStart.loop();
    fixture::stale = false;
    frame(interruptedStart);
    assert(!attached && !fixture::begins && !fixture::writes);
    interruptedStart.stop();
    TeleOp<servoRobot> op;
    op.init(); op.start(); frame(op,0,1); frame(op,BTN_X);
    assert(!attached && !fixture::begins);
    frame(op);
    assert(attached == 2 && position[SERVO_1] == 70 && position[SERVO_2] == 85);
    frame(op,BTN_X | BTN_RB,1); wheels(100,-100);
    assert(position[SERVO_1] == 20 && position[SERVO_2] == 82);
    frame(op,BTN_X | BTN_RB,0,1); wheels(-100,-100);
    assert(servoWrites == 2);
    // A full second of real neutral packets after raising leaves both servos attached.
    for (int i = 0; i < 100; ++i) frame(op);
    assert(position[SERVO_2] == 82 && attached == 2 && servoWrites == 2);
    frame(op,0,1,-1); wheels(100,0);
    frame(op,BTN_X | BTN_A | BTN_LB | BTN_RB,0.05f); wheels(0,0);
    frame(op,BTN_A | BTN_LB); assert(servoWrites == 2);
    frame(op); frame(op,BTN_A | BTN_RB,-1); wheels(-100,100);
    assert(position[SERVO_1] == 70 && position[SERVO_2] == 80);
    frame(op); frame(op,BTN_RB); assert(position[SERVO_2] == 80);
    frame(op); frame(op,BTN_LB); assert(position[SERVO_2] == 83);
    frame(op,0,1); fixture::timeMs += 50; op.loop();
    wheels(0,0); assert(!attached);
    frame(op); frame(op,BTN_X,1); wheels(0,0); assert(!attached);
    for (int fault = 0; fault < 4; ++fault) {
        op.start(); frame(op); frame(op,0,1); wheels(100,-100);
        if (fault == 0) { fixture::stale = true; op.loop(); fixture::stale = false; }
        if (fault == 1) frame(op,0,std::numeric_limits<float>::quiet_NaN());
        if (fault == 2) frame(op,4,1);
        if (fault == 3) op.stop();
        assert(!attached); wheels(0,0);
    }
    fixture::failMotorPin = MOTOR_R_DIR;
    op.start(); frame(op); assert(!attached); wheels(0,0);
    frame(op,0,1); wheels(0,0);
    fixture::failMotorPin = -1;

    TeleOp<dcRobot> dc;
    dc.init(); dc.start(); frame(dc); frame(dc,BTN_RB); wheels(-35,0);
    frame(dc); wheels(0,0); // release commands coast, with no open-loop holding power
    frame(dc,BTN_RB); wheels(-35,0);
    fixture::upperOpen = true; dc.loop(); wheels(0,0);
    frame(dc); frame(dc,BTN_LB); wheels(20,0);
    fixture::upperOpen = false; fixture::lowerOpen = true; dc.loop(); wheels(0,0);
    fixture::lowerOpen = false; frame(dc); frame(dc,BTN_RB);
    frame(dc,BTN_RB); frame(dc,BTN_RB); frame(dc,BTN_RB); wheels(0,0);
    frame(dc); frame(dc,BTN_LB); dc.stop(); wheels(0,0); assert(!attached);
    TeleOp<trial125> crane;
    crane.init(); crane.start(); frame(crane);
    frame(crane, trial125.close);
    assert(position[SERVO_1] == 105);
    const int gripBefore = position[SERVO_1];
    for (int i = 0; i < 24; ++i) {
        frame(crane); frame(crane, trial125.lower);
        assert(position[SERVO_2] >= trial125.liftSafeMinDeg && position[SERVO_2] <= 125);
        assert(position[SERVO_1] == gripBefore);
    }
    assert(position[SERVO_2] == 125);
    const int writesAtRelease = servoWrites;
    for (int i = 0; i < 100; ++i) frame(crane);
    assert(position[SERVO_2] == 125 && servoWrites == writesAtRelease && attached == 2);
    for (int i = 0; i < 28; ++i) {
        frame(crane); frame(crane, trial125.raise);
        assert(position[SERVO_2] >= 50 && position[SERVO_2] <= 125);
    }
    assert(position[SERVO_2] == 50 && position[SERVO_1] == 105);
    frame(crane); frame(crane, trial125.open);
    assert(position[SERVO_1] == 85 && position[SERVO_2] == 50);
    crane.stop();
    // Failure of the second servo must release the first and inhibit the drivetrain.
    failingServoPin = SERVO_2;
    const int beginsBeforeFailure = fixture::begins;
    crane.start(); frame(crane);
    assert(attached == 0 && fixture::begins == beginsBeforeFailure);
    const int writesBeforeRetry = servoWrites;
    failingServoPin = -1;
    frame(crane); frame(crane, trial125.close | trial125.lower, 1);
    assert(attached == 0 && fixture::begins == beginsBeforeFailure && servoWrites == writesBeforeRetry);

    // Frozen packet stream across millis() rollover: keep the 50 ms fault boundary.
    fixture::timeMs = std::numeric_limits<uint32_t>::max() - 40;
    crane.start(); frame(crane); frame(crane, trial125.close, 1);
    wheels(100,-100);
    fixture::timeMs += 49;
    crane.loop();
    assert(attached == 2 && position[SERVO_1] == 105);
    ++fixture::timeMs;
    crane.loop(); wheels(0,0); assert(!attached);
    frame(crane); frame(crane, trial125.lower, 1); wheels(0,0); assert(!attached);
    // Check both steering extremes and normalized mixed commands in every quadrant.
    TeleOp<servoRobot> driveTest;
    driveTest.init(); driveTest.start(); frame(driveTest);
    struct DriveCase { float throttle, steering; int left, right; };
    const DriveCase mixes[] = {
        {0,-1,100,100}, {0,1,-100,-100},
        {1,0.5f,33,-100}, {1,-0.5f,100,-33},
        {-1,0.5f,-100,33}, {-1,-0.5f,-33,100},
        {0.1f,-0.1f,0,0}
    };
    for (const auto& command : mixes) {
        frame(driveTest, 0, command.throttle, command.steering);
        wheels(command.left, command.right);
    }
    driveTest.stop();
    TeleOp<mapped> v2;
    v2.init(); v2.start(); frame(v2, BTN_RT); assert(!attached);
    frame(v2); assert(attached == 2);
    frame(v2, mapped.close); assert(position[SERVO_1] == 105);
    frame(v2); const int previousHeight = position[SERVO_2];
    frame(v2, BTN_A | BTN_RT, 1, 0.5f); wheels(100,-33);
    assert(position[SERVO_1] == 85 && position[SERVO_2] == previousHeight - mapped.stepDeg);
    const int writesAfterStep = servoWrites;
    frame(v2, BTN_RT, -1, 0); wheels(-100,100);
    frame(v2, BTN_RT | mapped.lower); frame(v2, mapped.lower);
    fixture::axes[3] = 1;
    frame(v2, BTN_X | BTN_Y); wheels(0,0); assert(servoWrites == writesAfterStep);
    fixture::axes[3] = 0;
    frame(v2, BTN_LB);
    assert(position[SERVO_2] == previousHeight);
    frame(v2, BTN_LB);
    assert(servoWrites == writesAfterStep + 1);
    frame(v2); frame(v2, BTN_A | BTN_RB); frame(v2, BTN_RB);
    assert(position[SERVO_1] == 85 && servoWrites == writesAfterStep + 1);
    frame(v2); frame(v2, BTN_RB);
    assert(position[SERVO_1] == 105);
    const int writesBeforeStop = servoWrites;
    frame(v2, BTN_RT | BTN_LT, 1, 1); wheels(0,0);
    assert(!attached && servoWrites == writesBeforeStop);
    frame(v2); frame(v2, BTN_RT); assert(!attached);
    exerciseExtension<downPlus>(88, 90, 80);
    exerciseExtension<downMinus>(77, 75, 85);
    puts("PASS: competition configuration, drive, grip, servo/DC lift, limits, faults and STOP");
}
