// HYPERDRIVE V4 host regression suite - the 96 cases of the master directive,
// section 23, plus extras. Runs the real OpMode code against host doubles of
// the SDK. Every hardware value in this file is synthetic and never enters
// the robot build. A PASS here is HOST-TESTED, not HARDWARE-VERIFIED.
#include "NRL.h"
#include "HexaServo.h"
using HexaDCMotorConfig = HexaDCMotor::Pins;
#include "../RobotFirmware/opmodes/HyperdriveTeleOp.cpp"
#include "../RobotFirmware/opmodes/HyperdriveCommissioning.cpp"
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string_view>
using namespace hyperdrive;

static int angle[64], attached, writes, failing = -1, begins;
HexaServo::HexaServo(const HexaServoConfig& c) : m_config(c) {}
bool HexaServo::begin() {
    ++begins;
    if (m_config.signalPin == failing) return false;
    ++attached; angle[m_config.signalPin] = m_config.startAngle;
    return true;
}
void HexaServo::detach() { --attached; }
void HexaServo::setPosition(float a) { ++writes; angle[m_config.signalPin] = a; }

static int passed = 0, current = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL T%02d line %d: %s\n", current, __LINE__, #c); exit(1); } } while (0)
#define T(n, name, ...) do { current = n; reset(); __VA_ARGS__; reset(); ++passed; printf("T%02d PASS  %s\n", n, name); } while (0)
#define X(n, name, ...) do { current = 100 + n; reset(); __VA_ARGS__; reset(); ++passed; printf("X%02d PASS  %s\n", n, name); } while (0)

// ---- synthetic robots -------------------------------------------------------
constexpr Robot fixtureRobot() {
    Robot r;
    const unsigned masks[SlotCount] = {BTN_X, BTN_A, BTN_Y, BTN_DPAD_DOWN, BTN_DPAD_UP,
                                       BTN_DPAD_LEFT, BTN_DPAD_RIGHT};
    for (int i = 0; i < SlotCount; ++i) { r.map.slot[i].mask = masks[i]; r.map.slot[i].status = Status::Verified; }
    r.map.fullLayoutEnabled = true;
    r.drive.enabled = r.drive.motor4PinsFreeVerified = r.drive.driverCapacityVerified = true;
    for (Status& s : r.drive.groupObserved) s = Status::Verified;
    r.drive.wheel[FrontLeft]  = {3, Channel::M4, -1, Status::Verified};
    r.drive.wheel[RearLeft]   = {4, Channel::R,   1, Status::Verified};
    r.drive.wheel[FrontRight] = {2, Channel::L,   1, Status::Verified};
    r.drive.wheel[RearRight]  = {1, Channel::M3, -1, Status::Verified};
    r.drive.throttleSign = r.drive.steeringSign = 1;
    r.drive.maxPwm = 100; r.drive.deadband = .1f;
    r.grip = {true, SERVO_1, 85, 120, 85, 85, 120, 30, 200, 30};
    r.crane = {true, SERVO_2, 30, 120, 30, 30, 120, 30, 200, 40};
    r.payload = {true, SERVO_3, 70, 90, 70, 70, 90, 30, 200, 25};
    r.parkedPayload = 70;
    r.craneClearanceVerified = r.payloadPathAndPowerVerified = true;
    r.safety.responsesVerified = true;
    r.pickup = {true, true, 500, 700};
    r.turns = {TurnMode::Timed, 50, 1, 300, 160, 80, 85, 0, 0, 1};
    return r;
}
inline constexpr Robot rig = fixtureRobot();
inline constexpr Robot gyro = [] { auto r = rig; r.turns.mode = TurnMode::Heading;
    r.turns.headingRightSign = 1; r.turns.toleranceDeg = 1; return r; }();
inline constexpr Robot calibrated = [] { auto r = rig; r.stow = {true, 120, 60, 75, DIGITAL_D1}; return r; }();
inline constexpr Robot slowLink = [] { auto r = rig; r.safety.linkTimeoutMs = 500; r.pickup.timeoutMs = 611; return r; }();
inline constexpr Robot dropOnLink = [] { auto r = rig; r.safety.onLinkLoss = ServoResponse::RemoveAllPwm; return r; }();
inline constexpr Robot dropLiftOnCancel = [] { auto r = rig; r.safety.onCancelWithoutStow = ServoResponse::RemoveLiftPwm; return r; }();
inline constexpr Robot paired = [] { auto r = rig; r.drive.pairedSocketsVerified = true;
    r.drive.wheel[FrontLeft] = {3, Channel::L, 1, Status::Verified};
    r.drive.wheel[RearLeft] = {4, Channel::L, 1, Status::Verified};
    r.drive.wheel[FrontRight] = {2, Channel::R, -1, Status::Verified};
    r.drive.wheel[RearRight] = {1, Channel::R, -1, Status::Verified}; return r; }();
// L-labelled wheels on the robot's own right side: a robot-right turn drives them backward.
inline constexpr Robot mirrored = [] { auto r = rig; r.turns.rightTurnSign = -1; return r; }();
inline constexpr Robot mirroredGyro = [] { auto r = gyro; r.turns.rightTurnSign = -1; return r; }();
static_assert(valid(mirrored) && valid(mirroredGyro));
inline constexpr Robot slewed = [] { auto r = rig; r.drive.slewPwmPerSecond = 1000; return r; }();
inline constexpr Robot noRound = [] { auto r = rig; r.map = {}; r.pickup = {}; r.payload = {}; r.turns = {};
    r.map.slot[R2].mask = BTN_A; return r; }();   // mask present but Unverified: must stay inert
inline constexpr Robot servosOnly = [] { auto r = rig; r.drive = {}; r.drive.deadband = .1f; r.turns = {}; return r; }();
inline constexpr Robot wheelTool = [] { Robot r; r.drive.wheelCheckEnabled = true; r.drive.wheelCheckPwm = 40;
    r.drive.wheel[FrontLeft] = {3, Channel::L, 0, Status::Verified}; return r; }();
static_assert(valid(rig) && valid(gyro) && valid(calibrated) && valid(slowLink) && valid(dropOnLink));
static_assert(valid(paired) && valid(slewed) && valid(noRound) && valid(servosOnly) && valid(wheelTool));
static_assert(valid(configuration) && stowReady(calibrated) && !stowReady(rig));
template<typename F> constexpr Error with(F edit) { auto r = rig; edit(r); return check(r); }
template<typename F> constexpr Error withStow(F edit) { auto r = calibrated; edit(r); return check(r); }

// ---- driving the OpMode ------------------------------------------------------
static void reset() {
    CHECK(attached == 0);
    fixture::timeMs += 1000; fixture::stale = false; fixture::failMotorPin = -1; failing = -1;
    fixture::upperOpen = false; power.low = false; NRLComms::headingReady = true; NRLComms::heading = 0;
    fixture::buttons = 0; for (float& a : fixture::axes) a = 0;
    for (int& p : fixture::pwmByPin) p = 0;
}
static void frame(NRLOpMode& op, unsigned keys = 0, unsigned dt = 1, float f = 0, float t = 0) {
    fixture::timeMs += dt; ++fixture::sequence; fixture::buttons = keys;
    fixture::axes[1] = f; fixture::axes[2] = t; op.loop();
}
static void tickNoPacket(NRLOpMode& op, unsigned keys = 0, unsigned dt = 1) {   // radio silent
    fixture::timeMs += dt; fixture::buttons = keys; op.loop();
}
static void elapsed(NRLOpMode& op, unsigned ms, unsigned keys = 0, float f = 0) {
    while (ms) { unsigned dt = std::min(10u, ms); frame(op, keys, dt, f); ms -= dt; }
}
template<typename Op> static void start(Op& op, int servos = 3) {
    op.init(); op.start(); frame(op); CHECK(attached == servos);
    elapsed(op, 40); frame(op); CHECK(op.currentPhase() == Phase::Idle);
}
static void idle(NRLOpMode& op, unsigned ms) { elapsed(op, ms); frame(op); frame(op); }
static bool wheels(int l, int r) {
    return fixture::pwmByPin[MOTOR_4_DIR] == -l && fixture::pwmByPin[MOTOR_R_DIR] == l &&
           fixture::pwmByPin[MOTOR_L_DIR] == r && fixture::pwmByPin[MOTOR_3_DIR] == -r;
}
enum Stage { AtLowering, AtDwell, AtGripping, AtRaising, StageCount };
template<typename Op> static void r2To(Op& op, int stage) {
    frame(op, rig.map.slot[R2].mask); CHECK(op.currentPhase() == Phase::Lowering);
    if (stage >= AtDwell) { elapsed(op, 40); CHECK(op.currentPhase() == Phase::Dwell); }
    if (stage >= AtGripping) { elapsed(op, 500); CHECK(op.currentPhase() == Phase::Gripping); }
    if (stage >= AtRaising) { elapsed(op, 30); CHECK(op.currentPhase() == Phase::Raising); }
}
constexpr unsigned kR1 = BTN_X, kR2 = BTN_A, kR3 = BTN_Y, kR4 = BTN_DPAD_DOWN,
                   kL1 = BTN_DPAD_UP, kL3 = BTN_DPAD_LEFT, kL4 = BTN_DPAD_RIGHT;

int main() {
    // ================= Drivetrain =================
    T(1, "neutral input gives zero output", { TeleOp<rig> op; start(op); frame(op); CHECK(wheels(0, 0)); op.stop(); });
    T(2, "forward", { TeleOp<rig> op; start(op); frame(op, 0, 1, 1); CHECK(wheels(100, 100)); op.stop(); });
    T(3, "reverse", { TeleOp<rig> op; start(op); frame(op, 0, 1, -1); CHECK(wheels(-100, -100)); op.stop(); });
    T(4, "left steering", { TeleOp<rig> op; start(op); frame(op, 0, 1, 0, -1); CHECK(wheels(-100, 100)); op.stop(); });
    T(5, "right steering", { TeleOp<rig> op; start(op); frame(op, 0, 1, 0, 1); CHECK(wheels(100, -100)); op.stop(); });
    T(6, "combined throttle and steering", { TeleOp<rig> op; start(op); frame(op, 0, 1, 1, .5f); CHECK(wheels(100, 33)); op.stop(); });
    T(7, "command normalisation", {
        static_assert(mix(1, 1).left == 1 && mix(1, 1).right == 0 && mix(1, -1).left == 0 && mix(-1, -1).left == -1);
        static_assert(mix(.5f, .25f).left == .75f && mix(.5f, .25f).right == .25f);   // untouched inside range
        TeleOp<rig> op; start(op); frame(op, 0, 1, 1, 1); CHECK(wheels(100, 0)); frame(op, 0, 1, -1, 1); CHECK(wheels(0, -100)); op.stop(); });
    T(8, "independent wheel polarities", { TeleOp<rig> op; start(op); frame(op, 0, 1, 1);
        CHECK(fixture::pwmByPin[MOTOR_4_DIR] == -100 && fixture::pwmByPin[MOTOR_R_DIR] == 100);
        CHECK(fixture::pwmByPin[MOTOR_L_DIR] == 100 && fixture::pwmByPin[MOTOR_3_DIR] == -100); op.stop(); });
    T(9, "invalid motor routing rejected", {
        static_assert(with([](Robot& r) { r.drive.wheel[RearLeft].routing = Status::Unverified; }) == Error::DriveRouting);
        static_assert(with([](Robot& r) { r.drive.wheel[RearLeft].channel = Channel::None; }) == Error::DriveChannel);
        static_assert(with([](Robot& r) { r.drive.wheel[RearLeft].socket = 1; }) == Error::DriveSocket);
        static_assert(with([](Robot& r) { r.drive.motor4PinsFreeVerified = false; }) == Error::DriveChannel);
        static_assert(with([](Robot& r) { r.drive.wheel[FrontRight].sign = 0; }) == Error::DrivePolarity); });
    T(10, "duplicate motor channels rejected; proven paired sockets allowed", {
        static_assert(with([](Robot& r) { r.drive.wheel[RearRight].channel = Channel::M4; }) == Error::DriveDuplicate);
        static_assert(with([](Robot& r) { r.drive.pairedSocketsVerified = true; }) == Error::DriveGroup);   // paired board, yet wheels on four outputs
        static_assert(check([] { auto r = paired; r.drive.wheel[RearLeft].sign = -1; return r; }()) == Error::DriveDuplicate);
        static_assert(check([] { auto r = paired; r.drive.pairedSocketsVerified = false; return r; }()) == Error::DriveDuplicate);
        TeleOp<paired> op; const int before = fixture::begins; start(op);
        CHECK(fixture::begins == before + 2);   // two driver outputs, four wheels
        frame(op, 0, 1, 1, .5f);
        CHECK(fixture::pwmByPin[MOTOR_L_DIR] == 100 && fixture::pwmByPin[MOTOR_R_DIR] == -33);
        CHECK(fixture::pwmByPin[MOTOR_3_DIR] == 0 && fixture::pwmByPin[MOTOR_4_DIR] == 0); op.stop(); });
    T(11, "PWM saturation", {
        static_assert(toPwm(1, 0, 100) == 100 && toPwm(5, 0, 100) == 100 && toPwm(-5, 0, 100) == -100);
        static_assert(with([](Robot& r) { r.drive.maxPwm = 256; }) == Error::DrivePwm);
        static_assert(with([](Robot& r) { r.drive.maxPwm = 10; }) == Error::DrivePwm);
        static_assert(with([](Robot& r) { r.drive.minPwm = 100; }) == Error::DrivePwm);
        TeleOp<rig> op; start(op);
        for (float f : {-1.f, -.7f, .3f, 1.f}) for (float t : {-1.f, .4f, 1.f}) { frame(op, 0, 1, f, t);
            for (int p : fixture::pwmByPin) CHECK(p >= -100 && p <= 100); }
        op.stop(); });
    T(12, "deadband", { TeleOp<rig> op; start(op); frame(op, 0, 1, .1f, -.1f); CHECK(wheels(0, 0));
        frame(op, 0, 1, .2f); CHECK(wheels(20, 20));
        static_assert(with([](Robot& r) { r.drive.deadband = 0; }) == Error::DriveDeadband);
        static_assert(with([](Robot& r) { r.drive.deadband = 1; }) == Error::DriveDeadband); op.stop(); });
    T(13, "controller disconnect stops the drive", { TeleOp<rig> op; start(op); frame(op, 0, 1, 1); CHECK(wheels(100, 100));
        fixture::stale = true; op.loop(); CHECK(wheels(0, 0) && op.currentPhase() == Phase::LinkLost);
        fixture::stale = false; frame(op, 0, 1, 1); CHECK(wheels(0, 0)); op.stop(); });
    T(14, "STOP while driving", { TeleOp<rig> op; start(op); frame(op, 0, 1, 1); frame(op, BTN_LT, 1, 1);
        CHECK(wheels(0, 0) && attached == 0 && op.currentPhase() == Phase::Off); frame(op, 0, 1, 1); CHECK(wheels(0, 0)); });
    T(15, "RT while driving", { TeleOp<rig> op; start(op); frame(op, 0, 1, 1); frame(op, BTN_RT, 1, 1);
        CHECK(wheels(0, 0) && op.currentPhase() == Phase::Inhibited); op.stop(); });
    T(16, "initialisation failure at each motor", {
        for (int w = 0; w < WheelCount; ++w) { TeleOp<rig> op; fixture::failMotorPin = pinsOf(rig.drive.wheel[w].channel).dir;
            const int sent = fixture::writes; op.init(); op.start(); frame(op);
            CHECK(op.currentPhase() == Phase::Faulted && attached == 0);
            frame(op); frame(op, 0, 1, 1); CHECK(fixture::writes == sent && wheels(0, 0)); op.stop(); } });
    T(17, "no output from disabled channels", { TeleOp<servosOnly> op; const int b = fixture::begins, w = fixture::writes;
        start(op); frame(op, 0, 1, 1, 1); CHECK(fixture::begins == b && fixture::writes == w); op.stop(); });

    // ================= Claw =================
    T(18, "claw open command", { TeleOp<rig> op; start(op); frame(op, BTN_LB); idle(op, 30); frame(op, BTN_LB); CHECK(angle[SERVO_1] == 85); op.stop(); });
    T(19, "claw close command", { TeleOp<rig> op; start(op); CHECK(angle[SERVO_1] == 85); frame(op, BTN_LB);
        CHECK(angle[SERVO_1] == 120 && op.clawState().commanded == 120 && !op.clawState().settled && op.currentOwner() == Owner::Manual);
        idle(op, 30); CHECK(op.clawState().settled && !ServoState::verified); op.stop(); });
    T(20, "claw toggles once per press", { TeleOp<rig> op; start(op);
        for (int want : {120, 85, 120, 85}) { frame(op, BTN_LB); CHECK(angle[SERVO_1] == want); idle(op, 30); } op.stop(); });
    T(21, "held LB does not repeat", { TeleOp<rig> op; start(op); frame(op, BTN_LB); const int n = writes;
        elapsed(op, 300, BTN_LB); CHECK(writes == n && angle[SERVO_1] == 120); op.stop(); });
    T(22, "LB re-pressed before settling does not re-toggle", { TeleOp<rig> op; start(op); frame(op, BTN_LB); frame(op); const int n = writes;
        frame(op, BTN_LB); CHECK(writes == n && angle[SERVO_1] == 120 && op.currentOwner() == Owner::Safety);
        idle(op, 40); frame(op, BTN_LB); CHECK(angle[SERVO_1] == 85); op.stop(); });
    T(23, "LB during automation cancels instead of fighting it", { TeleOp<rig> op; start(op); r2To(op, AtLowering); const int n = writes;
        frame(op, BTN_LB); CHECK(writes == n && angle[SERVO_1] == 85 && op.currentPhase() == Phase::Settling && op.currentOwner() == Owner::Safety);
        elapsed(op, 800); CHECK(writes == n); op.stop(); });
    T(24, "closed-claw command kept through the lift", { TeleOp<rig> op; start(op); r2To(op, AtRaising); const int n = writes;
        CHECK(angle[SERVO_1] == 120); elapsed(op, 40); CHECK(angle[SERVO_1] == 120 && writes == n && op.clawState().commanded == 120); op.stop(); });

    // ================= Crane =================
    T(25, "crane raise", { TeleOp<rig> op; start(op); frame(op, BTN_RB); idle(op, 40); frame(op, BTN_RB); CHECK(angle[SERVO_2] == 30); op.stop(); });
    T(26, "crane lower", { TeleOp<rig> op; start(op); frame(op, BTN_RB); CHECK(angle[SERVO_2] == 120 && op.craneState().requested == 120); op.stop(); });
    T(27, "held RB does not repeat", { TeleOp<rig> op; start(op); frame(op, BTN_RB); const int n = writes;
        elapsed(op, 300, BTN_RB); CHECK(writes == n && angle[SERVO_2] == 120); op.stop(); });
    T(28, "missing lowered endpoint rejected", {
        static_assert(with([](Robot& r) { r.crane.second = -1; }) == Error::CraneEndpoint);
        static_assert(with([](Robot& r) { r.crane.second = r.crane.high = 121; }) == Error::CraneEndpoint);
        static_assert(configuration.crane.second == 120 && configuration.crane.high == 120); });
    T(29, "invalid travel limits rejected", {
        static_assert(with([](Robot& r) { r.crane.high = 110; }) == Error::Crane);
        static_assert(with([](Robot& r) { r.crane.low = r.crane.high; }) == Error::Crane);
        static_assert(with([](Robot& r) { r.crane.start = 90; }) == Error::Crane);
        static_assert(with([](Robot& r) { r.crane.pulse180 = r.crane.pulse0; }) == Error::Crane);
        static_assert(with([](Robot& r) { r.crane.settleMs = 0; }) == Error::Crane); });
    T(30, "unverified clearance rejected", { static_assert(with([](Robot& r) { r.craneClearanceVerified = false; }) == Error::CraneClearance);
        static_assert(check([] { auto r = rig; r.crane.second = r.crane.high = 87; r.craneClearanceVerified = false; return r; }()) == Error::CraneClearance); });
    T(31, "crane initialisation failure", { TeleOp<rig> op; failing = SERVO_2; const int w = fixture::writes; op.init(); op.start(); frame(op);
        CHECK(op.currentPhase() == Phase::Faulted && attached == 0); frame(op, BTN_LB); frame(op, 0, 1, 1); CHECK(attached == 0 && fixture::writes == w); op.stop(); });
    T(32, "settling interval gates the next command", { TeleOp<rig> op; start(op); frame(op, BTN_RB); elapsed(op, 39);
        CHECK(op.currentPhase() == Phase::Settling && !op.craneState().settled);
        frame(op); CHECK(op.currentPhase() == Phase::Idle && op.craneState().settled); op.stop(); });

    // ================= Payload =================
    T(33, "R1 raises payload", { TeleOp<rig> op; start(op); frame(op, kR4); idle(op, 25); frame(op, kR1); CHECK(angle[SERVO_3] == 70); op.stop(); });
    T(34, "R4 lowers payload", { TeleOp<rig> op; start(op); frame(op, kR4); CHECK(angle[SERVO_3] == 90 && op.payloadState().commanded == 90); op.stop(); });
    T(35, "opposing payload commands rejected", { TeleOp<rig> op; start(op); const int n = writes; frame(op, kR1 | kR4);
        CHECK(writes == n); frame(op, kR4); CHECK(writes == n); frame(op); frame(op, kR4); CHECK(writes == n + 1); op.stop(); });
    T(36, "invalid payload port rejected", {
        static_assert(with([](Robot& r) { r.payload.pin = SERVO_2; }) == Error::PayloadPort);
        static_assert(with([](Robot& r) { r.payload.pin = SERVO_1; }) == Error::PayloadPort);
        static_assert(with([](Robot& r) { r.payload.pin = SERVO_4; }) == Error::Ok); });
    T(37, "missing payload endpoints rejected", {
        static_assert(with([](Robot& r) { r.payload.first = r.payload.second = -1; }) == Error::Payload);
        static_assert(with([](Robot& r) { r.parkedPayload = -1; }) == Error::Payload);
        static_assert(with([](Robot& r) { r.payloadPathAndPowerVerified = false; }) == Error::PayloadPath);
        static_assert(!configuration.payload.enabled && configuration.payload.first == -1); });
    T(38, "crane/payload collision interlock", { TeleOp<rig> op; start(op);
        frame(op, BTN_RB); idle(op, 40); int n = writes; frame(op, kR4); CHECK(writes == n);   // crane down: payload locked
        frame(op); frame(op, BTN_RB); idle(op, 40); frame(op, kR4); idle(op, 25); n = writes;   // payload out:
        frame(op, BTN_RB); frame(op); frame(op, BTN_LB); frame(op); frame(op, kR2); CHECK(writes == n);   // crane, claw, pickup locked
        op.stop(); });

    // ================= R2 automatic collection =================
    T(39, "R2 valid start", { TeleOp<rig> op; start(op); frame(op, kR2);
        CHECK(angle[SERVO_2] == 120 && angle[SERVO_1] == 85 && op.currentPhase() == Phase::Lowering && op.currentOwner() == Owner::Pickup); op.stop(); });
    T(40, "R2 invalid starts rejected", { TeleOp<rig> op; start(op);
        frame(op, BTN_LB); idle(op, 30); int n = writes; frame(op, kR2); CHECK(writes == n);        // claw closed
        frame(op); frame(op, BTN_LB); idle(op, 30); frame(op, BTN_RB); idle(op, 40); n = writes;
        frame(op, kR2); CHECK(writes == n);                                                          // crane already lowered
        frame(op); frame(op, BTN_RB); idle(op, 40); n = writes;
        frame(op, kR2, 1, 1); CHECK(writes == n && wheels(0, 0)); op.stop(); });                    // stick not neutral
    T(41, "R2 lowering stage waits the calibrated interval", { TeleOp<rig> op; start(op); frame(op, kR2); elapsed(op, 39);
        CHECK(op.currentPhase() == Phase::Lowering && angle[SERVO_1] == 85 && !op.craneState().settled);
        frame(op); CHECK(op.currentPhase() == Phase::Dwell && op.craneState().settled); op.stop(); });
    T(42, "R2 dwell: claw still open 499 ms after lowering completed", { TeleOp<rig> op; start(op); r2To(op, AtDwell); elapsed(op, 499);
        CHECK(op.currentPhase() == Phase::Dwell && angle[SERVO_1] == 85); op.stop(); });
    T(43, "R2 dwell: claw closes at 500 ms", { TeleOp<rig> op; start(op); r2To(op, AtDwell); elapsed(op, 499); frame(op);
        CHECK(op.currentPhase() == Phase::Gripping && angle[SERVO_1] == 120); op.stop(); });
    T(44, "R2 claw closes with crane still lowered", { TeleOp<rig> op; start(op); r2To(op, AtGripping);
        CHECK(angle[SERVO_1] == 120 && angle[SERVO_2] == 120 && !op.clawState().settled); op.stop(); });
    T(45, "R2 grip settling before the lift", { TeleOp<rig> op; start(op); r2To(op, AtGripping); elapsed(op, 29); CHECK(angle[SERVO_2] == 120);
        frame(op); CHECK(angle[SERVO_2] == 30 && op.currentPhase() == Phase::Raising && op.clawState().settled); op.stop(); });
    T(46, "R2 raising stage and completion", { TeleOp<rig> op; start(op); r2To(op, AtRaising); elapsed(op, 39); CHECK(op.currentPhase() == Phase::Raising);
        frame(op); CHECK(op.currentPhase() == Phase::Idle && angle[SERVO_1] == 120 && angle[SERVO_2] == 30 && wheels(0, 0)); op.stop(); });
    T(47, "R2 cancellation at every stage (RT and stick)", {
        for (int s = 0; s < StageCount; ++s) {
            { TeleOp<rig> op; start(op); r2To(op, s); const int n = writes; frame(op, BTN_RT);
              CHECK(op.currentPhase() == Phase::Inhibited && writes == n && wheels(0, 0)); elapsed(op, 800); CHECK(writes == n); op.stop(); }
            { TeleOp<rig> op; start(op); r2To(op, s); const int n = writes; frame(op, 0, 1, 1);
              CHECK(op.currentOwner() == Owner::Safety && writes == n && wheels(0, 0));
              elapsed(op, 800, 0, 1); CHECK(writes == n && wheels(0, 0) && op.currentPhase() == Phase::Idle);
              frame(op); frame(op, 0, 1, 1); CHECK(wheels(100, 100)); op.stop(); }
        } });
    T(48, "R2 total timeout", { TeleOp<slowLink> op; start(op); r2To(op, AtDwell); frame(op, 0, 400); CHECK(op.currentPhase() == Phase::Dwell);
        const int n = writes; frame(op, 0, 400); CHECK(op.currentPhase() == Phase::Faulted && writes == n);
        elapsed(op, 800); frame(op, kR2); CHECK(writes == n); op.stop(); });
    T(49, "controller loss during R2", {
        for (int s = 0; s < StageCount; ++s) { TeleOp<rig> op; start(op); r2To(op, s); const int n = writes;
            fixture::stale = true; op.loop(); fixture::stale = false;
            CHECK(op.currentPhase() == Phase::LinkLost && writes == n && attached == 3 && wheels(0, 0));
            frame(op); idle(op, 40); CHECK(op.currentPhase() == Phase::Idle && writes == n); elapsed(op, 800); CHECK(writes == n); op.stop(); }
        TeleOp<dropOnLink> op; start(op); r2To(op, AtLowering); fixture::stale = true; op.loop(); fixture::stale = false;
        CHECK(op.currentPhase() == Phase::Faulted && attached == 0); op.stop(); });
    T(50, "STOP during R2 at every stage", {
        for (int s = 0; s < StageCount; ++s) { TeleOp<rig> op; start(op); r2To(op, s); const int n = writes;
            if (s & 1) frame(op, BTN_LT); else op.stop();
            CHECK(attached == 0 && op.currentPhase() == Phase::Off && wheels(0, 0)); elapsed(op, 800); CHECK(writes == n); } });

    // ================= R3 quick pickup =================
    T(51, "R3 valid lowered start", { TeleOp<rig> op; start(op); frame(op, BTN_RB); idle(op, 40); frame(op, kR3);
        CHECK(op.currentPhase() == Phase::Gripping && angle[SERVO_2] == 120); op.stop(); });
    T(52, "R3 rejected with crane raised", { TeleOp<rig> op; start(op); const int n = writes; frame(op, kR3);
        CHECK(writes == n && op.currentPhase() == Phase::Idle); op.stop(); });
    T(53, "R3 rejected while crane state is unsettled", { TeleOp<rig> op; start(op); frame(op, BTN_RB); frame(op); const int n = writes;
        frame(op, kR3); CHECK(writes == n && angle[SERVO_1] == 85 && op.currentPhase() != Phase::Gripping); op.stop(); });
    T(54, "R3 closes the claw at once, no dwell", { TeleOp<rig> op; start(op); frame(op, BTN_RB); idle(op, 40); const int n = writes; frame(op, kR3);
        CHECK(angle[SERVO_1] == 120 && writes == n + 1); op.stop(); });
    T(55, "R3 grip settling", { TeleOp<rig> op; start(op); frame(op, BTN_RB); idle(op, 40); frame(op, kR3); elapsed(op, 29); CHECK(angle[SERVO_2] == 120);
        frame(op); CHECK(angle[SERVO_2] == 30); op.stop(); });
    T(56, "R3 raises and completes", { TeleOp<rig> op; start(op); frame(op, BTN_RB); idle(op, 40); frame(op, kR3); elapsed(op, 30); elapsed(op, 40);
        CHECK(op.currentPhase() == Phase::Idle && angle[SERVO_1] == 120 && angle[SERVO_2] == 30); op.stop(); });
    T(57, "R3 cancellation keeps the claw closed", { TeleOp<rig> op; start(op); frame(op, BTN_RB); idle(op, 40); frame(op, kR3); const int n = writes;
        frame(op, 0, 1, 1); elapsed(op, 800); CHECK(writes == n && angle[SERVO_1] == 120 && angle[SERVO_2] == 120); op.stop(); });
    T(58, "R3 timeout", { TeleOp<slowLink> op; start(op); frame(op, BTN_RB); idle(op, 40); frame(op, kR3); frame(op, 0, 400); frame(op, 0, 400);
        CHECK(op.currentPhase() == Phase::Faulted); op.stop(); });

    // ================= Turning =================
    T(59, "L1 180-degree request", { TeleOp<rig> op; start(op); frame(op, kL1); CHECK(wheels(50, -50) && op.currentOwner() == Owner::Turn);
        elapsed(op, 159); CHECK(wheels(50, -50)); frame(op); CHECK(wheels(0, 0) && op.currentPhase() == Phase::Idle); op.stop(); });
    T(60, "L3 right 90 request", { TeleOp<rig> op; start(op); frame(op, kL3); CHECK(wheels(50, -50)); elapsed(op, 79); CHECK(wheels(50, -50));
        frame(op); CHECK(wheels(0, 0)); op.stop(); });
    T(61, "L4 left 90 request uses its own calibration", { TeleOp<rig> op; start(op); frame(op, kL4); CHECK(wheels(-50, 50)); elapsed(op, 84); CHECK(wheels(-50, 50));
        frame(op); CHECK(wheels(0, 0)); op.stop(); });
    T(62, "missing turn calibration", {
        static_assert(with([](Robot& r) { r.turns.rightMs = 0; }) == Error::TurnCalibration);
        static_assert(with([](Robot& r) { r.turns.leftMs = 300; }) == Error::TurnCalibration);
        static_assert(with([](Robot& r) { r.turns.pwm = 101; }) == Error::Turn);
        static_assert(with([](Robot& r) { r.turns.halfTurnSign = 0; }) == Error::Turn);
        static_assert(check([] { auto r = gyro; r.turns.headingRightSign = 0; return r; }()) == Error::TurnCalibration);
        TeleOp<noRound> op; start(op, 2); frame(op, kL3); CHECK(wheels(0, 0)); op.stop(); });
    T(63, "heading wraparound", { TeleOp<gyro> op; start(op); NRLComms::heading = 170; frame(op, kL3); CHECK(wheels(50, -50));
        NRLComms::heading = -145; frame(op); CHECK(wheels(50, -50)); NRLComms::heading = -101; frame(op); CHECK(wheels(0, 0)); op.stop(); });
    T(64, "heading loss stops the turn", { TeleOp<gyro> op; start(op); frame(op, kL4); CHECK(wheels(-50, 50));
        NRLComms::headingReady = false; frame(op); CHECK(wheels(0, 0) && op.currentPhase() == Phase::Faulted); op.stop();
        TeleOp<gyro> cold; start(cold); NRLComms::headingReady = false; frame(cold, kL3); CHECK(wheels(0, 0) && cold.currentPhase() == Phase::Faulted); cold.stop(); });
    T(65, "turn timeout", { TeleOp<gyro> op; start(op); frame(op, kL1); elapsed(op, 299); CHECK(wheels(50, -50));
        frame(op); CHECK(wheels(0, 0) && op.currentPhase() == Phase::Faulted); op.stop();
        TeleOp<slowLink> timed; start(timed); frame(timed, kL1); frame(timed, 0, 400); CHECK(wheels(0, 0) && timed.currentPhase() == Phase::Faulted); timed.stop(); });
    T(66, "manual override cancels a turn", { TeleOp<rig> op; start(op); frame(op, kL3); frame(op, 0, 1, 0, -1);
        CHECK(wheels(0, 0)); elapsed(op, 60, 0, 1); CHECK(wheels(0, 0)); frame(op); frame(op, 0, 1, 1); CHECK(wheels(100, 100)); op.stop(); });
    T(67, "RT cancels a turn", { TeleOp<rig> op; start(op); frame(op, kL1); frame(op, BTN_RT); CHECK(wheels(0, 0) && op.currentPhase() == Phase::Inhibited);
        elapsed(op, 300); CHECK(wheels(0, 0)); op.stop(); });
    T(68, "STOP during a turn", { TeleOp<rig> op; start(op); frame(op, kL1); frame(op, BTN_LT); CHECK(wheels(0, 0) && attached == 0 && op.currentPhase() == Phase::Off); });

    // ================= RT cancel and safe stow =================
    T(69, "RT while idle stows crane, then payload, then claw", { TeleOp<calibrated> op; start(op); const int n = writes; frame(op, BTN_RT);
        CHECK(angle[SERVO_2] == 60 && writes == n + 1 && op.currentOwner() == Owner::Stow);
        elapsed(op, 40, BTN_RT); CHECK(angle[SERVO_3] == 75 && writes == n + 2);
        elapsed(op, 25, BTN_RT); CHECK(angle[SERVO_1] == 120 && writes == n + 3);
        elapsed(op, 30, BTN_RT); CHECK(op.currentPhase() == Phase::AwaitNeutral); op.stop(); });
    T(70, "RT during manual driving", { TeleOp<calibrated> op; start(op); frame(op, 0, 1, 1); CHECK(wheels(100, 100));
        frame(op, BTN_RT, 1, 1); CHECK(wheels(0, 0) && angle[SERVO_2] == 60); elapsed(op, 200, BTN_RT, 1); CHECK(wheels(0, 0)); op.stop(); });
    T(71, "RT during R2", { TeleOp<calibrated> op; start(op); r2To(op, AtDwell); frame(op, BTN_RT); CHECK(angle[SERVO_2] == 60 && op.currentPhase() == Phase::Stow);
        elapsed(op, 200, BTN_RT); CHECK(op.currentPhase() == Phase::AwaitNeutral && angle[SERVO_3] == 75); op.stop(); });
    T(72, "RT during R3 leaves the closed claw alone", { TeleOp<calibrated> op; start(op); frame(op, BTN_RB); idle(op, 40); frame(op, kR3); const int n = writes;
        frame(op, BTN_RT); elapsed(op, 200, BTN_RT); CHECK(writes == n + 2 && angle[SERVO_1] == 120 && angle[SERVO_2] == 60); op.stop(); });
    T(73, "RT during turning", { TeleOp<calibrated> op; start(op); frame(op, kL3); CHECK(wheels(50, -50)); frame(op, BTN_RT);
        CHECK(wheels(0, 0) && angle[SERVO_2] == 60); op.stop(); });
    T(74, "RT during payload movement retargets the payload first", { TeleOp<calibrated> op; start(op); frame(op, kR4); const int n = writes; frame(op, BTN_RT);
        CHECK(angle[SERVO_3] == 75 && angle[SERVO_2] == 30 && writes == n + 1); elapsed(op, 25, BTN_RT); CHECK(angle[SERVO_2] == 60); op.stop(); });
    T(75, "RT held continuously does not retrigger", { TeleOp<calibrated> op; start(op); frame(op, BTN_RT); elapsed(op, 100, BTN_RT); const int n = writes;
        elapsed(op, 500, BTN_RT); CHECK(writes == n && op.currentPhase() == Phase::AwaitNeutral); op.stop(); });
    T(76, "repeated RT", { TeleOp<calibrated> op; start(op); frame(op, BTN_RT); frame(op); frame(op, BTN_RT); frame(op); int n = writes;
        elapsed(op, 10, BTN_RT); CHECK(writes == n);                 // toggling mid-stow restarts nothing
        elapsed(op, 100); frame(op); CHECK(op.currentPhase() == Phase::Idle); n = writes;
        frame(op, BTN_RT); CHECK(writes == n + 1 && op.currentPhase() == Phase::Stow); op.stop(); });
    T(77, "missing stow calibration: inhibit, no servo travel", { TeleOp<rig> op; start(op); frame(op, kR4); const int n = writes; frame(op, BTN_RT);
        CHECK(writes == n && attached == 3 && op.currentPhase() == Phase::Inhibited);
        frame(op); frame(op, 0, 1, 1); frame(op); frame(op, BTN_RB); frame(op); frame(op, BTN_RT); CHECK(writes == n && wheels(0, 0) && attached == 3); op.stop();
        TeleOp<dropLiftOnCancel> drop; start(drop); frame(drop, BTN_RT); CHECK(attached == 1 && drop.currentPhase() == Phase::Inhibited); drop.stop(); });
    T(78, "invalid stow targets rejected", {
        static_assert(withStow([](Robot& r) { r.stow.craneRest = 121; }) == Error::Stow);
        static_assert(withStow([](Robot& r) { r.stow.payloadRest = -1; }) == Error::Stow);
        static_assert(withStow([](Robot& r) { r.stow.gripRest = 80; }) == Error::Stow);
        static_assert(withStow([](Robot& r) { r.stow.permissivePin = -1; }) == Error::Stow); });
    T(79, "inputs during stow cannot interrupt or redirect it", { TeleOp<calibrated> op; start(op); const int n = writes; frame(op, BTN_RT);
        frame(op, BTN_LB, 1, 1); frame(op, kR2); elapsed(op, 120, kL3, 1); CHECK(writes == n + 3 && wheels(0, 0) && angle[SERVO_1] == 120); op.stop(); });
    T(80, "controller loss during stow", { TeleOp<calibrated> op; start(op); frame(op, BTN_RT); const int n = writes;
        fixture::stale = true; op.loop(); fixture::stale = false; CHECK(op.currentPhase() == Phase::LinkLost);
        elapsed(op, 200, BTN_RT); CHECK(writes == n); frame(op); idle(op, 40); CHECK(op.currentPhase() == Phase::Idle && writes == n); op.stop(); });
    T(81, "STOP during stow", { TeleOp<calibrated> op; start(op); frame(op, BTN_RT); const int n = writes; frame(op, BTN_RT | BTN_LT);
        CHECK(attached == 0 && op.currentPhase() == Phase::Off); elapsed(op, 200); CHECK(writes == n); });
    T(82, "neutral re-arming after stow", { TeleOp<calibrated> op; start(op); frame(op, BTN_RT); elapsed(op, 100, BTN_RT);
        frame(op, 0, 1, 1); CHECK(wheels(0, 0) && op.currentPhase() == Phase::AwaitNeutral); frame(op, kR2); CHECK(op.currentPhase() == Phase::AwaitNeutral);
        frame(op); CHECK(op.currentPhase() == Phase::Idle); frame(op, 0, 1, 1); CHECK(wheels(100, 100)); op.stop(); });
    T(83, "no unintended claw release", {
        { TeleOp<calibrated> op; start(op); frame(op, BTN_LB); idle(op, 30); frame(op, BTN_RT); elapsed(op, 300, BTN_RT); CHECK(angle[SERVO_1] == 120 && attached == 3); op.stop(); }
        { TeleOp<rig> op; start(op); frame(op, BTN_LB); idle(op, 30); frame(op, BTN_RT); elapsed(op, 300); CHECK(angle[SERVO_1] == 120 && attached == 3); op.stop(); }
        { TeleOp<rig> op; start(op); frame(op, BTN_LB); idle(op, 30); fixture::stale = true; op.loop(); fixture::stale = false; CHECK(angle[SERVO_1] == 120 && attached == 3); op.stop(); } });
    T(84, "no automatic restart after interruption", { TeleOp<calibrated> op; start(op); r2To(op, AtLowering); frame(op, BTN_RT | kR2); elapsed(op, 200, kR2);
        const int n = writes; elapsed(op, 800, kR2); CHECK(writes == n && op.currentPhase() == Phase::AwaitNeutral);
        frame(op); elapsed(op, 800); CHECK(writes == n && op.currentPhase() == Phase::Idle); op.stop(); });

    // ================= General =================
    T(85, "duplicate and unsupported button masks rejected", {
        static_assert(with([](Robot& r) { r.map.slot[L3].mask = r.map.slot[R2].mask; }) == Error::MapDuplicate);
        static_assert(with([](Robot& r) { r.map.slot[R2].mask = BTN_RT; }) == Error::MapMask);
        static_assert(with([](Robot& r) { r.map.slot[R2].mask = BTN_LB; }) == Error::MapMask);
        static_assert(with([](Robot& r) { r.map.slot[R2].mask = 1 << 2; }) == Error::MapMask);
        static_assert(with([](Robot& r) { r.map.slot[R2].mask = BTN_X | BTN_Y; }) == Error::MapMask);
        for (int a = 0; a < SlotCount; ++a) for (int b = 0; b < a; ++b) { auto r = configuration;
            r.map.slot[a].mask = r.map.slot[b].mask = BTN_Y; CHECK(check(r) == Error::MapDuplicate); } });
    T(86, "unassigned and unverified buttons never act", { TeleOp<noRound> op; start(op, 2); const int n = writes, w = fixture::writes;
        for (unsigned bit : {BTN_X, BTN_A, BTN_Y, BTN_DPAD_DOWN, BTN_DPAD_UP, BTN_DPAD_LEFT, BTN_DPAD_RIGHT}) { frame(op, bit); frame(op); }
        CHECK(writes == n && wheels(0, 0) && op.currentPhase() == Phase::Idle && fixture::writes >= w);
        frame(op, BTN_RB); CHECK(angle[SERVO_2] == 120);   // bumpers work without any round button
        static_assert(key(noRound.map, R2) == 0 && key(configuration.map, R1) == 0); op.stop(); });
    T(87, "invalid configurations rejected with a specific reason", {
        static_assert(with([](Robot& r) { r.map.slot[L4].status = Status::Unverified; }) == Error::MapIncomplete);
        static_assert(with([](Robot& r) { r.map.fullLayoutEnabled = false; r.map.slot[L4].status = Status::Unverified; }) == Error::MapUnverified);
        static_assert(with([](Robot& r) { r.map.fullLayoutEnabled = false; r.map.slot[R1].mask = 0; r.map.slot[R1].status = Status::Unverified; }) == Error::MapUnverified);
        static_assert(with([](Robot& r) { r.map.slot[R1].mask = 0; }) == Error::MapMask);
        static_assert(with([](Robot& r) { r.team.channel = 3; }) == Error::TeamChannel);
        static_assert(with([](Robot& r) { r.team.number = 0; }) == Error::TeamChannel);
        static_assert(with([](Robot& r) { r.safety.responsesVerified = false; }) == Error::ServoResponse);
        static_assert(with([](Robot& r) { r.drive.driverCapacityVerified = false; }) == Error::DriveCapacity);
        static_assert(with([](Robot& r) { r.drive.throttleSign = 0; }) == Error::DriveAxis);
        static_assert(with([](Robot& r) { r.grip.second = 121; r.grip.high = 121; }) == Error::Grip);
        static_assert(with([](Robot& r) { r.pickup.timingAndGeometryVerified = false; }) == Error::Pickup);
        static_assert(with([](Robot& r) { r.pickup.timeoutMs = 610; }) == Error::PickupTiming);
        static_assert(with([](Robot& r) { r.pickup.dwellMs = 499; }) == Error::PickupTiming);
        static_assert(with([](Robot& r) { r.drive.enabled = false; }) == Error::Turn);
        static_assert(with([](Robot& r) { r.drive.wheelCheckEnabled = true; r.drive.wheelCheckPwm = 40; }) == Error::WheelCheck);
        static_assert(check([] { auto r = wheelTool; r.drive.wheelCheckPwm = 65; return r; }()) == Error::WheelCheck);
        static_assert(check([] { auto r = wheelTool; r.drive.wheel[FrontLeft].routing = Status::Unverified; return r; }()) == Error::WheelCheck); });
    T(88, "timer rollover", { fixture::timeMs = std::numeric_limits<uint32_t>::max() - 100; TeleOp<rig> op; start(op); frame(op, kR2);
        elapsed(op, 40); elapsed(op, 499); CHECK(angle[SERVO_1] == 85); CHECK(fixture::timeMs < 1000);   // wrapped mid-routine
        frame(op); CHECK(angle[SERVO_1] == 120); elapsed(op, 30); elapsed(op, 40); CHECK(op.currentPhase() == Phase::Idle);
        frame(op); frame(op, 0, 1, 1); CHECK(wheels(100, 100)); op.stop(); });
    T(89, "stale controller packets", { TeleOp<rig> op; start(op); const int n = writes;
        tickNoPacket(op, BTN_LB); CHECK(writes == n);               // a button without a fresh packet is not a press
        for (int i = 0; i < 47; ++i) tickNoPacket(op); CHECK(op.currentPhase() == Phase::Idle);
        tickNoPacket(op, 0, 2); CHECK(op.currentPhase() == Phase::LinkLost); op.stop(); });
    T(90, "conflicting actions rejected", { TeleOp<rig> op; start(op); const int n = writes;
        frame(op, kL3 | kL4); CHECK(wheels(0, 0)); frame(op, 0, 1, 1); CHECK(wheels(0, 0));   // no jump into driving
        frame(op); frame(op, BTN_LB | BTN_RB); frame(op, BTN_RB); CHECK(writes == n);
        frame(op); frame(op, 0, 1, 1); CHECK(wheels(100, 100)); frame(op, BTN_LB, 1, 1); CHECK(wheels(0, 0) && writes == n); op.stop(); });
    T(91, "failed initialisation enables nothing", { TeleOp<rig> op; failing = SERVO_1; const int b = fixture::begins; op.init(); op.start(); frame(op);
        CHECK(op.currentPhase() == Phase::Faulted && attached == 0 && fixture::begins == b); op.stop(); });
    T(92, "partial initialisation is rolled back", { TeleOp<rig> op; failing = SERVO_3; const int b = fixture::begins, n = writes; op.init(); op.start(); frame(op);
        CHECK(attached == 0 && fixture::begins == b && op.currentPhase() == Phase::Faulted);
        failing = -1; frame(op); frame(op, BTN_LB); frame(op, 0, 1, 1); CHECK(attached == 0 && writes == n && wheels(0, 0)); op.stop(); });
    T(93, "delivered program: payload, pickups, turns and round buttons never act", { const int b = begins, m = fixture::begins, s = fixture::writes;
        InputMonitor mon; mon.init(); mon.start(); frame(mon, 0xFFF, 1, 1, 1); mon.stop();
        HyperdriveWheelCheck wc; wc.init(); wc.start(); frame(wc); frame(wc, BTN_LB); frame(wc, BTN_RB); wc.stop();   // wheel check is off in this build
        CHECK(begins == b && fixture::begins == m && fixture::writes == s);
        HyperdriveTeleOp op; op.init(); op.start(); frame(op); CHECK(begins == b + 2 && attached == 2);   // claw and crane only; no third servo
        elapsed(op, 6000); frame(op); frame(op); const int w = writes;
        for (unsigned k : {0x001u, 0x002u, 0x008u, 0x010u, 0x020u, 0x040u, 0x080u}) { frame(op, k); frame(op); CHECK(op.currentPhase() == Phase::Idle); }
        CHECK(writes == w && begins == b + 2 && fixture::pwmByPin[MOTOR_L_DIR] == 0 && fixture::pwmByPin[MOTOR_R_DIR] == 0);
        op.stop();
        static_assert(!configuration.payload.enabled && !configuration.pickup.enabled && !configuration.drive.wheelCheckEnabled &&
                      configuration.turns.mode == TurnMode::Disabled && !stowReady(configuration));
        for (int slot = 0; slot < SlotCount; ++slot) CHECK(key(configuration.map, slot) == 0); });
    T(94, "recovery from link loss, repeatedly, only via fresh neutral", { TeleOp<rig> op; start(op);
        for (int i = 0; i < 3; ++i) { frame(op, 0, 1, 1); CHECK(wheels(100, 100));
            fixture::stale = true; op.loop(); op.loop(); fixture::stale = false; CHECK(wheels(0, 0));
            frame(op, 0, 1, 1); frame(op, BTN_LB); CHECK(op.currentPhase() == Phase::LinkLost && wheels(0, 0));   // held inputs do not re-arm
            frame(op); idle(op, 40); CHECK(op.currentPhase() == Phase::Idle); }
        op.stop(); });
    T(95, "safety priority ordering within one tick", {
        { TeleOp<calibrated> op; start(op); frame(op, 0, 1, 1); const int n = writes; fixture::stale = true; fixture::upperOpen = true;
          frame(op, BTN_LT | BTN_RT | BTN_LB, 1, 1); CHECK(op.currentPhase() == Phase::Off && attached == 0 && writes == n && wheels(0, 0));
          fixture::stale = false; fixture::upperOpen = false; }      // STOP first
        { TeleOp<calibrated> op; start(op); const int n = writes; fixture::stale = true; frame(op, BTN_RT | BTN_LB, 1, 1);
          CHECK(op.currentPhase() == Phase::LinkLost && writes == n); fixture::stale = false; op.stop(); }                                       // link before RT
        { TeleOp<calibrated> op; start(op); const int n = writes; fixture::upperOpen = true; frame(op, BTN_RT);
          CHECK(op.currentPhase() == Phase::Faulted && writes == n && std::string_view(op.fault()) == "safety input"); fixture::upperOpen = false; op.stop(); }   // fault before RT
        { TeleOp<calibrated> op; start(op); power.low = true; frame(op, 0, 1, 1); CHECK(op.currentPhase() == Phase::Faulted && wheels(0, 0)); power.low = false; op.stop(); }
        { TeleOp<calibrated> op; start(op); frame(op, BTN_RT | BTN_LB, 1, 1);
          CHECK(op.currentPhase() == Phase::Stow && angle[SERVO_1] == 85 && wheels(0, 0)); op.stop(); }                                         // RT before manual
        { TeleOp<rig> op; start(op); frame(op, kL3); frame(op, kL3 | BTN_LB); CHECK(wheels(0, 0) && angle[SERVO_1] == 85); op.stop(); } });      // automation before manual
    T(96, "safe shutdown from every state", {
        for (int s = 0; s < 6; ++s) { TeleOp<calibrated> op; start(op);
            if (s == 1) frame(op, 0, 1, 1); if (s == 2) r2To(op, AtGripping); if (s == 3) frame(op, kL1);
            if (s == 4) frame(op, BTN_RT); if (s == 5) { fixture::stale = true; op.loop(); fixture::stale = false; }
            const int n = writes; op.stop(); CHECK(attached == 0 && wheels(0, 0) && op.currentPhase() == Phase::Off);
            op.stop(); frame(op, BTN_LB, 1, 1); elapsed(op, 100, 0, 1); CHECK(attached == 0 && writes == n && wheels(0, 0)); } });

    // ================= Extras =================
    X(1, "manual acceleration limit; safety stops are immediate", { TeleOp<slewed> op; start(op); frame(op, 0, 10, 1); CHECK(wheels(10, 10));
        frame(op, 0, 10, 1); CHECK(wheels(20, 20)); elapsed(op, 100, 0, 1); CHECK(wheels(100, 100));
        frame(op, 0, 10); CHECK(wheels(90, 90));                 // controlled deceleration on release
        frame(op, BTN_RT, 1, 1); CHECK(wheels(0, 0)); op.stop(); });
    X(2, "measured PWM floor mapping", { static_assert(toPwm(0, 20, 100) == 0 && toPwm(.5f, 20, 100) == 60 && toPwm(-1, 20, 100) == -100 && toPwm(.01f, 20, 100) == 21); });
    X(3, "wheel check spins one routed wheel under a dead-man", { WheelCheck<wheelTool> op; const int b = fixture::begins; op.init(); op.start(); frame(op, BTN_LB);
        CHECK(fixture::begins == b);                              // LB held at start: no arming
        frame(op); CHECK(fixture::begins == b + 1 && op.selectedWheel() == FrontLeft);
        frame(op, BTN_LB); CHECK(fixture::pwmByPin[MOTOR_L_DIR] == 40);
        for (int p : {MOTOR_R_DIR, MOTOR_3_DIR, MOTOR_4_DIR}) CHECK(fixture::pwmByPin[p] == 0);
        frame(op, BTN_LB | BTN_X); CHECK(fixture::pwmByPin[MOTOR_L_DIR] == 0);
        frame(op, BTN_LB); CHECK(fixture::pwmByPin[MOTOR_L_DIR] == 40); frame(op); CHECK(fixture::pwmByPin[MOTOR_L_DIR] == 0);
        frame(op, BTN_RB); CHECK(op.selectedWheel() == FrontLeft);   // unrouted wheels cannot be selected
        frame(op, BTN_LB); fixture::stale = true; op.loop(); fixture::stale = false; CHECK(fixture::pwmByPin[MOTOR_L_DIR] == 0);
        frame(op, BTN_LB); CHECK(fixture::pwmByPin[MOTOR_L_DIR] == 0);   // must pass through neutral again
        frame(op); frame(op, BTN_LB); CHECK(fixture::pwmByPin[MOTOR_L_DIR] == 40);
        frame(op, BTN_LB | BTN_RT); CHECK(fixture::pwmByPin[MOTOR_L_DIR] == 0); frame(op, BTN_LB); CHECK(fixture::pwmByPin[MOTOR_L_DIR] == 0); });
    X(5, "board motor groups: two paired outputs or four separate ones, never a mix", {
        static_assert(logicalOutputs(paired.drive) == 2 && logicalOutputs(rig.drive) == 4 && logicalOutputs(configuration.drive) == 2);
        static_assert(configuration.drive.group[RearRight] == 2 && configuration.drive.group[FrontRight] == 2);
        static_assert(configuration.drive.group[FrontLeft] == 1 && configuration.drive.group[RearLeft] == 1);
        static_assert(configuration.drive.groupObserved[RearRight] == Status::Verified && configuration.drive.groupObserved[FrontRight] == Status::Verified);
        // Delivered routing: two paired groups on the two onboard outputs, no polarity, still disabled.
        static_assert(configuration.drive.pairedSocketsVerified);
        static_assert(configuration.drive.wheel[FrontLeft].channel == Channel::L && configuration.drive.wheel[RearLeft].channel == Channel::L);
        static_assert(configuration.drive.wheel[FrontRight].channel == Channel::R && configuration.drive.wheel[RearRight].channel == Channel::R);
        for (const auto& w : configuration.drive.wheel) CHECK(w.routing == Status::Verified);
        static_assert(configuration.drive.wheel[FrontLeft].sign == -1 && configuration.drive.wheel[RearLeft].sign == -1);    // provisional, from the factory test
        static_assert(configuration.drive.wheel[FrontRight].sign == 1 && configuration.drive.wheel[RearRight].sign == 1);
        static_assert(check([] { auto r = configuration; r.drive.wheelCheckEnabled = true; r.drive.wheelCheckPwm = 25; return r; }()) == Error::WheelCheck);   // wheel check and drive never share a build
        TeleOp<paired> op; const int b = fixture::begins; start(op); CHECK(fixture::begins == b + 2);   // exactly one object per GPIO pair
        frame(op, 0, 1, 0, 1); CHECK(fixture::pwmByPin[MOTOR_L_DIR] == 100 && fixture::pwmByPin[MOTOR_R_DIR] == 100);   // right turn: groups oppose, R flipped
        frame(op, kL3 & 0); frame(op); frame(op, kL3); CHECK(fixture::pwmByPin[MOTOR_L_DIR] == 50 && fixture::pwmByPin[MOTOR_R_DIR] == 50);   // automatic turn on two outputs
        frame(op, BTN_RT); CHECK(fixture::pwmByPin[MOTOR_L_DIR] == 0 && fixture::pwmByPin[MOTOR_R_DIR] == 0);
        CHECK(fixture::pwmByPin[MOTOR_3_DIR] == 0 && fixture::pwmByPin[MOTOR_4_DIR] == 0); op.stop(); });
    X(6, "turn direction follows the robot, not the wheel labels", {
        static_assert(with([](Robot& r) { r.turns.rightTurnSign = 0; }) == Error::TurnDirection);
        static_assert(with([](Robot& r) { r.turns.rightTurnSign = 2; }) == Error::TurnDirection);
        static_assert(configuration.turns.rightTurnSign == 0 && configuration.turns.mode == TurnMode::Disabled);
        { TeleOp<mirrored> op; start(op); frame(op, kL3); CHECK(wheels(-50, 50)); elapsed(op, 79); CHECK(wheels(-50, 50)); frame(op); CHECK(wheels(0, 0)); op.stop(); }   // right 90: own duration
        { TeleOp<mirrored> op; start(op); frame(op, kL4); CHECK(wheels(50, -50)); elapsed(op, 84); CHECK(wheels(50, -50)); frame(op); CHECK(wheels(0, 0)); op.stop(); }   // left 90: own duration
        { TeleOp<mirrored> op; start(op); frame(op, kL1); CHECK(wheels(-50, 50)); frame(op, BTN_RT); CHECK(wheels(0, 0)); op.stop(); }   // 180 with halfTurnSign = right
        { TeleOp<mirroredGyro> op; start(op); NRLComms::heading = 170; frame(op, kL3); CHECK(wheels(-50, 50));   // heading progress still counted as a right turn
          NRLComms::heading = -145; frame(op); CHECK(wheels(-50, 50)); NRLComms::heading = -101; frame(op); CHECK(wheels(0, 0)); op.stop(); }
        { TeleOp<mirrored> op; start(op); frame(op, 0, 1, 0, 1); CHECK(wheels(100, -100)); op.stop(); } });   // manual steering untouched: it uses steeringSign
    X(7, "radio identity: channel 1 verified, team number still a placeholder", {
        static_assert(configuration.team.channel == 1 && configuration.team.channelVerified && !configuration.team.confirmed);
        static_assert(with([](Robot& r) { r.team.channel = 2; }) == Error::TeamChannel);
        static_assert(with([](Robot& r) { r.team.channel = 6; }) == Error::Ok && with([](Robot& r) { r.team.channel = 11; }) == Error::Ok); });
    X(8, "wheel-check variant of the delivered routing: one group at PWM 25 under a dead-man", {
        static constexpr Robot wheelDiag = [] { auto r = configuration; r.drive.enabled = false; r.drive.wheelCheckEnabled = true; r.drive.wheelCheckPwm = 25; return r; }();
        static_assert(valid(wheelDiag));
        const int servoBegins = begins, servoWrites = writes, m = fixture::begins;
        WheelCheck<wheelDiag> wc; wc.init(); wc.start(); frame(wc, BTN_LB); CHECK(fixture::begins == m);   // LB held at START: stays unarmed
        frame(wc); CHECK(fixture::begins == m + 2 && wc.selectedWheel() == FrontLeft);                    // two outputs, never four
        for (int p : {MOTOR_L_DIR, MOTOR_R_DIR, MOTOR_3_DIR, MOTOR_4_DIR}) CHECK(fixture::pwmByPin[p] == 0);
        frame(wc, BTN_LB); CHECK(fixture::pwmByPin[MOTOR_L_DIR] == 25 && fixture::pwmByPin[MOTOR_R_DIR] == 0);   // LF/LB group only
        frame(wc); CHECK(fixture::pwmByPin[MOTOR_L_DIR] == 0);
        frame(wc, BTN_RB); frame(wc); CHECK(wc.selectedWheel() == RearLeft);
        frame(wc, BTN_LB); CHECK(fixture::pwmByPin[MOTOR_L_DIR] == 25 && fixture::pwmByPin[MOTOR_R_DIR] == 0); frame(wc);
        frame(wc, BTN_RB); frame(wc); CHECK(wc.selectedWheel() == FrontRight);
        frame(wc, BTN_LB); CHECK(fixture::pwmByPin[MOTOR_R_DIR] == 25 && fixture::pwmByPin[MOTOR_L_DIR] == 0);   // RF/RB group only
        frame(wc, BTN_LB, 1, 1, 1); CHECK(fixture::pwmByPin[MOTOR_R_DIR] == 25);                                 // sticks do nothing here
        frame(wc, BTN_LB | BTN_RB); CHECK(fixture::pwmByPin[MOTOR_R_DIR] == 0);                                  // any second button stops it
        frame(wc); frame(wc, BTN_LB); fixture::stale = true; wc.loop(); fixture::stale = false; CHECK(fixture::pwmByPin[MOTOR_R_DIR] == 0);   // link loss
        frame(wc, BTN_LB); CHECK(fixture::pwmByPin[MOTOR_R_DIR] == 0); frame(wc); frame(wc, BTN_LB); CHECK(fixture::pwmByPin[MOTOR_R_DIR] == 25);
        frame(wc, BTN_LB | BTN_RT); CHECK(fixture::pwmByPin[MOTOR_R_DIR] == 0);                                  // RT ends the check
        frame(wc); frame(wc, BTN_LB); CHECK(fixture::pwmByPin[MOTOR_R_DIR] == 0);
        wc.start(); frame(wc); frame(wc, BTN_LB); frame(wc, BTN_LB | BTN_LT); CHECK(fixture::pwmByPin[MOTOR_L_DIR] == 0 && fixture::pwmByPin[MOTOR_R_DIR] == 0);   // LT ends it
        wc.stop();
        for (int p : {MOTOR_3_DIR, MOTOR_4_DIR}) CHECK(fixture::pwmByPin[p] == 0);
        CHECK(begins == servoBegins && writes == servoWrites && fixture::pwmByPin[MOTOR_L_DIR] == 0 && fixture::pwmByPin[MOTOR_R_DIR] == 0); });
    X(9, "measured servo calibration: poses, limits, waits and responses as recorded", {
        static_assert(configuration.grip.first == 85 && configuration.grip.second == 120 && configuration.grip.low == 85 && configuration.grip.high == 120);
        static_assert(configuration.crane.first == 30 && configuration.crane.second == 120 && configuration.crane.low == 30 && configuration.crane.high == 120);
        static_assert(configuration.grip.pulse0 == 500 && configuration.grip.pulse180 == 2500 && configuration.crane.pulse0 == 500 && configuration.crane.pulse180 == 2500);
        static_assert(configuration.grip.start == 85 && configuration.crane.start == 30);                  // open and raised
        static_assert(configuration.grip.settleMs >= 2130 && configuration.crane.settleMs >= 4800);      // never shorter than the timed moves
        static_assert(configuration.craneClearanceVerified && configuration.grip.enabled && configuration.crane.enabled && !configuration.payload.enabled);
        static_assert(configuration.safety.onLinkLoss == ServoResponse::Hold && configuration.safety.onCancelWithoutStow == ServoResponse::Hold &&
                      configuration.safety.onFault == ServoResponse::Hold && configuration.safety.responsesVerified);
        static_assert(check([] { auto r = configuration; r.safety.responsesVerified = false; return r; }()) == Error::ServoResponse);
        static_assert(with([](Robot& r) { r.grip.second = r.grip.high = 150; }) == Error::Grip);       // superseded closed angle
        static_assert(with([](Robot& r) { r.grip.second = r.grip.high = 160; }) == Error::Grip);       // tight position
        static_assert(with([](Robot& r) { r.crane.second = r.crane.high = 125; }) == Error::CraneEndpoint); });   // beyond the tested 120
    X(10, "claw and crane with the recorded values: staged start, toggles, waits, holds on link loss", {
        static constexpr Robot manual = [] { auto r = configuration; r.drive.enabled = false;
            return r; }();   // the delivered servo settings, drive switched off to isolate them
        TeleOp<manual> op; const int m = fixture::begins; op.init(); op.start();
        frame(op, BTN_LB); CHECK(attached == 0);                                   // nothing powers up while a button is held
        frame(op); CHECK(attached == 2 && angle[SERVO_1] == 85 && angle[SERVO_2] == 30);   // start pose = staged pose, no other motion
        const int n = writes; elapsed(op, 5990); frame(op, BTN_LB); CHECK(writes == n);    // start-up wait is the crane's 6 s
        elapsed(op, 20); frame(op); frame(op);
        frame(op, BTN_LB); CHECK(angle[SERVO_1] == 120); elapsed(op, 2690); CHECK(op.currentPhase() == Phase::Settling);
        elapsed(op, 10); CHECK(op.currentPhase() == Phase::Idle); frame(op); frame(op);
        frame(op, BTN_RB); CHECK(angle[SERVO_2] == 120 && angle[SERVO_1] == 120); elapsed(op, 5990); CHECK(op.currentPhase() == Phase::Settling);
        elapsed(op, 10); CHECK(op.currentPhase() == Phase::Idle); frame(op); frame(op);
        frame(op, BTN_RB); CHECK(angle[SERVO_2] == 30); const int k = writes;
        fixture::stale = true; op.loop(); fixture::stale = false;                  // radio lost mid-lift
        CHECK(op.currentPhase() == Phase::LinkLost && attached == 2 && writes == k && angle[SERVO_1] == 120);
        frame(op); elapsed(op, 6000); frame(op); frame(op); CHECK(op.currentPhase() == Phase::Idle && writes == k);
        frame(op, BTN_RT); CHECK(op.currentPhase() == Phase::Inhibited && writes == k && attached == 2);   // RT: no stow route, nothing moves
        frame(op, BTN_LT); CHECK(attached == 0 && op.currentPhase() == Phase::Off);
        CHECK(fixture::begins == m); });                                             // no motor output in this configuration
    X(11, "DELIVERED IMAGE: drive capped at 50 plus LB claw and RB crane, all stops intact", {
        static_assert(configuration.drive.enabled && configuration.drive.maxPwm == 50 && configuration.drive.minPwm == 0 && logicalOutputs(configuration.drive) == 2);
        static_assert(configuration.drive.throttleSign == 1 && configuration.drive.steeringSign == -1 && configuration.drive.deadband == 0.10f);
        const int sb = begins, m = fixture::begins;
        auto L = [] { return fixture::pwmByPin[MOTOR_L_DIR]; }; auto R = [] { return fixture::pwmByPin[MOTOR_R_DIR]; };
        auto ready = [](NRLOpMode& o) { elapsed(o, 6000); frame(o); frame(o); };
        HyperdriveTeleOp op; op.init(); CHECK(attached == 0 && fixture::begins == m);   // INIT powers nothing
        op.start(); CHECK(attached == 0);
        frame(op, 0, 1, 1); CHECK(attached == 0 && fixture::begins == m);        // stick held at START: nothing arms
        frame(op, BTN_LB); CHECK(attached == 0);                                 // nor with a button held
        const int w0 = writes;
        frame(op); CHECK(attached == 2 && begins == sb + 2 && fixture::begins == m + 2);   // claw + crane, two motor outputs
        CHECK(angle[SERVO_1] == 85 && angle[SERVO_2] == 30 && writes == w0);      // start-up issues the staged poses and nothing else
        frame(op, 0, 1, 1); frame(op, BTN_LB); CHECK(L() == 0 && R() == 0 && writes == w0);   // nothing accepted during the start-up wait
        ready(op); CHECK(op.currentPhase() == Phase::Idle && writes == w0 && L() == 0 && R() == 0);
        frame(op, 0, 1, 1);  CHECK(L() == -50 && R() == 50);                     // forward
        frame(op, 0, 1, -1); CHECK(L() == 50 && R() == -50);                     // reverse
        frame(op, 0, 1, 0, 1); CHECK(L() == 50 && R() == 50);                    // right stick, as seen under the factory app
        frame(op, 0, 1, .1f, .1f); CHECK(L() == 0 && R() == 0);                  // dead zone
        for (float f : {-1.f, -.5f, .5f, 1.f}) for (float t : {-1.f, .5f, 1.f}) { frame(op, 0, 1, f, t); CHECK(L() >= -50 && L() <= 50 && R() >= -50 && R() <= 50); }
        frame(op); frame(op);
        frame(op, BTN_LB); CHECK(angle[SERVO_1] == 120 && L() == 0);             // LB closes the claw
        elapsed(op, 2700); frame(op); frame(op);
        frame(op, BTN_RB); CHECK(angle[SERVO_2] == 120 && angle[SERVO_1] == 120);   // RB lowers the crane, claw stays closed
        ready(op);
        frame(op, BTN_RB); CHECK(angle[SERVO_2] == 30); ready(op);               // RB raises it again
        frame(op, 0, 1, 1); CHECK(L() == -50 && R() == 50 && angle[SERVO_1] == 120);   // driving with a held CORE
        const int w1 = writes;
        fixture::stale = true; op.loop(); fixture::stale = false;                // radio lost while driving and holding
        CHECK(L() == 0 && R() == 0 && op.currentPhase() == Phase::LinkLost && attached == 2 && writes == w1 && angle[SERVO_1] == 120 && angle[SERVO_2] == 30);
        frame(op, 0, 1, 1); frame(op, BTN_LB); CHECK(L() == 0 && writes == w1);  // held inputs do not bring it back
        frame(op); ready(op); frame(op, 0, 1, 1); CHECK(L() == -50 && writes == w1);   // only fresh neutral does
        frame(op, BTN_RT, 1, 1); CHECK(L() == 0 && R() == 0 && op.currentPhase() == Phase::Inhibited && writes == w1 && attached == 2);   // RT: stop, hold, latch
        frame(op); frame(op, 0, 1, 1); frame(op); frame(op, BTN_LB); CHECK(L() == 0 && writes == w1);
        frame(op, BTN_LT); CHECK(attached == 0 && op.currentPhase() == Phase::Off && L() == 0 && R() == 0);   // LT: STOP
        for (int a : {angle[SERVO_1], angle[SERVO_2]}) CHECK(a >= 30 && a <= 120);
        CHECK(begins == sb + 2);                                                 // no third servo, ever
        for (int p : {MOTOR_3_DIR, MOTOR_4_DIR}) CHECK(fixture::pwmByPin[p] == 0); });
    X(4, "confirmed socket order is fixed in the configuration", {
        static_assert(configuration.drive.wheel[RearRight].socket == 1 && configuration.drive.wheel[FrontRight].socket == 2);
        static_assert(configuration.drive.wheel[FrontLeft].socket == 3 && configuration.drive.wheel[RearLeft].socket == 4);
        static_assert(configuration.grip.pin == SERVO_1 && configuration.grip.first == 85 && configuration.grip.second == 120);
        static_assert(configuration.crane.pin == SERVO_2 && configuration.crane.first == 30 && configuration.pickup.dwellMs == 500); });

    printf("PASS: HYPERDRIVE V4 host suite - %d cases (96 directive + 11 extra)\n", passed);
}
