#pragma once
#include "NRL.h"

namespace hyperdrive {
// Signed axis selectors: magnitude 1=LX, 2=LY, 3=RX, 4=RY; minus reverses.
// Port fields take BoardPins constants; zero buttons/axis/sign mean unconfigured.
struct MotorPort {
    int dir = -1, pwm = -1;
};
struct Drive {
    bool enabled = false;
    MotorPort left{}, right{};
    int leftSign = 0, rightSign = 0, forwardAxis = 0, turnAxis = 0;
    int maxPwm = 0;
    float deadband = 0;
};
struct Pose {
    int pin = -1, pulse0 = 0, pulse180 = 0;
    int first = -1, second = -1, start = -1; // open/close or lowered/raised
};
enum class LiftType { Disabled, PositionalServo, BrushedDC };
struct Robot {
    Drive drive{};
    bool gripEnabled = false;
    Pose grip{}, liftServo{};
    unsigned open = 0, close = 0, raise = 0, lower = 0;
    LiftType lift = LiftType::Disabled;
    int stepDeg = 0;
    MotorPort liftMotor{};
    int raisePwm = 0, lowerPwm = 0, upperSwitch = -1, lowerSwitch = -1;
    unsigned jogMs = 0;
    // Positional crane only. Extension requires an independently measured safe interval.
    int lowerExtensionDeg = 0;
    int gripSafeMinDeg = -1, gripSafeMaxDeg = -1;
    int liftSafeMinDeg = -1, liftSafeMaxDeg = -1;
    // Set only after the observed resistance at 100° has been resolved and
    // the supported linkage has been verified through its entire intended path.
    bool craneClearanceVerified = false;
    bool controllerLayoutConfirmed = false;
};
inline constexpr int requestedServo1ClosedDeg = 105;
inline constexpr int requestedServo2LoweredDeg = 125;
inline constexpr Robot configuration = [] {
    Robot r;
    // Proposed axes follow the SDK example. Verify signs on the handheld.
    r.drive.forwardAxis = 2; // LY
    r.drive.turnAxis = 3;    // RX
    r.open = BTN_A;
    r.close = BTN_RB;
    r.raise = BTN_RT;        // maintained toggle; one step, then return to neutral
    r.lower = BTN_LB;       // LT remains reserved for exit/STOP
    r.grip.pin = SERVO_1;
    r.grip.first = 85;
    r.grip.second = requestedServo1ClosedDeg;
    r.liftServo.pin = SERVO_2;
    r.liftServo.second = 50; // raised; 90 degrees is a middle reference, not a startup pose
    r.liftServo.first = requestedServo2LoweredDeg;
    r.lowerExtensionDeg = 0;
    return r;
}();

constexpr bool withinMeasuredTravel(Pose p, int low, int high) {
    return low >= 0 && low < high && high <= 180 &&
           p.first >= low && p.first <= high && p.second >= low && p.second <= high &&
           p.start >= low && p.start <= high;
}
// Applies when activating either confirmed mechanism in the team configuration.
constexpr bool teamServoCalibrationOK(const Robot& r) {
    if (r.gripEnabled && !(r.grip.pin == SERVO_1 && r.grip.second == requestedServo1ClosedDeg &&
        withinMeasuredTravel(r.grip, r.gripSafeMinDeg, r.gripSafeMaxDeg))) return false;
    if (r.lift == LiftType::Disabled) return true;
    return r.craneClearanceVerified && r.lift == LiftType::PositionalServo && r.liftServo.pin == SERVO_2 &&
           r.liftServo.first == requestedServo2LoweredDeg && r.lowerExtensionDeg == 0 &&
           withinMeasuredTravel(r.liftServo, r.liftSafeMinDeg, r.liftSafeMaxDeg);
}
static_assert(teamServoCalibrationOK(configuration), "Verify servo targets, measured travel and crane clearance before enabling");

constexpr bool onboard(MotorPort m) {
    return (m.dir == MOTOR_L_DIR && m.pwm == MOTOR_L_PWM) ||
           (m.dir == MOTOR_R_DIR && m.pwm == MOTOR_R_PWM);
}
constexpr int magnitude(int x) { return x < 0 ? -x : x; }
constexpr bool signedRange(int x, int min, int max) {
    return (x >= min && x <= max) || (x >= -max && x <= -min);
}
constexpr bool poseOK(Pose p) {
    return (p.pin == SERVO_1 || p.pin == SERVO_2 || p.pin == SERVO_3 || p.pin == SERVO_4)
        && p.pulse0 > 0 && p.pulse0 < p.pulse180 && p.pulse180 < 20000
        && p.first >= 0 && p.first <= 180 && p.second >= 0 && p.second <= 180
        && p.first != p.second && p.start >= (p.first < p.second ? p.first : p.second)
        && p.start <= (p.first > p.second ? p.first : p.second);
}
// Call after validating the baseline pose and the 0..5 degree extension.
constexpr int loweredTarget(const Robot& r) {
    const int direction = r.liftServo.first > r.liftServo.second ? 1 : -1;
    return r.liftServo.first + direction * r.lowerExtensionDeg;
}
constexpr bool craneTravelOK(const Robot& r) {
    if (r.lowerExtensionDeg < 0 || r.lowerExtensionDeg > 5) return false;
    if (r.liftSafeMinDeg == -1 && r.liftSafeMaxDeg == -1)
        return r.lowerExtensionDeg == 0; // original endpoints bound legacy configurations
    const int low = r.liftSafeMinDeg, high = r.liftSafeMaxDeg;
    if (low < 0 || high > 180 || low >= high) return false;
    const int lowered = loweredTarget(r);
    // The baseline and start lie between the raised and extended endpoints.
    return low <= lowered && lowered <= high &&
           low <= r.liftServo.second && r.liftServo.second <= high;
}
constexpr unsigned keys = BTN_X | BTN_A | BTN_Y | BTN_LB | BTN_RB | BTN_DPAD_UP |
                          BTN_DPAD_DOWN | BTN_DPAD_LEFT | BTN_DPAD_RIGHT;
constexpr bool switchPin(int p) { return p == DIGITAL_D1 || p == DIGITAL_D2 || p == DIGITAL_D3; }
constexpr bool valid(const Robot& r) {
    if (r.lift != LiftType::Disabled && r.lift != LiftType::PositionalServo &&
        r.lift != LiftType::BrushedDC) return false;
    if ((r.drive.enabled || r.gripEnabled || r.lift != LiftType::Disabled) &&
        !r.controllerLayoutConfirmed) return false;
    const Drive& d = r.drive;
    if (d.enabled && !(onboard(d.left) && onboard(d.right) && d.left.dir != d.right.dir
        && signedRange(d.leftSign, 1, 1) && signedRange(d.rightSign, 1, 1)
        && signedRange(d.forwardAxis, 1, 4)
        && signedRange(d.turnAxis, 1, 4)
        && magnitude(d.forwardAxis) != magnitude(d.turnAxis)
        && d.maxPwm > 10 && d.maxPwm <= 255 && d.deadband > 0 && d.deadband < 1)) return false;
    if (r.gripEnabled && !poseOK(r.grip)) return false;
    if (r.lift == LiftType::PositionalServo && !(poseOK(r.liftServo) && craneTravelOK(r) && r.stepDeg > 0 &&
        r.stepDeg <= 5 && (!r.gripEnabled || r.grip.pin != r.liftServo.pin))) return false;
    if (r.lift == LiftType::BrushedDC && !(onboard(r.liftMotor) &&
        (!d.enabled || (r.liftMotor.dir != d.left.dir && r.liftMotor.dir != d.right.dir)) &&
        switchPin(r.upperSwitch) && switchPin(r.lowerSwitch) && r.upperSwitch != r.lowerSwitch &&
        signedRange(r.raisePwm, 11, 255) &&
        signedRange(r.lowerPwm, 11, 255) &&
        (r.raisePwm > 0) != (r.lowerPwm > 0) && r.jogMs > 0 && r.jogMs <= 100)) return false;
    unsigned used = 0;
    const unsigned buttons[] = {r.open, r.close, r.raise, r.lower};
    for (int i = 0; i < 4; ++i) {
        if (i < 2 ? !r.gripEnabled : r.lift == LiftType::Disabled) continue;
        const unsigned b = buttons[i];
        const unsigned allowed = i == 2 && r.lift == LiftType::PositionalServo ? keys | BTN_RT : keys;
        if (!(b & allowed) || (b & (b - 1)) || (used & b)) return false;
        used |= b;
    }
    return true;
}
}
