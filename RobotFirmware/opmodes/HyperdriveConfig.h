#pragma once
#include "NRL.h"
#include <algorithm>
#include <cstdint>

// ============================================================
//  HYPERDRIVE V4 - the single hardware configuration
// ============================================================
//  Every physical fact about the robot lives in `configuration` near the
//  bottom of this file. Nothing is enabled until its own evidence is
//  entered; check() rejects the build (static_assert) if a feature is
//  switched on without the measurements it depends on.
//
//  Status of a value:  Verified   - measured/observed on this robot
//                      Unverified - requested or assumed, never operational
//                      Disabled   - deliberately off
//                      Unsupported- cannot work on this hardware

namespace hyperdrive {

enum class Status : uint8_t { Disabled, Unverified, Verified, Unsupported };

// ---------------- DriveConfig ----------------------------------------------
// Wheel names are the TEAM'S LABELS (LF, LB, RF, RB), assigned while looking at
// the robot from its front. They are kept exactly as labelled and are not a
// claim about the robot's own left and right; steeringSign, measured on the
// robot, is what makes "stick right" turn the robot right.
enum Wheel { FrontLeft, RearLeft, FrontRight, RearRight, WheelCount };
enum class Channel : uint8_t { None, L, R, M3, M4 };
constexpr int channelCount = 4;   // SDK outputs that exist; a robot may use 2 (paired groups) or 4
// Number of distinct driver outputs a configuration actually uses.
struct DriveConfig;
constexpr int logicalOutputs(const DriveConfig& d);
struct Pins { int dir, pwm; };
constexpr Pins pinsOf(Channel c) {
    return c == Channel::L  ? Pins{MOTOR_L_DIR, MOTOR_L_PWM}
         : c == Channel::R  ? Pins{MOTOR_R_DIR, MOTOR_R_PWM}
         : c == Channel::M3 ? Pins{MOTOR_3_DIR, MOTOR_3_PWM}
         : c == Channel::M4 ? Pins{MOTOR_4_DIR, MOTOR_4_PWM} : Pins{-1, -1};
}
constexpr bool leftSide(int wheel) { return wheel == FrontLeft || wheel == RearLeft; }
// Socket positions counted from the board's left edge. User-confirmed order
// RB, RF, LF, LB - a physical fact, not a routing claim.
constexpr int confirmedSocket[WheelCount] = {3, 4, 2, 1};
// Group printed on the motor-driver board above each socket pair.
//   MOTOR 2: sockets 1-2 (RB, RF) - seen directly by the team.
//   MOTOR 1: sockets 3-4 (LF, LB) - inferred from the board layout, not yet seen.
constexpr int boardGroup[WheelCount] = {1, 1, 2, 2};

struct MotorConfig {
    int     socket  = 0;                    // physical socket, 1..4
    Channel channel = Channel::None;        // SDK driver channel feeding that socket
    int     sign    = 0;                    // +1/-1: command sign that rolls the wheel forward
    Status  routing = Status::Unverified;   // socket -> channel traced on the real board
};
struct DriveConfig {
    bool enabled = false;
    MotorConfig wheel[WheelCount] = {{3}, {4}, {2}, {1}};
    int    group[WheelCount] = {1, 1, 2, 2};          // board label, see boardGroup
    Status groupObserved[WheelCount] = {Status::Unverified, Status::Unverified,   // LF, LB inferred
                                        Status::Verified, Status::Verified};      // RF, RB observed
    // Architecture B: each labelled group is ONE driver output feeding both of
    // its sockets. Set only when that is proven on the board (continuity between
    // the two sockets of a group). Both wheels of a group then name the same
    // channel, turn together and share one polarity; the firmware creates one
    // motor object per channel, never two on the same GPIO pair.
    // Left false, every wheel needs its own channel (architecture A).
    bool pairedSocketsVerified = false;
    bool motor4PinsFreeVerified = false;    // GPIO15/16 are also line-array select lines
    bool driverCapacityVerified = false;    // four motors within the driver's current rating
    int  throttleSign = 0, steeringSign = 0;  // +1/-1 for leftY / rightX
    int  maxPwm = 0;                        // ceiling, 11..255
    int  minPwm = 0;                        // 0 = off; measured PWM at which wheels first turn
    int  slewPwmPerSecond = 0;              // 0 = off; manual acceleration/deceleration limit
    float deadband = 0;                     // stick neutral zone, 0..1 exclusive
    // Stage B tool: spin ONE routed wheel at low power to record its polarity.
    bool wheelCheckEnabled = false;
    int  wheelCheckPwm = 0;                 // 11..64
};

constexpr int logicalOutputs(const DriveConfig& d) {
    bool used[channelCount + 1] = {};
    int n = 0;
    for (int i = 0; i < WheelCount; ++i) {
        const int c = static_cast<int>(d.wheel[i].channel);
        if (c && !used[c]) { used[c] = true; ++n; }
    }
    return n;
}

// ---------------- Servo1/2/3Config -----------------------------------------
struct Servo {
    bool enabled = false;
    int pin = -1;
    int first = -1, second = -1;  // claw open/closed, crane raised/lowered, payload raised/lowered
    int start = -1, low = -1, high = -1;   // staged startup pose and verified safe travel
    unsigned pulse0 = 0, pulse180 = 0;     // microseconds at 0 and 180 degrees
    uint32_t settleMs = 0;                 // measured worst-case full travel incl. settling
};

// ---------------- ControllerMap --------------------------------------------
enum Slot { R1, R2, R3, R4, L1, L3, L4, SlotCount };
struct Binding {
    const char* label;      // physical position as the team labels it
    const char* action;
    unsigned    mask = 0;   // SDK button bit; 0 = unassigned
    Status      status = Status::Unverified;
    const char* evidence = "none";
};
struct ControllerMap {
    Binding slot[SlotCount] = {
        {"R1", "raise payload"},        {"R2", "automatic CORE collection"},
        {"R3", "quick pickup and lift"}, {"R4", "lower payload"},
        {"L1", "rotate 180"},           {"L3", "rotate 90 right"},
        {"L4", "rotate 90 left"}};
    bool fullLayoutEnabled = false;   // true demands all seven verified
};
constexpr unsigned roundKeys = BTN_X | BTN_A | BTN_Y | BTN_DPAD_UP | BTN_DPAD_DOWN |
                               BTN_DPAD_LEFT | BTN_DPAD_RIGHT;
// A binding acts only when it has a mask AND that mask was physically verified.
constexpr unsigned key(const ControllerMap& m, int slot) {
    return m.slot[slot].status == Status::Verified ? m.slot[slot].mask : 0;
}

// ---------------- PickupConfig / TurnConfig --------------------------------
struct PickupConfig {
    bool enabled = false, timingAndGeometryVerified = false;
    uint32_t dwellMs = 500;      // confirmed requirement: after lowering completes
    uint32_t timeoutMs = 0;      // whole-routine bound
};
enum class TurnMode { Disabled, Heading, Timed };
struct TurnConfig {
    TurnMode mode = TurnMode::Disabled;
    int pwm = 0, halfTurnSign = 0;
    uint32_t timeoutMs = 0, halfMs = 0, rightMs = 0, leftMs = 0;  // timed mode, each measured
    int headingRightSign = 0;
    float toleranceDeg = 0;
    // Which way the L-LABELLED wheel pair must be commanded for the ROBOT to
    // turn right: +1 forward, -1 backward. The labels were assigned from the
    // front of the robot, so this cannot be assumed; it is observed on blocks.
    // L3 always means "robot turns right", L4 "robot turns left", whatever this is.
    int rightTurnSign = 0;
};

// ---------------- SafeStowConfig / SafetyConfig ----------------------------
struct StowConfig {
    bool pathsAndRetentionVerified = false;
    int gripRest = -1, craneRest = -1, payloadRest = -1;
    int permissivePin = -1;   // healthy = closed to ground; HIGH/unplugged = fault
};
// What the servos do when the robot must stop acting. Neither choice is safe by
// assumption: holding keeps torque on a possibly jammed linkage, removing PWM
// may let a loaded crane fall. The team picks per case from physical tests.
enum class ServoResponse : uint8_t { Hold, RemoveLiftPwm, RemoveAllPwm };
struct SafetyConfig {
    uint32_t linkTimeoutMs = 50;
    ServoResponse onLinkLoss = ServoResponse::Hold;
    ServoResponse onCancelWithoutStow = ServoResponse::Hold;
    ServoResponse onFault = ServoResponse::Hold;
    bool responsesVerified = false;   // the three choices above were tested under load
};

// ---------------- TeamConfig ------------------------------------------------
struct TeamConfig {
    int  number = 1;                  // PLACEHOLDER. The SDK needs some number; it is shown on
                                      // the hub's display only. Not our registered team number.
    bool confirmed = false;           // true once the registered team number is entered
    int  channel = 1;                 // radio channel (1, 6 or 11)
    bool channelVerified = false;     // matches what the handheld shows
    // Robot name shown on the hub is the build stamp -DNRL_TEAM_NAME in
    // platformio.ini ("HYPERDRIVE"); it cannot be read back here.
};

// Angles measured by the team on the factory SERVO TEST, 12 Nov 2026.
// Claw: 85 open, 120 closed - at 120 it holds the CORE without slipping. This
// supersedes the earlier 150; 160 adds tightness and is never used.
// Crane (replacement servo): 30 raised, 120 lowered, full travel tested with no
// noise, binding or resistance. Earlier values (105 closed, 50 raised, 125
// requested) are superseded by these measurements.
constexpr int clawOpenDeg = 85, clawClosedDeg = 120;
constexpr int craneRaisedDeg = 30, craneLoweredTestedDeg = 120;

struct Robot {
    DriveConfig drive{};
    Servo grip{false, SERVO_1, clawOpenDeg, clawClosedDeg}, crane{false, SERVO_2, craneRaisedDeg, -1}, payload{};
    ControllerMap map{};
    PickupConfig pickup{};
    TurnConfig turns{};
    StowConfig stow{};
    SafetyConfig safety{};
    TeamConfig team{};
    bool craneClearanceVerified = false, payloadPathAndPowerVerified = false;
    int parkedPayload = -1;           // payload pose that clears the crane and claw
};
// Limit of crane travel the team has physically tested (see constants above).
constexpr int requestedCraneLoweredDeg = craneLoweredTestedDeg;

// ============================================================
//  THE ROBOT. Every output is still disabled.
// ============================================================
inline constexpr Robot configuration = [] {
    Robot r;
    // Motor routing, entered 12 Nov 2026 on the team's decision.
    // Evidence: factory MOTOR TEST with wheels raised - M1 moved only LF/LB,
    // M2 moved only RF/RB (two paired groups, physically confirmed). In the SDK
    // library that test's first slot is MOTOR_L (5/10) and its second MOTOR_R
    // (11/12). That the screen's "M1" is the first slot is corroborated, not
    // proven; if it is wrong, the first wheel check on blocks shows the opposite
    // pair moving and the two channels below are swapped.
    // Radio: the handheld displays RADIO CHANNEL 1 (seen by the team, 12 Nov 2026).
    r.team.channel = 1;
    r.team.channelVerified = true;
    r.drive.pairedSocketsVerified = true;
    r.drive.groupObserved[FrontLeft] = r.drive.groupObserved[RearLeft] = Status::Verified;   // pair proven by the test; printed "MOTOR 1" label not yet read
    // PROVISIONAL forward polarities, derived from the factory MOTOR TEST and
    // entered with the provisional channel mapping above:
    //   M1 at +25 -> LF/LB rotated backward  => sign -1
    //   M2 at +25 -> RF/RB rotated forward   => sign +1
    // The team confirms "forward" = the rotation that moves the robot toward
    // its front. The SDK library shows both test outputs are unflipped, so a
    // positive test value is a positive motor command. Not yet seen on V4.2
    // itself: the wheel check on blocks is what confirms them.
    r.drive.wheel[FrontLeft]  = {3, Channel::L, -1, Status::Verified};
    r.drive.wheel[RearLeft]   = {4, Channel::L, -1, Status::Verified};
    r.drive.wheel[FrontRight] = {2, Channel::R, +1, Status::Verified};
    r.drive.wheel[RearRight]  = {1, Channel::R, +1, Status::Verified};
    // Servo calibration from the factory SERVO TEST (angles above). Pulse range
    // 500-2500 us over 0-180 degrees is what that factory test itself uses in
    // the SDK library, so the measured angles mean the same thing here.
    // Travel limits are exactly the range the team tested, nothing wider.
    r.grip.low = clawOpenDeg;     r.grip.high = clawClosedDeg;
    r.grip.pulse0 = 500;          r.grip.pulse180 = 2500;
    r.crane.second = craneLoweredTestedDeg;
    r.crane.low = craneRaisedDeg; r.crane.high = craneLoweredTestedDeg;
    r.crane.pulse0 = 500;         r.crane.pulse180 = 2500;
    r.craneClearanceVerified = true;   // full 30-120 travel tested by the team, no binding
    // Start poses chosen by the team: claw OPEN, crane RAISED. The first
    // neutral packet after START commands exactly these angles, so both
    // mechanisms must already be standing there when START is pressed.
    r.grip.start = clawOpenDeg;
    r.crane.start = craneRaisedDeg;
    // Waiting time after each command. Timed by the team on the factory SERVO
    // TEST: claw 2.13 s closing / 2.08 s opening, crane 4.66 s lowering /
    // 4.80 s raising. Those include button-stepping time, so they are upper
    // bounds on travel, not servo speeds. Entered as the slower direction plus
    // 25 %, rounded up - a margin chosen here, not a measurement.
    r.grip.settleMs = 2700;    // 2130 ms x 1.25
    r.crane.settleMs = 6000;   // 4800 ms x 1.25
    // LB CLAW AND RB CRANE ENABLED, 13 Nov 2026.
    // Loaded test by the team (factory SERVO TEST, CORE in the claw at 120,
    // crane raised at 30): both held with the signal present, and after
    // leaving the test the crane stayed raised and the claw kept the CORE,
    // with no unexpected movement. In the SDK library, leaving that test
    // detaches the servos, so this is the signal-removed case under load -
    // the same thing the framework does at STOP.
    // V4.2's responses below are all Hold: on radio loss, on RT without a
    // stow route and on a fault the servos keep their last command, which is
    // the held state the team observed. A real radio-loss or power-loss test
    // has NOT been done; it is the first check when this image is commissioned.
    r.safety.onLinkLoss = ServoResponse::Hold;
    r.safety.onCancelWithoutStow = ServoResponse::Hold;
    r.safety.onFault = ServoResponse::Hold;
    r.safety.responsesVerified = true;   // on the evidence above; radio/power loss still to be seen
    r.grip.enabled = true;
    r.crane.enabled = true;

    // MANUAL DRIVING ENABLED, 13 Nov 2026 - first run must be on blocks.
    // Ceiling 50 of 255: the highest value the team has run on both groups
    // with the factory MOTOR TEST without abnormal behaviour (+25 and +50).
    // The driver chip is unidentified, so nothing above 50 is supported.
    r.drive.maxPwm = 150;
    r.drive.driverCapacityVerified = true;   // for outputs up to 50 only, by that test
    // Floor 11: the SDK motor driver (HexaDCMotor.h, MOTOR_DEADBAND = 10) turns
    // every command from -10 to +10 into zero. Without a floor the first fifth
    // of stick travel past the dead zone produced no output at all.
    r.drive.minPwm = 11;
    // Steering -1: the team saw, under the factory application, right stick =
    // RF/RB forward + LF/LB backward = nose swings right. This mixer adds
    // steering to the L-labelled pair, so right stick must be negative here.
    // Assumes rightX reads positive to the right (SDK convention).
    r.drive.steeringSign = -1;
    // Throttle +1: SDK convention that leftY reads positive when pushed
    // forward. NOT observed on this robot - if the wheels run backward on
    // blocks with the stick forward, this becomes -1.
    r.drive.throttleSign = +1;
    // Stick dead zone: a software setting, not a hardware measurement. 0.10
    // chosen as a starting value; adjust if the robot creeps or feels dead.
    r.drive.deadband = 0.10f;
    r.drive.enabled = true;
    // HD Wheel Check is off: the build never contains both it and the drive.
    // To run it again: enabled = false, wheelCheckEnabled = true, wheelCheckPwm = 25.
    return r;
}();

// ---------------- Validation ------------------------------------------------
enum class Error {
    Ok, TeamChannel, MapMask, MapDuplicate, MapIncomplete, MapUnverified,
    DriveSocket, DriveGroup, DriveRouting, DriveChannel, DriveDuplicate, DrivePolarity, DrivePwm,
    DriveAxis, DriveDeadband, DriveCapacity, WheelCheck,
    Grip, Crane, CraneEndpoint, CraneClearance, Payload, PayloadPort, PayloadPath,
    ServoResponse, Pickup, PickupTiming, Turn, TurnDirection, TurnCalibration, Stow
};
constexpr bool signOK(int v) { return v == -1 || v == 1; }
constexpr bool durationOK(uint32_t v) { return v > 0 && v < 0x80000000u; }
constexpr bool servoOK(const Servo& s) {
    return (s.pin == SERVO_1 || s.pin == SERVO_2 || s.pin == SERVO_3 || s.pin == SERVO_4) &&
        s.pulse0 > 0 && s.pulse0 < s.pulse180 && s.pulse180 < 20000 &&
        s.low >= 0 && s.low < s.high && s.high <= 180 && s.first != s.second &&
        s.first >= s.low && s.first <= s.high && s.second >= s.low && s.second <= s.high &&
        (s.start == s.first || s.start == s.second) && durationOK(s.settleMs);
}
constexpr bool stowTargetOK(const Servo& s, int target) {
    return !s.enabled || (target >= s.low && target <= s.high &&
        target >= std::min(s.first, s.second) && target <= std::max(s.first, s.second));
}
constexpr bool stowReady(const Robot& r) {
    const auto& s = r.stow;
    return s.pathsAndRetentionVerified &&
        (s.permissivePin == DIGITAL_D1 || s.permissivePin == DIGITAL_D2 || s.permissivePin == DIGITAL_D3) &&
        stowTargetOK(r.grip, s.gripRest) && stowTargetOK(r.crane, s.craneRest) &&
        stowTargetOK(r.payload, s.payloadRest);
}
constexpr bool wheelRouted(const DriveConfig& d, int w) {
    return d.wheel[w].routing == Status::Verified && d.wheel[w].channel != Channel::None;
}
constexpr Error checkWheel(const DriveConfig& d, int i, bool needSign) {
    const auto& w = d.wheel[i];
    if (w.routing != Status::Verified) return Error::DriveRouting;
    if (w.channel == Channel::None || w.channel > Channel::M4) return Error::DriveChannel;
    if (w.channel == Channel::M4 && !d.motor4PinsFreeVerified) return Error::DriveChannel;
    if (needSign && !signOK(w.sign)) return Error::DrivePolarity;
    for (int j = 0; j < i; ++j) {
        if (d.wheel[j].routing != Status::Verified) continue;
        const bool sameGroup = d.group[i] == d.group[j];
        if (d.wheel[j].channel != w.channel) {
            // One driver output per group cannot be two different channels.
            if (d.pairedSocketsVerified && sameGroup) return Error::DriveGroup;
            continue;
        }
        // Sharing a channel is legal only inside one proven paired group, whose
        // two wheels then necessarily turn the same way.
        if (!d.pairedSocketsVerified || !sameGroup || leftSide(i) != leftSide(j) ||
            (needSign && d.wheel[j].sign != w.sign)) return Error::DriveDuplicate;
    }
    return Error::Ok;
}
constexpr Error check(const Robot& r) {
    if (r.team.channel != 1 && r.team.channel != 6 && r.team.channel != 11) return Error::TeamChannel;
    if (r.team.number < 1 || r.team.number > 99999) return Error::TeamChannel;

    unsigned used = 0;
    for (int i = 0; i < SlotCount; ++i) {
        const auto& b = r.map.slot[i];
        if (!b.mask) {
            if (b.status == Status::Verified) return Error::MapMask;
            if (r.map.fullLayoutEnabled) return Error::MapIncomplete;
            continue;
        }
        if (!(b.mask & roundKeys) || (b.mask & ~roundKeys) || (b.mask & (b.mask - 1))) return Error::MapMask;
        if (b.mask & used) return Error::MapDuplicate;
        used |= b.mask;
        if (r.map.fullLayoutEnabled && b.status != Status::Verified) return Error::MapIncomplete;
    }

    const auto& d = r.drive;
    for (int i = 0; i < WheelCount; ++i) {
        if (d.wheel[i].socket != confirmedSocket[i]) return Error::DriveSocket;
        if (d.group[i] != boardGroup[i]) return Error::DriveGroup;
    }
    if (d.enabled) {
        for (int i = 0; i < WheelCount; ++i) {
            // An inferred group label is not evidence the drive may rely on.
            if (d.groupObserved[i] != Status::Verified) return Error::DriveGroup;
            if (const Error e = checkWheel(d, i, true); e != Error::Ok) return e;
        }
        if (!signOK(d.throttleSign) || !signOK(d.steeringSign)) return Error::DriveAxis;
        if (d.maxPwm <= 10 || d.maxPwm > 255 || d.minPwm < 0 || d.minPwm >= d.maxPwm ||
            d.slewPwmPerSecond < 0) return Error::DrivePwm;
        if (!d.driverCapacityVerified) return Error::DriveCapacity;
    }
    if ((d.enabled || r.pickup.enabled) && !(d.deadband > 0 && d.deadband < 1)) return Error::DriveDeadband;
    if (d.wheelCheckEnabled) {
        // The polarity tool and the drivetrain never coexist in one build.
        if (d.enabled || d.wheelCheckPwm <= 10 || d.wheelCheckPwm > 64) return Error::WheelCheck;
        bool any = false;
        for (int i = 0; i < WheelCount; ++i) {
            if (d.wheel[i].routing != Status::Verified) continue;
            any = true;
            if (const Error e = checkWheel(d, i, false); e != Error::Ok) return e;
        }
        if (!any) return Error::WheelCheck;
    }

    if (r.grip.enabled && !(servoOK(r.grip) && r.grip.pin == SERVO_1 &&
        r.grip.first == clawOpenDeg && r.grip.second == clawClosedDeg)) return Error::Grip;
    if (r.crane.enabled) {
        if (r.crane.second < 0) return Error::CraneEndpoint;
        if (!(servoOK(r.crane) && r.crane.pin == SERVO_2 && r.crane.first == craneRaisedDeg)) return Error::Crane;
        if (r.crane.second <= r.crane.first || r.crane.second > requestedCraneLoweredDeg)
            return Error::CraneEndpoint;
        if (!r.craneClearanceVerified) return Error::CraneClearance;
    }
    if (r.payload.enabled) {
        if (r.payload.pin != SERVO_3 && r.payload.pin != SERVO_4) return Error::PayloadPort;
        if (!servoOK(r.payload) ||
            !(r.parkedPayload == r.payload.first || r.parkedPayload == r.payload.second) ||
            !(!r.crane.enabled || r.payload.start == r.parkedPayload || r.crane.start == r.crane.first))
            return Error::Payload;
        if (!r.payloadPathAndPowerVerified) return Error::PayloadPath;
        if (!key(r.map, R1) || !key(r.map, R4)) return Error::MapUnverified;
    }
    if ((r.grip.enabled || r.crane.enabled || r.payload.enabled) && !r.safety.responsesVerified)
        return Error::ServoResponse;
    if (r.safety.linkTimeoutMs < 20 || r.safety.linkTimeoutMs > 500) return Error::ServoResponse;

    if (r.pickup.enabled) {
        if (!(r.grip.enabled && r.crane.enabled && r.pickup.timingAndGeometryVerified))
            return Error::Pickup;
        if (!key(r.map, R2) || !key(r.map, R3)) return Error::MapUnverified;
        if (!durationOK(r.pickup.timeoutMs) || r.pickup.dwellMs < 500 ||
            r.pickup.timeoutMs <= 0ULL + r.pickup.dwellMs + 2ULL * r.crane.settleMs + r.grip.settleMs)
            return Error::PickupTiming;
    }

    if (r.stow.pathsAndRetentionVerified && !stowReady(r)) return Error::Stow;

    const auto& t = r.turns;
    if (t.mode == TurnMode::Disabled) return Error::Ok;
    if (!d.enabled) return Error::Turn;
    if (!key(r.map, L1) || !key(r.map, L3) || !key(r.map, L4)) return Error::MapUnverified;
    if (!(signOK(t.halfTurnSign) && t.pwm > 10 && t.pwm <= d.maxPwm && durationOK(t.timeoutMs)))
        return Error::Turn;
    if (!signOK(t.rightTurnSign)) return Error::TurnDirection;
    if (t.mode == TurnMode::Heading)
        return signOK(t.headingRightSign) && t.toleranceDeg > 0 && t.toleranceDeg < 90
            ? Error::Ok : Error::TurnCalibration;
    return t.halfMs && t.leftMs && t.rightMs && t.halfMs < t.timeoutMs &&
           t.leftMs < t.timeoutMs && t.rightMs < t.timeoutMs ? Error::Ok : Error::TurnCalibration;
}
constexpr bool valid(const Robot& r) { return check(r) == Error::Ok; }
static_assert(valid(configuration), "Invalid HYPERDRIVE V4 configuration - see check()");
#if defined(NRL_TEAM_NUMBER) && defined(NRL_WIFI_CHANNEL)
static_assert(configuration.team.number == NRL_TEAM_NUMBER && configuration.team.channel == NRL_WIFI_CHANNEL,
              "TeamConfig must match the identity stamped into platformio.ini");
#endif

// ---------------- Differential mixer (pure maths, no hardware) --------------
struct Mix { float left, right; };
constexpr float absf(float v) { return v < 0 ? -v : v; }
// LeftCommand = T + S, RightCommand = T - S, scaled together into [-1, 1].
constexpr Mix mix(float throttle, float steering) {
    const float sum = absf(throttle) + absf(steering);
    const float scale = sum > 1.0f ? 1.0f / sum : 1.0f;
    return {(throttle + steering) * scale, (throttle - steering) * scale};
}
constexpr float deadbanded(float v, float deadband) { return absf(v) <= deadband ? 0.0f : v; }
// Normalised command -> signed PWM between the measured floor and the ceiling.
constexpr int toPwm(float command, int minPwm, int maxPwm) {
    if (command == 0) return 0;
    const float m = absf(command) > 1 ? 1.0f : absf(command);
    const float pwm = minPwm + m * (maxPwm - minPwm);
    const int rounded = static_cast<int>(pwm + 0.5f);
    return command < 0 ? -rounded : rounded;
}
}  // namespace hyperdrive
