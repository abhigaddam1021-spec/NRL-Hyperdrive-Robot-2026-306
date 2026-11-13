#include "HyperdriveConfig.h"
#include <algorithm>
#include <cmath>

// ============================================================
//  HYPERDRIVE V4 TeleOp - one coordinator owns every actuator
// ============================================================
//  Priority inside each 100 Hz tick, highest first. A higher rule returns
//  before a lower one can write an output:
//    1. LT / SDK STOP
//    2. controller link loss, invalid input, safety input, action deadline
//    3. RT cancel (safe stow when calibrated, otherwise inhibit)
//    4. interlocks (neutral re-arm, conflicting buttons)
//    5. the running automation (pickup or turn)
//    6. manual driving and single mechanism commands
//
//  Servo positions are never measured. "settled" below means only that the
//  calibrated worst-case travel time has passed since a command.

namespace hyperdrive {
static HexaServoConfig sdkServo(const Servo& s) {
    return {static_cast<uint8_t>(s.pin), static_cast<uint16_t>(s.pulse0),
        static_cast<uint16_t>(s.pulse180), 0, 180, 0, static_cast<float>(s.start), 0};
}
static HexaDCMotorConfig sdkMotor(Channel c) {
    return {static_cast<uint8_t>(pinsOf(c).dir), static_cast<uint8_t>(pinsOf(c).pwm)};
}

enum class Phase { Off, Arming, Settling, Idle, Lowering, Dwell, Gripping, Raising,
                   Turning, Stow, AwaitNeutral, Inhibited, LinkLost, Faulted };
enum class Owner { None, Safety, Manual, Pickup, Turn, Stow };

// What is known about one servo. There is no position feedback on this robot,
// so `verified` can never become true here.
struct ServoState {
    int  requested = -1;    // where the coordinator wants it
    int  commanded = -1;    // last angle actually written to the driver
    bool settled = false;   // travel time has elapsed since that write (estimate)
    static constexpr bool verified = false;
};

template<const Robot& R>
class TeleOp : public NRLOpMode {
    static_assert(valid(R), "Invalid HYPERDRIVE V4 configuration");

    // --- hardware-output layer ------------------------------------------------
    HexaDCMotor motors[channelCount] = {
        HexaDCMotor{sdkMotor(Channel::L)}, HexaDCMotor{sdkMotor(Channel::R)},
        HexaDCMotor{sdkMotor(Channel::M3)}, HexaDCMotor{sdkMotor(Channel::M4)}};
    bool channelReady[channelCount] = {};
    bool driveReady = false;
    float sideNow[2] = {0, 0};   // PWM currently applied to left / right
    HexaServo grip{sdkServo(R.grip)}, crane{sdkServo(R.crane)}, payload{sdkServo(R.payload)};
    bool gReady = false, cReady = false, pReady = false;
    ServoState gripS, craneS, payloadS;

    // --- coordinator state ----------------------------------------------------
    Phase phase = Phase::Off;
    Owner owner = Owner::None;
    const char* lastFault = "none";
    uint32_t sequence = 0, lastFrame = 0, since = 0, actionStart = 0, waitMs = 0, lastTick = 0;
    unsigned previous = 0, initiatingKey = 0;
    bool neutralSeen = false, payloadFirst = false;
    unsigned stowStage = 0;
    int turnSign = 0, turnDegrees = 0;
    float lastHeading = 0, progress = 0;

    static int index(Channel c) { return static_cast<int>(c) - 1; }

    void writeSides(float left, float right) {
        sideNow[0] = left; sideNow[1] = right;
        if (!driveReady) return;
        bool written[channelCount] = {};
        for (int w = 0; w < WheelCount; ++w) {
            const int c = index(R.drive.wheel[w].channel);
            if (written[c]) continue;   // paralleled sockets: one output, written once
            written[c] = true;
            motors[c].setSpeed(static_cast<int>(std::lround(sideNow[leftSide(w) ? 0 : 1])) *
                               R.drive.wheel[w].sign);
        }
    }
    // Safety stops are never rate limited.
    void stopDrive() { writeSides(0, 0); }
    // Manual driving only: move toward the target no faster than the slew limit.
    void driveToward(int left, int right, uint32_t dtMs) {
        const float target[2] = {static_cast<float>(left), static_cast<float>(right)};
        float next[2];
        const float step = R.drive.slewPwmPerSecond * (dtMs * 0.001f);
        for (int i = 0; i < 2; ++i) {
            const float delta = target[i] - sideNow[i];
            next[i] = R.drive.slewPwmPerSecond == 0 || std::fabs(delta) <= step
                ? target[i] : sideNow[i] + (delta > 0 ? step : -step);
        }
        writeSides(next[0], next[1]);
    }

    void move(HexaServo& servo, ServoState& state, int to) {
        state.requested = to;
        servo.setPosition(to);
        state.commanded = to;
        state.settled = false;
    }
    void markSettled() { gripS.settled = gReady; craneS.settled = cReady; payloadS.settled = pReady; }
    uint32_t longestSettle() const {
        return std::max({gReady ? R.grip.settleMs : 0u, cReady ? R.crane.settleMs : 0u,
                         pReady ? R.payload.settleMs : 0u});
    }
    void removePwm(bool includeClaw) {
        if (cReady) { crane.detach(); cReady = false; }
        if (pReady) { payload.detach(); pReady = false; }
        if (includeClaw && gReady) { grip.detach(); gReady = false; }
    }

    void enter(Phase next, Owner who, uint32_t now, uint32_t duration, const char* text) {
        phase = next; owner = who; since = now; waitMs = duration;
        telemetry.addData("action", text);
    }
    // Ends all motion for this run. Only a new INIT/START leaves this state.
    void fault(const char* reason, uint32_t now) {
        stopDrive();
        initiatingKey = 0; neutralSeen = false;
        if (R.safety.onFault != ServoResponse::Hold)
            removePwm(R.safety.onFault == ServoResponse::RemoveAllPwm);
        lastFault = reason;
        telemetry.addData("fault", reason);
        enter(Phase::Faulted, Owner::Safety, now, 0, "FAULT - STOP");
    }
    // Radio loss is recoverable when the servos kept their commands.
    void linkLost(uint32_t now) {
        stopDrive();
        initiatingKey = 0; neutralSeen = false;
        lastFault = "link lost";
        telemetry.addData("fault", lastFault);
        if (R.safety.onLinkLoss != ServoResponse::Hold) {
            removePwm(R.safety.onLinkLoss == ServoResponse::RemoveAllPwm);
            enter(Phase::Faulted, Owner::Safety, now, 0, "FAULT - STOP");   // poses now unknown
            return;
        }
        enter(Phase::LinkLost, Owner::Safety, now, 0, "link lost");
    }
    // Operator took over mid-automation: stop, keep the servo commands already
    // issued, wait out their travel, then demand a fresh neutral.
    void cancel(uint32_t now) {
        stopDrive();
        initiatingKey = 0; neutralSeen = false;
        enter(Phase::Settling, Owner::Safety, now, longestSettle(), "cancelled");
    }

    bool initialize() {
        // Servos first. begin() immediately commands the staged startup pose.
        if (R.grip.enabled && !(gReady = grip.begin())) return false;
        if (R.crane.enabled && !(cReady = crane.begin())) return false;
        if (R.payload.enabled && !(pReady = payload.begin())) return false;
        if (R.drive.enabled) {
            for (int w = 0; w < WheelCount; ++w) {
                const int c = index(R.drive.wheel[w].channel);
                if (channelReady[c]) continue;
                if (!motors[c].begin()) return false;
                channelReady[c] = true;
                motors[c].stop();
            }
            driveReady = true;
        }
        gripS.requested = gripS.commanded = R.grip.start;
        craneS.requested = craneS.commanded = R.crane.start;
        payloadS.requested = payloadS.commanded = R.payload.start;
        return true;
    }
    void shutdownOutputs() {
        for (int c = 0; c < channelCount; ++c)
            if (channelReady[c]) { motors[c].stop(); channelReady[c] = false; }
        driveReady = false; sideNow[0] = sideNow[1] = 0;
        removePwm(true);
    }

    bool parked() const { return !pReady || (payloadS.commanded == R.parkedPayload && payloadS.settled); }
    bool collecting() const {
        return phase == Phase::Lowering || phase == Phase::Dwell ||
               phase == Phase::Gripping || phase == Phase::Raising;
    }
    void closeForPickup(uint32_t now) {
        move(grip, gripS, R.grip.second);
        enter(Phase::Gripping, Owner::Pickup, now, R.grip.settleMs, "grip wait");
    }
    void complete(uint32_t now, const char* text = "complete") {
        stopDrive();
        markSettled();
        neutralSeen = false; initiatingKey = 0;
        enter(Phase::Idle, Owner::None, now, 0, text);
    }

    // --- RT: cancel, then stow only along calibrated, verified routes ---------
    void nextStowStage(uint32_t now) {
        while (stowStage < 3) {
            const unsigned stage = stowStage++;
            if (stage == 2) {
                // A closed claw may be holding a CORE: its command is left alone.
                if (!gReady || gripS.commanded == R.grip.second) continue;
                move(grip, gripS, R.stow.gripRest);
                enter(Phase::Stow, Owner::Stow, now, R.grip.settleMs, "stow claw");
            } else if ((stage == 0) == payloadFirst) {
                if (!pReady) continue;
                move(payload, payloadS, R.stow.payloadRest);
                enter(Phase::Stow, Owner::Stow, now, R.payload.settleMs, "stow payload");
            } else {
                if (!cReady) continue;
                move(crane, craneS, R.stow.craneRest);
                enter(Phase::Stow, Owner::Stow, now, R.crane.settleMs, "stow crane");
            }
            return;   // one mechanism at a time
        }
        markSettled();
        enter(Phase::AwaitNeutral, Owner::Safety, now, 0, "release/neutral");
    }
    void cancelForStow(uint32_t now) {
        stopDrive();
        payloadFirst = phase == Phase::Settling && initiatingKey &&
            (initiatingKey == key(R.map, R1) || initiatingKey == key(R.map, R4));
        initiatingKey = 0; neutralSeen = false; stowStage = 0;
        if constexpr (stowReady(R)) {
            nextStowStage(now);
        } else {
            // No verified stow route: issue no new servo travel and latch.
            if (R.safety.onCancelWithoutStow != ServoResponse::Hold)
                removePwm(R.safety.onCancelWithoutStow == ServoResponse::RemoveAllPwm);
            enter(Phase::Inhibited, Owner::Safety, now, 0, "stow unavail");
        }
    }

    // --- automation -----------------------------------------------------------
    void advance(uint32_t now) {
        if (phase == Phase::Turning) {
            if (R.turns.mode == TurnMode::Heading) {
                const float h = NRLComms::getHeadingFree();
                progress += std::remainder(h - lastHeading, 360.f) * R.turns.headingRightSign * turnSign;
                lastHeading = h;
                if (progress >= turnDegrees - R.turns.toleranceDeg) complete(now, "turn done");
            } else if (now - since >= waitMs) complete(now, "turn done ~");   // timed = approximate
            return;
        }
        if (now - since < waitMs) return;
        switch (phase) {
            case Phase::Settling: complete(now, owner == Owner::Safety ? "ready" : "complete"); break;
            case Phase::Lowering:   // lowering interval over: only now does the dwell start
                craneS.settled = true;
                enter(Phase::Dwell, Owner::Pickup, now, R.pickup.dwellMs, "dwell");
                break;
            case Phase::Dwell: closeForPickup(now); break;
            case Phase::Gripping:
                gripS.settled = true;
                move(crane, craneS, R.crane.first);   // claw keeps its closed command
                enter(Phase::Raising, Owner::Pickup, now, R.crane.settleMs, "raise wait");
                break;
            case Phase::Raising: complete(now, "pickup done"); break;
            default: break;
        }
    }
    void beginTurn(unsigned k, uint32_t now) {
        const auto& t = R.turns;
        turnSign = k == key(R.map, L1) ? t.halfTurnSign : (k == key(R.map, L3) ? 1 : -1);
        turnDegrees = k == key(R.map, L1) ? 180 : 90;
        if (t.mode == TurnMode::Heading) {
            lastHeading = NRLComms::getHeadingFree();
            if (!NRLComms::isHeadingReady() || !std::isfinite(lastHeading)) { fault("heading fault", now); return; }
            progress = 0;
        }
        const auto duration = k == key(R.map, L1) ? t.halfMs : (k == key(R.map, L3) ? t.rightMs : t.leftMs);
        enter(Phase::Turning, Owner::Turn, now, duration, "turn");
        // turnSign is the robot's direction (+1 right); rightTurnSign converts it
        // to commands for the L-labelled and R-labelled wheel pairs.
        const int labelled = turnSign * t.rightTurnSign * t.pwm;
        writeSides(labelled, -labelled);
    }
    // One fresh, exclusive button press while idle, stationary and settled.
    void dispatch(unsigned k, uint32_t now) {
        initiatingKey = k; actionStart = now;
        if (k == BTN_LB) {
            if (gReady && parked()) {
                move(grip, gripS, gripS.commanded == R.grip.first ? R.grip.second : R.grip.first);
                enter(Phase::Settling, Owner::Manual, now, R.grip.settleMs, "claw wait");
            }
        } else if (k == BTN_RB) {
            if (cReady && parked()) {
                move(crane, craneS, craneS.commanded == R.crane.first ? R.crane.second : R.crane.first);
                enter(Phase::Settling, Owner::Manual, now, R.crane.settleMs, "crane wait");
            }
        } else if (k == key(R.map, R1) || k == key(R.map, R4)) {
            // Payload moves only with the crane raised and at rest.
            if (pReady && (!cReady || (craneS.commanded == R.crane.first && craneS.settled))) {
                move(payload, payloadS, k == key(R.map, R1) ? R.payload.first : R.payload.second);
                enter(Phase::Settling, Owner::Manual, now, R.payload.settleMs, "payload wait");
            }
        } else if (k == key(R.map, R2) || k == key(R.map, R3)) {
            if (!R.pickup.enabled || !parked() ||
                gripS.commanded != R.grip.first || !gripS.settled || !craneS.settled) return;
            if (k == key(R.map, R2) && craneS.commanded == R.crane.first) {
                move(crane, craneS, R.crane.second);
                enter(Phase::Lowering, Owner::Pickup, now, R.crane.settleMs, "lower wait");
            } else if (k == key(R.map, R3) && craneS.commanded == R.crane.second) {
                closeForPickup(now);   // no lowering, no dwell
            }
        } else if (R.turns.mode != TurnMode::Disabled && driveReady) {
            beginTurn(k, now);
        }
    }
    bool actionKey(unsigned k) const {
        if (k == BTN_LB || k == BTN_RB) return true;
        for (int s = 0; s < SlotCount; ++s) if (k == key(R.map, s)) return true;
        return false;
    }

public:
    Phase currentPhase() const { return phase; }
    Owner currentOwner() const { return owner; }
    const char* fault() const { return lastFault; }
    const ServoState& clawState() const { return gripS; }
    const ServoState& craneState() const { return craneS; }
    const ServoState& payloadState() const { return payloadS; }

    void init() override {
        if constexpr (stowReady(R)) pinMode(R.stow.permissivePin, INPUT_PULLUP);
        telemetry.addData("action", "await START");
        telemetry.addData("team", R.team.confirmed ? "confirmed" : "PLACEHOLDER");
    }
    void start() override {
        float a, b, c, d; uint16_t keys;
        NRLGamepad::read(a, b, c, d, keys, sequence);
        lastFrame = lastTick = millis(); previous = 0; neutralSeen = false; initiatingKey = 0;
        lastFault = "none";
        phase = Phase::Arming; owner = Owner::Safety;
    }
    void loop() override {
        if (phase == Phase::Off) return;
        float lx, ly, rx, ry; uint16_t buttons; uint32_t packet;
        NRLGamepad::read(lx, ly, rx, ry, buttons, packet);
        const uint32_t now = millis();
        const uint32_t dt = now - lastTick;   // unsigned: correct across timer rollover
        lastTick = now;

        // 1. STOP
        if (buttons & BTN_LT) { stop(); telemetry.addData("action", "STOP"); return; }
        if (phase == Phase::Faulted) return;

        // 2. link, input validity, safety input, deadlines
        const bool linkBad = NRLComms::isInputStale() || now - lastFrame >= R.safety.linkTimeoutMs;
        const bool fresh = packet != sequence;
        if (fresh) { sequence = packet; lastFrame = now; }
        if (linkBad) {
            previous = 0;
            if (phase != Phase::LinkLost && phase != Phase::Arming) linkLost(now);
            return;
        }
        if (buttons & ~(roundKeys | BTN_LB | BTN_RB | BTN_RT | BTN_LT)) { fault("bad buttons", now); return; }
        for (float axis : {lx, ly, rx, ry})
            if (!(axis >= -1 && axis <= 1)) { fault("bad axis", now); return; }
        if constexpr (!R.drive.enabled && !R.grip.enabled && !R.crane.enabled && !R.payload.enabled) {
            telemetry.addData("mask", static_cast<float>(buttons));
            telemetry.addData("LY", ly); telemetry.addData("RX", rx);
        }
        const float f = deadbanded(ly, R.drive.deadband), t = deadbanded(rx, R.drive.deadband);
        const bool driving = f != 0 || t != 0;
        const bool centred = !driving && std::fabs(lx) <= R.drive.deadband && std::fabs(ry) <= R.drive.deadband;
        const unsigned keys = buttons & (roundKeys | BTN_LB | BTN_RB);
        const unsigned rising = fresh ? keys & ~previous : 0;
        if (fresh) previous = keys;

        if constexpr (stowReady(R)) {
            if (phase != Phase::Arming &&
                (digitalRead(R.stow.permissivePin) == HIGH || power.isBatteryLow())) {
                fault("safety input", now); return;
            }
        }
        if (collecting() && now - actionStart >= R.pickup.timeoutMs) { fault("pickup timeout", now); return; }
        if (phase == Phase::Turning) {
            if (now - actionStart >= R.turns.timeoutMs) { fault("turn timeout", now); return; }
            if (R.turns.mode == TurnMode::Heading &&
                (!NRLComms::isHeadingReady() || !std::isfinite(NRLComms::getHeadingFree()))) {
                fault("heading fault", now); return;
            }
        }

        // Reconnected: nothing resumes. Fresh neutral first, then wait out any travel.
        if (phase == Phase::LinkLost) {
            if (fresh && !buttons && centred)
                enter(Phase::Settling, Owner::Safety, now, longestSettle(), "relinked");
            return;
        }
        if (phase == Phase::Inhibited) return;

        // 3. RT
        if (phase == Phase::Stow) {
            if (now - since >= waitMs) nextStowStage(now);
            return;
        }
        if (phase == Phase::AwaitNeutral) {
            if (fresh && !buttons && centred) { neutralSeen = true; enter(Phase::Idle, Owner::None, now, 0, "ready"); }
            return;
        }
        if (phase != Phase::Arming && (buttons & BTN_RT)) { cancelForStow(now); return; }

        // 4. arming and interlocks
        if (phase == Phase::Arming) {
            if (!fresh || buttons || !centred) return;
            if (!initialize()) { fault("init failed", now); shutdownOutputs(); return; }
            enter(Phase::Settling, Owner::Safety, now, longestSettle(), "startup wait");
            return;
        }

        // 5. running action
        if (phase != Phase::Idle) {
            const bool takeover = driving || (keys & ~initiatingKey) || rising;
            if (takeover && owner != Owner::Safety) { cancel(now); return; }
            advance(now);
            return;
        }

        // 6. manual
        if (fresh && !keys && !driving) neutralSeen = true;
        if ((keys & (keys - 1)) || (keys && driving)) {   // contradictory input
            neutralSeen = false; stopDrive(); return;
        }
        if (keys) {
            stopDrive();
            if (neutralSeen && rising && actionKey(keys)) dispatch(keys, now);
            neutralSeen = false;
            return;
        }
        if (driveReady && neutralSeen) {
            owner = driving ? Owner::Manual : Owner::None;
            const Mix m = mix(f * R.drive.throttleSign, t * R.drive.steeringSign);
            driveToward(toPwm(m.left, R.drive.minPwm, R.drive.maxPwm),
                        toPwm(m.right, R.drive.minPwm, R.drive.maxPwm), dt);
        }
    }
    // SDK STOP. The framework also detaches every servo after this returns, so
    // a raised load must be mechanically supported through STOP.
    void stop() override {
        shutdownOutputs();
        phase = Phase::Off; owner = Owner::None;
    }
};
using HyperdriveTeleOp = TeleOp<configuration>;
REGISTER_OPMODE(HyperdriveTeleOp, "Hyperdrive TeleOp", TELEOP);
}  // namespace hyperdrive
