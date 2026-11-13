#include "NRL.h"

// ============================================================
//  HYPERDRIVE TeleOp
// ============================================================
//  Left stick up/down   = drive forward / backward
//  Right stick left/right = turn
//  LB = open / close the claw      RB = raise / lower the crane
//  LT toggle = STOP (handled by the NRL controller and framework, not here)
//
//  Every number the team measured is in the block below. If something moves
//  the wrong way on the first test (wheels off the ground!), change it HERE.

// ---------------- settings ---------------------------------------------------
// Motor test, 12 Nov: a positive command turned the M1 wheels (labelled LF/LB)
// BACKWARD and the M2 wheels (labelled RF/RB) FORWARD. So M1 is flipped.
// If one side runs backward with the stick pushed forward, change that side.
static constexpr bool  M1_FLIPPED = true;
static constexpr bool  M2_FLIPPED = false;
// Top speed, 0.0 to 1.0. The old code capped this at 0.2 (PWM 50 of 255),
// which is too weak to move the robot on the floor. Raise or lower to taste.
static constexpr float DRIVE_POWER = 0.5f;
// Swap these two signs if forward/backward or left/right come out reversed.
static constexpr float FORWARD_SIGN = 1.0f;
static constexpr float TURN_SIGN    = 1.0f;
// Sticks never rest at exactly zero. Anything smaller than this counts as zero.
static constexpr float STICK_DEADBAND = 0.10f;

// Servo test, 12 Nov: claw 85 open / 120 closed, crane 30 raised / 120 lowered.
static constexpr float CLAW_OPEN = 85.0f,    CLAW_CLOSED   = 120.0f;
static constexpr float CRANE_RAISED = 30.0f, CRANE_LOWERED = 120.0f;

// ---------------- hardware ---------------------------------------------------
static HexaDCMotor motorM1{{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM, .flipped = M1_FLIPPED }};
static HexaDCMotor motorM2{{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM, .flipped = M2_FLIPPED }};
static HexaServo   claw   {{ .signalPin = SERVO_1, .startAngle = CLAW_OPEN }};
static HexaServo   crane  {{ .signalPin = SERVO_2, .startAngle = CRANE_RAISED }};

// TankDrive(left, right) adds the turn to its first motor. The team saw that
// turning right needs the M2 wheels forward and the M1 wheels backward (the
// wheel labels were written looking at the robot from the front, so "R" wheels
// are on the robot's own left). That makes M2 the first motor here.
static TankDrive drive(motorM2, motorM1);

class HyperdriveTeleOp : public NRLOpMode {
    bool clawClosed = false;
    bool craneLowered = false;
    bool servosOK = false;
    bool motorsOK = false;

    static float deadband(float v) {
        return (v > -STICK_DEADBAND && v < STICK_DEADBAND) ? 0.0f : v;
    }

public:
    // INIT: nothing may move (rule R8.11), so no hardware is started here.
    void init() override {
        telemetry.addData("status", "ready - press START");
    }

    // START: begin() sends each servo to its start angle (claw open, crane
    // raised), which is why it lives here and not in init().
    void start() override {
        const bool clawOK  = claw.begin();
        const bool craneOK = crane.begin();
        servosOK = clawOK && craneOK;
        const bool m1OK = motorM1.begin();
        const bool m2OK = motorM2.begin();
        motorsOK = m1OK && m2OK;
        drive.setScale(DRIVE_POWER);
        drive.stop();
        clawClosed = false;
        craneLowered = false;
    }

    // 100 Hz. Never waits, never blocks.
    void loop() override {
        // Sticks always drive. If the radio drops, the framework zeroes the
        // sticks and stops the drive by itself; it comes back when the link does.
        drive.drive(deadband(gamepad1.leftY())  * FORWARD_SIGN,
                    deadband(gamepad1.rightX()) * TURN_SIGN);

        // justPressed() is true for one loop per press, so holding a bumper
        // does not flip the servo back and forth.
        if (gamepad1.justPressed(BTN_LB)) {
            clawClosed = !clawClosed;
            claw.setPosition(clawClosed ? CLAW_CLOSED : CLAW_OPEN);
        }
        if (gamepad1.justPressed(BTN_RB)) {
            craneLowered = !craneLowered;
            crane.setPosition(craneLowered ? CRANE_LOWERED : CRANE_RAISED);
        }

        telemetry.addData("claw",  clawClosed   ? "closed"  : "open");
        telemetry.addData("crane", craneLowered ? "lowered" : "raised");
        if (!servosOK) telemetry.addData("servos", "begin FAILED");
        if (!motorsOK) telemetry.addData("motors", "begin FAILED");
    }

    // STOP: motors do not stop on their own.
    void stop() override {
        drive.stop();
        motorM1.stop();
        motorM2.stop();
        claw.detach();
        crane.detach();
    }
};

REGISTER_OPMODE(HyperdriveTeleOp, "Hyperdrive TeleOp", TELEOP);
