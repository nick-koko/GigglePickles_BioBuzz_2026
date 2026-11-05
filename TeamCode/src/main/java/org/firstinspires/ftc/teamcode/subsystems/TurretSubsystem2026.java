package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GlobalRobotData;
import org.firstinspires.ftc.teamcode.util.TurretController2026;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/** 2026 turret integration. CHASSIS_ONLY never maps or commands turret hardware. */
@Configurable
public final class TurretSubsystem2026 implements Subsystem {
    public static final TurretSubsystem2026 INSTANCE = new TurretSubsystem2026();

    // Select before Init. FIXED uses only the two servos, so it also works without encoders.
    public static TurretController2026.Mode MODE = TurretController2026.Mode.CHASSIS_ONLY;
    public static String LEFT_SERVO_NAME = "Lurret";
    public static String RIGHT_SERVO_NAME = "Rurret";
    // Read only: this is also a drive motor. Never reset its count or change its mode/direction here.
    public static String ENCODER_NAME = "back_left";
    public static String ABSOLUTE_ENCODER_NAME = "analog_turret_encoder";
    public static double ENCODER_CPR = 4096.0;
    public static double ENCODER_TO_TURRET_RATIO = 81.0 / 45.0;
    public static double QUAD_DIRECTION_SIGN = -1.0;
    public static double ABSOLUTE_DIRECTION_SIGN = 1.0;
    public static double ABSOLUTE_MAX_VOLTAGE = 3.255;
    public static double ABSOLUTE_TURRET_OFFSET_DEGREES = 0.0;
    public static double SERVO_CENTER_POSITION = 0.5;
    public static double SERVO_TRAVEL_DEGREES = 355.0;
    public static double MIN_ANGLE_DEGREES = -130.0;
    public static double MAX_ANGLE_DEGREES = 130.0;
    public static double MIN_SERVO_POSITION = 0.0;
    public static double MAX_SERVO_POSITION = 1.0;
    // Initial mechanical calibration, also used in FIXED mode. Tracking learns a correction at center.
    public static double SERVO_ZERO_OFFSET_DEGREES = 0.0;
    // TOTAL separation: 2 degrees means left +1 degree and right -1 degree.
    public static double SERVO_DIFFERENTIAL_DEGREES = 2.0;
    // Front-facing zero: 0. Rear-facing zero: 180, matching last year's convention.
    public static double ROBOT_FRONT_TO_TURRET_ZERO_OFFSET_DEGREES = 0.0;
    public static double ROBOT_FRONT_RELATIVE_SIGN = 1.0;
    // Used only with CHASSIS_ONLY, including a turret mechanically locked at a known bearing.
    public static double CHASSIS_ONLY_SHOOTER_HEADING_DEGREES = 0.0;

    public static double STARTUP_CENTER_ANGLE_DEGREES = 0.0;
    public static double STARTUP_SETTLE_SECONDS = 2.0;
    public static double STARTUP_TIMEOUT_SECONDS = 6.0;
    public static double STARTUP_MAX_CENTER_ERROR_DEGREES = 20.0;
    public static double HANDOFF_ANALOG_TOLERANCE_DEGREES = 5.0;
    public static double READY_TOLERANCE_DEGREES = 2.0;
    public static double READY_SPEED_DEGREES_PER_SECOND = 15.0;
    public static int READY_LOOPS_REQUIRED = 3;

    // These control settings and the fixed angle may be adjusted live in Panels.
    public static double RATE_LIMIT_DEGREES_PER_SECOND = 532.5;
    public static double OUTER_LOOP_KP = 0.12;
    public static double OUTER_LOOP_MAX_TRIM_DEGREES = 8.0;
    public static double FIXED_ANGLE_DEGREES = 0.0;
    // Existing autonomous paths already turn the chassis to shoot. Hold a constant turret angle.
    public static double AUTONOMOUS_HOLD_ANGLE_DEGREES = 0.0;

    private TurretController2026 controller;
    private TurretController2026.Config config;

    private TurretSubsystem2026() {}

    @Override
    public void initialize() {
        config = configuration();
        TurretController2026.Mode selected = MODE == null
                ? TurretController2026.Mode.CHASSIS_ONLY : MODE;
        try {
            TurretController2026.Hardware hardware = null;
            if (selected != TurretController2026.Mode.CHASSIS_ONLY) {
                // Mapping and sensor reads only: no position, direction, or PWM writes in Init.
                Servo left = ActiveOpMode.hardwareMap().get(Servo.class, LEFT_SERVO_NAME);
                Servo right = ActiveOpMode.hardwareMap().get(Servo.class, RIGHT_SERVO_NAME);
                DcMotor encoder = selected == TurretController2026.Mode.TRACKING
                        ? ActiveOpMode.hardwareMap().get(DcMotor.class, ENCODER_NAME) : null;
                AnalogInput absolute = selected == TurretController2026.Mode.TRACKING
                        ? ActiveOpMode.hardwareMap().get(AnalogInput.class, ABSOLUTE_ENCODER_NAME) : null;
                hardware = new TurretController2026.Hardware() {
                    private boolean directionsConfigured;
                    public int encoderTicks() { return encoder.getCurrentPosition(); }
                    public double absoluteVoltage() { return absolute.getVoltage(); }
                    public void writeServos(double l, double r) {
                        // Only reached after the controller has enabled outputs.
                        if (!directionsConfigured) {
                            left.setDirection(Servo.Direction.FORWARD);
                            right.setDirection(Servo.Direction.FORWARD);
                            directionsConfigured = true;
                        }
                        left.setPosition(l);
                        right.setPosition(r);
                    }
                };
            }
            controller = new TurretController2026(selected, config, hardware, now());
        } catch (IllegalArgumentException exception) {
            controller = new TurretController2026(selected, config, new TurretController2026.Hardware() {
                public int encoderTicks() { return 0; }
                public double absoluteVoltage() { return Double.NaN; }
                public void writeServos(double l, double r) { /* Fault prevents output. */ }
            }, now());
            controller.fail("Turret hardware unavailable: " + exception.getMessage());
        }
    }

    public void prepareTeleop() {
        if (GlobalRobotData.hasAutonRun) controller.restore(GlobalRobotData.endAutonTurret2026);
        // Consume even an invalid reference so it cannot be reused by a later practice run.
        GlobalRobotData.endAutonTurret2026 = null;
    }

    public void prepareAutonomous() {
        GlobalRobotData.endAutonTurret2026 = null;
        controller.start(now(), true); // Movement is allowed during autonomous Init.
    }

    public void startTeleop() { controller.start(now(), false); }

    @Override
    public void periodic() {
        config.rateLimit = RATE_LIMIT_DEGREES_PER_SECOND;
        config.outerKp = OUTER_LOOP_KP;
        config.maxTrim = OUTER_LOOP_MAX_TRIM_DEGREES;
        config.fixedAngle = FIXED_ANGLE_DEGREES;
        controller.update(now());
    }

    public void aimAt(double targetX, double targetY, double robotX, double robotY, double heading) {
        controller.aim(Math.toDegrees(Math.atan2(targetY - robotY, targetX - robotX)),
                Math.toDegrees(heading));
    }

    public boolean usesTurretAim() { return controller.getMode() == TurretController2026.Mode.TRACKING; }
    public boolean canShoot() { return controller.canShoot(now()); }
    public double getChassisAimOffsetRadians() {
        return Math.toRadians(controller.getMode() == TurretController2026.Mode.FIXED
                ? controller.getFixedRobotRelativeHeading() : CHASSIS_ONLY_SHOOTER_HEADING_DEGREES);
    }

    /** Save before stopping; this samples current counts, not the last loop's measured angle. */
    public void saveAutonomousHandoff() {
        GlobalRobotData.endAutonTurret2026 = controller.snapshot();
    }

    public void stop() { controller.stop(); } // No repositioning or PWM changes at Stop.

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret mode", controller.getMode());
        telemetry.addData("Turret state", controller.getState());
        telemetry.addData("Turret status", controller.getStatus());
        if (controller.getMode() == TurretController2026.Mode.CHASSIS_ONLY) return;
        telemetry.addData("Turret handoff restored", controller.isRestoredFromAutonomous());
        telemetry.addData("Turret angle / target", "%.1f / %.1f deg",
                controller.getMeasuredAngle(), controller.getTarget());
        telemetry.addData("Turret speed", "%.1f deg/s", controller.getVelocity());
        telemetry.addData("Turret analog phase", "%.1f deg", controller.getRawDegrees());
        telemetry.addData("Turret servo zero offset", "%.2f deg", controller.getServoOffset());
        telemetry.addData("Turret servo L / R", "%.4f / %.4f",
                controller.getLeftPosition(), controller.getRightPosition());
        telemetry.addData("Turret aim reachable / settled", "%s / %s",
                controller.isTargetReachable(), controller.isAimReady());
        telemetry.addData("Turret permits shooting", canShoot());
    }

    private static TurretController2026.Config configuration() {
        TurretController2026.Config c = new TurretController2026.Config();
        c.hardwareKey = LEFT_SERVO_NAME + ":" + RIGHT_SERVO_NAME + ":" + ENCODER_NAME
                + ":" + ABSOLUTE_ENCODER_NAME;
        c.encoderCpr = ENCODER_CPR;
        c.encoderRatio = ENCODER_TO_TURRET_RATIO;
        c.quadSign = QUAD_DIRECTION_SIGN;
        c.absoluteSign = ABSOLUTE_DIRECTION_SIGN;
        c.absoluteMaxVoltage = ABSOLUTE_MAX_VOLTAGE;
        c.absoluteOffset = ABSOLUTE_TURRET_OFFSET_DEGREES;
        c.servoCenter = SERVO_CENTER_POSITION;
        c.servoTravel = SERVO_TRAVEL_DEGREES;
        c.minAngle = MIN_ANGLE_DEGREES;
        c.maxAngle = MAX_ANGLE_DEGREES;
        c.minServo = MIN_SERVO_POSITION;
        c.maxServo = MAX_SERVO_POSITION;
        c.servoZeroOffset = SERVO_ZERO_OFFSET_DEGREES;
        c.differential = SERVO_DIFFERENTIAL_DEGREES;
        c.frontOffset = ROBOT_FRONT_TO_TURRET_ZERO_OFFSET_DEGREES;
        c.robotRelativeSign = ROBOT_FRONT_RELATIVE_SIGN;
        c.centerAngle = STARTUP_CENTER_ANGLE_DEGREES;
        c.settleSeconds = STARTUP_SETTLE_SECONDS;
        c.timeoutSeconds = STARTUP_TIMEOUT_SECONDS;
        c.maxCenterError = STARTUP_MAX_CENTER_ERROR_DEGREES;
        c.handoffTolerance = HANDOFF_ANALOG_TOLERANCE_DEGREES;
        c.readyTolerance = READY_TOLERANCE_DEGREES;
        c.readySpeed = READY_SPEED_DEGREES_PER_SECOND;
        c.readyLoops = READY_LOOPS_REQUIRED;
        c.rateLimit = RATE_LIMIT_DEGREES_PER_SECOND;
        c.outerKp = OUTER_LOOP_KP;
        c.maxTrim = OUTER_LOOP_MAX_TRIM_DEGREES;
        c.fixedAngle = FIXED_ANGLE_DEGREES;
        c.autonomousAngle = AUTONOMOUS_HOLD_ANGLE_DEGREES;
        return c;
    }

    private static double now() { return System.nanoTime() / 1e9; }
}
