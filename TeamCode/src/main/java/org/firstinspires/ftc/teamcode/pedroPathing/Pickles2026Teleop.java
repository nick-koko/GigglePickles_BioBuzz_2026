package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GlobalRobotData;
import org.firstinspires.ftc.teamcode.subsystems.FlowerIntakeSubsystem2026;
import org.firstinspires.ftc.teamcode.subsystems.IntakeWithSensorsSubsystem2026;
import org.firstinspires.ftc.teamcode.subsystems.LEDControlSubsystem2026;
import org.firstinspires.ftc.teamcode.subsystems.ShotSequenceController2026;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem2026;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem2026;
import org.firstinspires.ftc.teamcode.subsystems.shot.ShotCalibrationTable2026;
import org.firstinspires.ftc.teamcode.util.HeadingTurnController2026;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/**
 * Minimal driver-controlled OpMode for the 2026 BioBuzz robot.
 *
 * <p>This records alliance and accepts an autonomous pose handoff, but
 * intentionally has no alliance-specific driving, camera, or 2025
 * three-ball state-machine dependencies.</p>
 */
@Configurable
@TeleOp(name = "Pickles 2026 Minimal Teleop", group = "Comp")
public class Pickles2026Teleop extends NextFTCOpMode {

    public static double START_X_INCHES = 54.16;
    public static double START_Y_INCHES = 8.5;
    public static double START_HEADING_DEGREES = 90.0;

    public static double DRIVE_POWER = 1.0;
    public static double TURN_POWER = 0.5;
    public static double SHOOTER_FALLBACK_RPM = 3175.0;
    public static double SHOOTER_RPM_STEP = 25.0;
    // GP2 D-pad adjusts this additive trim in either regression or table mode.
    public static double SHOOTER_RPM_TRIM = 0.0;
    public static boolean USE_SHOT_TABLE = false;
    public static double REGRESSION_RPM_PER_INCH = 16.2; //26.5
    public static double REGRESSION_RPM_INTERCEPT = 2410; //2015
    // Shows only the values needed while tuning shots on the Driver Station.
    public static boolean SHOT_TUNING_TELEMETRY_ONLY = false;

    // Fixed X aim for all robot positions; Y target still switches by Y position.
    public static double SHOOT_TARGET_X = 58.0;
    public static double SHOOT_TARGET_Y = 60.0;
    public static double SHOOT_TARGET_Y_SPLIT = 72.0;
    public static double SHOOT_TARGET_Y_ABOVE_SPLIT = 84.0;
    public static boolean AUTO_AIM_STARTS_SHOOTER = true;

    // TODO: enter the measured limits. Zero disables that bound; units are inches
    // from the robot's Pedro pose to the currently selected shooting target.
    public static double MIN_SHOOT_DISTANCE_INCHES = 37.7;
    public static double MAX_SHOOT_DISTANCE_INCHES = 70.0;
    public static boolean SHOOT_RANGE_WARNINGS_ENABLED = true;
    // Keeps ball-count LEDs visible while collecting. False warns anywhere out of range.
    public static boolean SHOOT_RANGE_WARNING_ONLY_WHILE_AIMING_OR_FIRING = true;

    // false: custom P/D below. true: Pedro heading PIDF gains from Constants.
    public static boolean USE_PEDRO_AIM_PIDF = false;
    public static double AIM_TURN_KP_PER_DEGREE = 0.012;
    // Power per degree/second of measured yaw rate; zero disables damping.
    public static double AIM_TURN_KD_PER_DEGREE_PER_SECOND = 0.001;
    public static double CARDINAL_TURN_KP_PER_DEGREE = 0.009;
    // Applied to either controller: add kS, then enforce the minimum, then cap.
    // Both can be zero. Compensation is omitted during braking/inside tolerance.
    public static double AUTO_TURN_MIN_POWER = 0.0;
    // The old MIN_POWER was additive; this preserves that behavior by default.
    public static double AUTO_TURN_KS = 0.04; //0.052;
    public static double AUTO_TURN_MAX_POWER = 0.60;
    public static double AUTO_TURN_TOLERANCE_DEGREES = 1.0;
    public static double AUTO_TURN_SETTLE_SPEED_DEGREES_PER_SECOND = 5.0;
    public static boolean ENABLE_DPAD_TURNING = false;

    // Historical GP1 D-pad field-heading mapping.
    public static double DPAD_UP_HEADING_DEGREES = 0.0;
    public static double DPAD_LEFT_HEADING_DEGREES = 90.0;
    public static double DPAD_DOWN_HEADING_DEGREES = 180.0;
    public static double DPAD_RIGHT_HEADING_DEGREES = -90.0;

    private Pose startingPose;
    private boolean selectAllianceSide;
    private boolean startedFromAuton;
    private HeadingTurnController2026 headingTurnController;
    private boolean automaticTurnWasActive;
    private boolean previousUsePedroAimPidf;
    private boolean previousTableSolutionActive;
    private double previousAimTargetX;
    private double previousAimTargetY;
    private double automaticTurnRawPower;
    private final ShotSequenceController2026 shotSequence =
            new ShotSequenceController2026();

    public Pickles2026Teleop() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(
                        ShooterSubsystem2026.INSTANCE,
                        IntakeWithSensorsSubsystem2026.INSTANCE,
                        LEDControlSubsystem2026.INSTANCE,
                        FlowerIntakeSubsystem2026.INSTANCE,
                        TurretSubsystem2026.INSTANCE
                ),
                BulkReadComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        // Restore the turret reference before consuming the autonomous handoff.
        // This only reads sensors; even a missing reference cannot move the turret in Init.
        TurretSubsystem2026.INSTANCE.prepareTeleop();
        LEDControlSubsystem2026.INSTANCE.setShootRangeWarning(false);
        IntakeWithSensorsSubsystem2026.INSTANCE.stopAll();
        ShooterSubsystem2026.INSTANCE.stop();
        shotSequence.reset();

        startedFromAuton =
                GlobalRobotData.hasAutonRun && GlobalRobotData.endAutonPose != null;

        if (startedFromAuton) {
            startingPose = GlobalRobotData.endAutonPose.copy();
            selectAllianceSide = false;
            if (GlobalRobotData.endAutonBallCount >= 0) {
                IntakeWithSensorsSubsystem2026.INSTANCE.setBallCount(
                        GlobalRobotData.endAutonBallCount
                );
            }

            // Consume the handoff so another TeleOp cannot accidentally reuse it.
            GlobalRobotData.hasAutonRun = false;
            GlobalRobotData.endAutonPose = null;
            GlobalRobotData.endAutonBallCount = -1;
            GlobalRobotData.endAutonTurretAngleDegrees = Double.NaN;
            GlobalRobotData.endAutonTurretServoCommandAngleDegrees = Double.NaN;
            GlobalRobotData.endAutonTurretServoOffsetDegrees = Double.NaN;
        } else {
            startingPose = configuredStartingPose();
            selectAllianceSide = true;

            // Clear an incomplete handoff (flag set without a valid pose).
            GlobalRobotData.hasAutonRun = false;
            GlobalRobotData.endAutonPose = null;
        }

        //PedroComponent.follower().setStartingPose(startingPose);
    }

    @Override
    public void onWaitForStart() {
        IntakeWithSensorsSubsystem2026 intake = IntakeWithSensorsSubsystem2026.INSTANCE;

        if (selectAllianceSide) {
            if (gamepad1.xWasPressed()) {
                GlobalRobotData.allianceSide = GlobalRobotData.COLOR.BLUE;
            } else if (gamepad1.bWasPressed()) {
                GlobalRobotData.allianceSide = GlobalRobotData.COLOR.RED;
            }
        }

        telemetry.addLine("Hello Pickle of the robot");
        telemetry.addLine("This is an Mr. Todone Speaking,");
        telemetry.addLine("----------------------------------------------");
        if (GlobalRobotData.allianceSide == GlobalRobotData.COLOR.BLUE) {
            telemetry.addLine("Favorite fruit: Blueberries!!! (Blue)");
        } else {
            telemetry.addLine("Favorite fruit: Raspberries!!! (Red)");
        }

        if (selectAllianceSide) {
            telemetry.addLine("Press X for BLUE or B for RED");
        }

        telemetry.addLine("Pickles 2026 ready");
        telemetry.addLine("GP1: left stick drive/strafe, right stick turn");
        telemetry.addLine("GP1: hold LB to aim chassis or turret at shooting target");
        telemetry.addLine("GP1 D-pad: face absolute 0/90/180/-90 headings");
        telemetry.addLine("GP2 D-pad up/down: trim calculated shooter RPM");
        telemetry.addLine("GP1/GP2: A intake/re-arm, hold B reverse, X stop");
        telemetry.addLine("GP1/GP2: RB shooter toggle, press RT for timed feed");
        telemetry.addLine("GP2: LB toggles flower intake up/down");
        telemetry.addData("Starting pose source", startedFromAuton ? "AUTON" : "DEFAULT");
        telemetry.addData(
                "Starting pose",
                "(%.1f, %.1f, %.1f deg)",
                startingPose.getX(),
                startingPose.getY(),
                Math.toDegrees(startingPose.getHeading())
        );
        telemetry.addData("Tracked balls", intake.getBallCount());
        telemetry.addData(
                "Ball sensors",
                "shooter %s | middle-high %s | middle-low %s | entry %s",
                beamState(intake.isBallAtShooterBeamBroken()),
                beamState(intake.isMiddleHighBeamBroken()),
                beamState(intake.isMiddleLowBeamBroken()),
                beamState(intake.isIntakeEntryBeamBroken())
        );
        telemetry.addData("Shooter gate", "%.3f", intake.getGatePosition());
        telemetry.addData("Shot RPM selection", USE_SHOT_TABLE ? "TABLE (regression if empty)" : "REGRESSION");
        TurretSubsystem2026.INSTANCE.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        TurretSubsystem2026.INSTANCE.startTeleop();
        PedroComponent.follower().setStartingPose(startingPose);
        PedroComponent.follower().startTeleopDrive();
        headingTurnController = new HeadingTurnController2026(
                PedroComponent.follower().getConstants()
        );
        automaticTurnWasActive = false;
        FlowerIntakeSubsystem2026.INSTANCE.moveUp();
        IntakeWithSensorsSubsystem2026.INSTANCE.intakeForward();
        ShooterSubsystem2026.INSTANCE.spinUp(calculateShotSettings(startingPose).rpm);
        shotSequence.reset();
    }

    @Override
    public void onUpdate() {
        IntakeWithSensorsSubsystem2026 intake = IntakeWithSensorsSubsystem2026.INSTANCE;
        ShooterSubsystem2026 shooter = ShooterSubsystem2026.INSTANCE;
        FlowerIntakeSubsystem2026 flower = FlowerIntakeSubsystem2026.INSTANCE;
        LEDControlSubsystem2026 leds = LEDControlSubsystem2026.INSTANCE;
        TurretSubsystem2026 turret = TurretSubsystem2026.INSTANCE;

        if (gamepad2.dpadUpWasPressed()) {
            SHOOTER_RPM_TRIM += Math.abs(SHOOTER_RPM_STEP);
        }
        if (gamepad2.dpadDownWasPressed()) {
            SHOOTER_RPM_TRIM -= Math.abs(SHOOTER_RPM_STEP);
        }
        if (gamepad2.left_bumper) {
            flower.moveDown();
        }
        else {
            flower.moveUp();
        }

        double drive = -gamepad1.left_stick_y * DRIVE_POWER;
        double strafe = -gamepad1.left_stick_x * DRIVE_POWER;
        double turn = -gamepad1.right_stick_x * TURN_POWER;

        Pose pose = PedroComponent.follower().getPose();
        double angularVelocityRadiansPerSecond =
                PedroComponent.follower().getPoseTracker().getAngularVelocity();
        ShotSettings shot = calculateShotSettings(pose);
        double targetX = shot.aimX;
        double targetY = shot.aimY;
        double targetDx = targetX - pose.getX();
        double targetDy = targetY - pose.getY();
        // Range warnings and regression use distance to the nominal physical target,
        // not the calibrated aim point (which may intentionally be above/beside it).
        double targetDistanceInches = shot.distanceInches;
        double targetHeadingRadians = Math.atan2(targetDy, targetDx);

        String turnMode = "MANUAL";
        double commandedHeadingRadians = Double.NaN;
        double headingErrorRadians = 0.0;
        automaticTurnRawPower = 0.0;

        boolean aimRequested = gamepad1.left_bumper || gamepad2.right_bumper;
        if (aimRequested) {
            if (turret.usesTurretAim()) {
                turret.aimAt(targetX, targetY, pose.getX(), pose.getY(), pose.getHeading());
                turnMode = "TURRET TARGET / MANUAL CHASSIS";
            } else {
                if (!automaticTurnWasActive || previousUsePedroAimPidf != USE_PEDRO_AIM_PIDF
                        || shot.nominalX != previousAimTargetX || shot.nominalY != previousAimTargetY
                        || (shot.table != null) != previousTableSolutionActive) {
                    headingTurnController.reset();
                }
                commandedHeadingRadians = targetHeadingRadians - turret.getChassisAimOffsetRadians();
                headingErrorRadians = normalizeRadians(commandedHeadingRadians - pose.getHeading());
                turn = calculateAutomaticTurn(
                        pose,
                        commandedHeadingRadians,
                        headingErrorRadians,
                        angularVelocityRadiansPerSecond,
                        AIM_TURN_KP_PER_DEGREE
                );
                turnMode = "SHOOT TARGET";
            }

        } /* else if (ENABLE_DPAD_TURNING) {
            if (gamepad1.dpad_up) {
                commandedHeadingRadians =
                        Math.toRadians(DPAD_UP_HEADING_DEGREES);
                turnMode = "DPAD UP";
            } else if (gamepad1.dpad_left) {
                commandedHeadingRadians =
                        Math.toRadians(DPAD_LEFT_HEADING_DEGREES);
                turnMode = "DPAD LEFT";
            } else if (gamepad1.dpad_down) {
                commandedHeadingRadians =
                        Math.toRadians(DPAD_DOWN_HEADING_DEGREES);
                turnMode = "DPAD DOWN";
            } else if (gamepad1.dpad_right) {
                commandedHeadingRadians =
                        Math.toRadians(DPAD_RIGHT_HEADING_DEGREES);
                turnMode = "DPAD RIGHT";
            }

            if (Double.isFinite(commandedHeadingRadians)) {
                headingErrorRadians = normalizeRadians(
                        commandedHeadingRadians - pose.getHeading()
                );
                turn = calculateAutomaticTurn(
                        pose,
                        commandedHeadingRadians,
                        headingErrorRadians,
                        angularVelocityRadiansPerSecond,
                        CARDINAL_TURN_KP_PER_DEGREE
                );
            }
        } */

        automaticTurnWasActive = aimRequested && !turret.usesTurretAim();
        previousUsePedroAimPidf = USE_PEDRO_AIM_PIDF;
        // Smoothly changing table offsets must not re-prime Pedro's PIDF every loop.
        previousAimTargetX = shot.nominalX;
        previousAimTargetY = shot.nominalY;
        previousTableSolutionActive = shot.table != null;

        PedroComponent.follower().setTeleOpDrive(
                drive,
                strafe,
                turn,
                false // Field centric
        );

        if (gamepad1.aWasPressed() || gamepad2.aWasPressed() || (flower.getPosition() == FlowerIntakeSubsystem2026.Position.DOWN)) {
            intake.intakeForward();
        }
        boolean stopFeedWasPressed =
                gamepad1.xWasPressed() || gamepad2.xWasPressed();
        if (stopFeedWasPressed) {
            shotSequence.cancel();
            intake.stopIntake();
        } else {
            intake.setJamReversing(gamepad1.b || gamepad2.b);
        }
        updateShooter(shot.rpm, aimRequested, gamepad1.rightBumperWasPressed());

        // Calculate shooting range before processing trigger inputs.
        boolean limitsValid = Double.isFinite(MIN_SHOOT_DISTANCE_INCHES)
                && Double.isFinite(MAX_SHOOT_DISTANCE_INCHES)
                && MIN_SHOOT_DISTANCE_INCHES >= 0 && MAX_SHOOT_DISTANCE_INCHES >= 0
                && (MIN_SHOOT_DISTANCE_INCHES == 0 || MAX_SHOOT_DISTANCE_INCHES == 0
                || MIN_SHOOT_DISTANCE_INCHES <= MAX_SHOOT_DISTANCE_INCHES);
        boolean tooClose = limitsValid && MIN_SHOOT_DISTANCE_INCHES > 0
                && targetDistanceInches < MIN_SHOOT_DISTANCE_INCHES;
        boolean tooFar = limitsValid && MAX_SHOOT_DISTANCE_INCHES > 0
                && targetDistanceInches > MAX_SHOOT_DISTANCE_INCHES;

        // The flywheel may remain spinning, but no balls may feed outside the range.
        // Also preserve the turret's existing firing interlock.
        boolean canFire = !tooClose && !tooFar && turret.canShoot();
        if (!canFire) {
            shotSequence.cancel();
        }
        shotSequence.update(
                canFire && shotSequence.isTriggerPressed(
                        gamepad1.right_trigger,
                        gamepad2.right_trigger
                )
        );

        boolean firingRequested = shotSequence.isTriggerPressed(
                gamepad1.right_trigger, gamepad2.right_trigger) && !tooFar && !tooClose;
        leds.setShootRangeWarning(SHOOT_RANGE_WARNINGS_ENABLED && (tooClose || tooFar)
                && (!SHOOT_RANGE_WARNING_ONLY_WHILE_AIMING_OR_FIRING
                || aimRequested || firingRequested || shotSequence.isActive()));
        String shootingRangeStatus = !SHOOT_RANGE_WARNINGS_ENABLED ? "DISABLED"
                : !limitsValid ? "INVALID LIMITS"
                : MIN_SHOOT_DISTANCE_INCHES == 0 && MAX_SHOOT_DISTANCE_INCHES == 0 ? "SET MIN/MAX LIMITS"
                : !Double.isFinite(targetDistanceInches) ? "UNKNOWN DISTANCE"
                : tooClose ? "TOO CLOSE" : tooFar ? "TOO FAR" : "IN RANGE";

        if (SHOT_TUNING_TELEMETRY_ONLY) {
            addShotSettingsTelemetry(shot);
            addShootingRangeTelemetry(shootingRangeStatus);
            turret.addTelemetry(telemetry);
            double shooterTargetRpm = shooter.getTargetFlywheelRpm() > 0.0
                    ? shooter.getTargetFlywheelRpm()
                    : shot.rpm;
            telemetry.addData("Robot X", "%.1f in", pose.getX());
            telemetry.addData("Robot Y", "%.1f in", pose.getY());
            telemetry.addData("Robot angle", "%.1f deg", Math.toDegrees(pose.getHeading()));
            telemetry.addData("Target X", "%.1f in", targetX);
            telemetry.addData("Target Y", "%.1f in", targetY);
            telemetry.addData("Distance to nominal target", "%.1f in", targetDistanceInches);
            telemetry.addData("Shooter target RPM", "%.0f", shooterTargetRpm);
            telemetry.addData("Flywheel measured RPM", "%.0f", shooter.getMeasuredFlywheelRpm());
            telemetry.update();
            return;
        }

        telemetry.addData("Pose X", "%.1f in", pose.getX());
        addShotSettingsTelemetry(shot);
        addShootingRangeTelemetry(shootingRangeStatus);
        turret.addTelemetry(telemetry);
        telemetry.addData("Pose Y", "%.1f in", pose.getY());
        telemetry.addData("Heading", "%.1f deg", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Turn mode", turnMode);
        telemetry.addData("Aim controller", USE_PEDRO_AIM_PIDF ? "PEDRO PIDF" : "CUSTOM PD");
        telemetry.addData("Custom aim gains", "P %.5f | D %.5f", AIM_TURN_KP_PER_DEGREE,
                AIM_TURN_KD_PER_DEGREE_PER_SECOND);
        telemetry.addData("Turn yaw rate", "%.1f deg/s", Math.toDegrees(angularVelocityRadiansPerSecond));
        telemetry.addData("Turn power", "base %.3f | command %.3f", automaticTurnRawPower, turn);
        telemetry.addData("Turn friction", "minimum %.3f | kS %.3f", AUTO_TURN_MIN_POWER, AUTO_TURN_KS);
        telemetry.addData(
                "Turn target",
                Double.isFinite(commandedHeadingRadians)
                        ? String.format(
                        "%.1f deg",
                        Math.toDegrees(commandedHeadingRadians)
                )
                        : "MANUAL"
        );
        telemetry.addData(
                "Turn error",
                "%.1f deg",
                Math.toDegrees(headingErrorRadians)
        );
        telemetry.addData(
                "Shooting target",
                "aim (%.1f, %.1f), nominal distance %.1f in",
                targetX,
                targetY,
                targetDistanceInches
        );
        telemetry.addData("Alliance", GlobalRobotData.allianceSide);
        telemetry.addData("Pose source", startedFromAuton ? "AUTON" : "DEFAULT");
        telemetry.addData("Flower intake", flower.getPosition());
        telemetry.addData(
                "Flower servo position",
                "%.3f",
                flower.getServoPosition()
        );
        telemetry.addData("Tracked balls", "%d / 4", intake.getBallCount());
        telemetry.addData(
                "Ball-count LEDs",
                "%s (%.0f us)",
                leds.getActivePreset(),
                leds.getTargetPulseUs()
        );
        telemetry.addData(
                "Ball tracking",
                IntakeWithSensorsSubsystem2026.ENABLE_BALL_TRACKING
                        ? "ENABLED"
                        : "IGNORED"
        );
        telemetry.addData(
                "Ball sensors",
                "shooter %s | middle-high %s | middle-low %s | entry %s",
                beamState(intake.isBallAtShooterBeamBroken()),
                beamState(intake.isMiddleHighBeamBroken()),
                beamState(intake.isMiddleLowBeamBroken()),
                beamState(intake.isIntakeEntryBeamBroken()),
                beamState(intake.isIntakeEntryBeamBroken()));
        telemetry.addData(
                "Tracked positions",
                "shooter %s | middle-high %s | middle-low %s | entry %s",
                occupancyState(intake.isShooterPositionOccupied()),
                occupancyState(intake.isMiddleHighPositionOccupied()),
                occupancyState(intake.isMiddleLowPositionOccupied()),
                occupancyState(intake.isIntakeEntryPositionOccupied())
        );
        telemetry.addData("Intake mode", intake.getIntakeMode());
        telemetry.addData("Jam reverse", intake.isJamReversing());
        telemetry.addData(
                "Transfer",
                intake.isTransferStoppedForCapacity()
                        ? "STOPPED - UPPER 2 FULL"
                        : (intake.isFeedingToShooter() ? "FEEDING SHOOTER" : "NORMAL")
        );
        telemetry.addData(
                "Intake capacity stop",
                intake.isIntakeStoppedForCapacity()
                        ? "STOPPED - 4 BALLS"
                        : "NOT FULL"
        );
        telemetry.addData("Intake target motor RPM", "%.0f", intake.getAppliedIntakeRpm());
        telemetry.addData("Intake measured motor RPM", "%.0f", intake.getMeasuredIntakeMotorRpm());
        telemetry.addData("Intake measured shaft RPM", "%.0f", intake.getMeasuredIntakeShaftRpm());
        telemetry.addData("Transfer target motor RPM", "%.0f", intake.getAppliedTransferRpm());
        telemetry.addData("Transfer measured motor RPM", "%.0f", intake.getMeasuredTransferMotorRpm());
        telemetry.addData("Transfer measured shaft RPM", "%.0f", intake.getMeasuredTransferShaftRpm());
        telemetry.addData("Shoot intake power", "%.2f", intake.getAppliedIntakePower());
        telemetry.addData("Shoot transfer power", "%.2f", intake.getAppliedTransferPower());
        telemetry.addData("Shooter gate", "%.3f", intake.getGatePosition());
        telemetry.addData("Shot sequence active", shotSequence.isActive());
        telemetry.addData(
                "Shot sequence time",
                "%.2f / %.2f sec",
                shotSequence.getElapsedSeconds(),
                shotSequence.getActiveFeedSeconds()
        );
        telemetry.addData("Shot boost active", shotSequence.isBoostActive());
        telemetry.addData("Shooter", shooter.isEnabled() ? "ON" : "OFF");
        telemetry.addData("Shooter target flywheel RPM", "%.0f", shooter.getTargetFlywheelRpm());
        telemetry.addData("Shooter measured flywheel RPM", "%.0f", shooter.getMeasuredFlywheelRpm());
        telemetry.addData("Shooter target motor RPM", "%.0f", shooter.getTargetMotorRpm());
        telemetry.addData("Shooter measured motor RPM", "%.0f", shooter.getMeasuredMotorRpm());
        telemetry.addData("Shooter RPM error", "%.0f", shooter.getErrorRpm());
        telemetry.addData(
                "Shooter PID + FF",
                "P %.3f  I %.3f  D %.3f  FF %.3f",
                shooter.getPTerm(),
                shooter.getITerm(),
                shooter.getDTerm(),
                shooter.getFeedforward()
        );
        telemetry.addData(
                "Shooter power",
                "requested %.3f  applied %.3f",
                shooter.getOutputBeforeSlew(),
                shooter.getAppliedPower()
        );
        telemetry.addData(
                "Shooter limiting",
                "slew %s  saturated %s",
                shooter.wasSlewLimited(),
                shooter.isCommandSaturated()
        );
        telemetry.addData(
                "Shooter battery",
                "raw %.2f V  filtered %.2f V  gain %.3f",
                shooter.getBatteryVoltageRaw(),
                shooter.getBatteryVoltageFiltered(),
                shooter.getVoltageCompGain()
        );
        telemetry.addData("Shooter current", "%.2f A", shooter.getMotorCurrentAmps());
        telemetry.update();
    }

    @Override
    public void onStop() {
        LEDControlSubsystem2026.INSTANCE.setShootRangeWarning(false);
        TurretSubsystem2026.INSTANCE.stop();
        shotSequence.cancel();
        IntakeWithSensorsSubsystem2026.INSTANCE.stopAll();
        ShooterSubsystem2026.INSTANCE.stop();
        super.onStop();
    }

    private static Pose configuredStartingPose() {
        return new Pose(
                START_X_INCHES,
                START_Y_INCHES,
                Math.toRadians(START_HEADING_DEGREES)
        );
    }

    private void addShootingRangeTelemetry(String status) {
        telemetry.addData("Shooting range", status);
        telemetry.addData("Shooting range limits", "min %.1f / max %.1f in (0 = disabled)",
                MIN_SHOOT_DISTANCE_INCHES, MAX_SHOOT_DISTANCE_INCHES);
        telemetry.addData("Range warning LEDs", LEDControlSubsystem2026.INSTANCE.isShootRangeWarningActive());
    }

    private static void updateShooter(double rpm, boolean aimRequested, boolean toggleRequested) {
        ShooterSubsystem2026 shooter = ShooterSubsystem2026.INSTANCE;
        boolean runShooter = shooter.isEnabled() || (aimRequested && AUTO_AIM_STARTS_SHOOTER);
        if (toggleRequested) runShooter = !runShooter;
        // One final RPM command. The selected regression/table result is never
        // overwritten by a fixed target later in the loop.
        if (runShooter) shooter.spinUp(rpm);
        else if (shooter.isEnabled()) shooter.stop();
    }

    private static ShotSettings calculateShotSettings(Pose pose) {
        // Always aim at X = 58, regardless of the robot's X position.
        double nominalX = SHOOT_TARGET_X;
        double nominalY = pose.getY() > SHOOT_TARGET_Y_SPLIT
                ? SHOOT_TARGET_Y_ABOVE_SPLIT : SHOOT_TARGET_Y;
        double distance = Math.hypot(nominalX - pose.getX(), nominalY - pose.getY());
        ShotCalibrationTable2026.Solution table = USE_SHOT_TABLE
                ? ShotCalibrationTable2026.lookup(pose.getX(), pose.getY()) : null;
        double baseRpm = table == null ? calculateShooterRpmForDistance(distance) : table.flywheelRpm;
        String source = table != null ? "TABLE"
                : USE_SHOT_TABLE ? "REGRESSION (no valid table rows)" : "REGRESSION";
        if (!Double.isFinite(baseRpm) || baseRpm <= 0) {
            baseRpm = Double.isFinite(SHOOTER_FALLBACK_RPM) ? Math.max(0, SHOOTER_FALLBACK_RPM) : 0;
            table = null;
            source = "FIXED FALLBACK (invalid calculation)";
        }
        double trim = Double.isFinite(SHOOTER_RPM_TRIM) ? SHOOTER_RPM_TRIM : 0;
        return new ShotSettings(nominalX, nominalY, distance, baseRpm,
                Math.max(0, baseRpm + trim), table, source);
    }

    private void addShotSettingsTelemetry(ShotSettings shot) {
        telemetry.addData("Shot RPM source", shot.source);
        telemetry.addData("Calculated RPM / trim", "%.0f / %+.0f", shot.baseRpm, SHOOTER_RPM_TRIM);
        telemetry.addData("Nominal shooting target", "(%.1f, %.1f)", shot.nominalX, shot.nominalY);
        telemetry.addData("Aim delta X / Y", "%+.2f / %+.2f in",
                shot.aimX - shot.nominalX, shot.aimY - shot.nominalY);
        if (shot.table != null) {
            telemetry.addData("Shot table samples / nearest", "%d / %.1f in",
                    shot.table.samplesUsed, shot.table.nearestSampleDistanceInches);
        }
    }

    private static final class ShotSettings {
        final double nominalX, nominalY, aimX, aimY, distanceInches, baseRpm, rpm;
        final ShotCalibrationTable2026.Solution table;
        final String source;

        ShotSettings(double nominalX, double nominalY, double distance, double baseRpm, double rpm,
                     ShotCalibrationTable2026.Solution table, String source) {
            this.nominalX = nominalX;
            this.nominalY = nominalY;
            aimX = nominalX; // Keep X fixed even when shot-table calibration is enabled.
            aimY = nominalY + (table == null ? 0 : table.deltaAimY);
            distanceInches = distance;
            this.baseRpm = baseRpm;
            this.rpm = rpm;
            this.table = table;
            this.source = source;
        }
    }

    private static double calculateShooterRpmForDistance(double distanceInches) {
        return REGRESSION_RPM_PER_INCH * distanceInches + REGRESSION_RPM_INTERCEPT;
    }

    private double calculateAutomaticTurn(
            Pose pose,
            double targetHeadingRadians,
            double headingErrorRadians,
            double angularVelocityRadiansPerSecond,
            double proportionalGainPerDegree
    ) {
        automaticTurnRawPower = USE_PEDRO_AIM_PIDF
                ? headingTurnController.calculatePedro(pose, targetHeadingRadians)
                : HeadingTurnController2026.calculateCustom(
                headingErrorRadians,
                angularVelocityRadiansPerSecond,
                proportionalGainPerDegree,
                AIM_TURN_KD_PER_DEGREE_PER_SECOND,
                AUTO_TURN_TOLERANCE_DEGREES
        );
        return HeadingTurnController2026.applyCompensationAndLimits(
                automaticTurnRawPower,
                headingErrorRadians,
                angularVelocityRadiansPerSecond,
                AUTO_TURN_MIN_POWER,
                AUTO_TURN_KS,
                AUTO_TURN_MAX_POWER,
                AUTO_TURN_TOLERANCE_DEGREES,
                AUTO_TURN_SETTLE_SPEED_DEGREES_PER_SECOND
        );
    }

    private static double normalizeRadians(double angle) {
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }

    private static String beamState(boolean broken) {
        return broken ? "BROKEN" : "clear";
    }

    private static String occupancyState(boolean occupied) {
        return occupied ? "OCCUPIED" : "empty";
    }
}