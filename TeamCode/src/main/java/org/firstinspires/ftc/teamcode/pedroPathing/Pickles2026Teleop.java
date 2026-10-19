package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.GlobalRobotData;
import org.firstinspires.ftc.teamcode.subsystems.IntakeWithSensorsSubsystem2026;
import org.firstinspires.ftc.teamcode.subsystems.ShotSequenceController2026;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem2026;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/**
 * Minimal driver-controlled OpMode for the 2026 BioBuzz robot.
 *
 * <p>This records alliance and accepts an autonomous pose handoff, but
 * intentionally has no alliance-specific driving, camera, turret, or 2025
 * three-ball state-machine dependencies.</p>
 */
@Configurable
@TeleOp(name = "Pickles 2026 Minimal Teleop", group = "Comp")
public class Pickles2026Teleop extends NextFTCOpMode {

    public static double START_X_INCHES = 58;
    public static double START_Y_INCHES = 8.5;
    public static double START_HEADING_DEGREES = 90.0;

    public static double DRIVE_POWER = 1.0;
    public static double TURN_POWER = 0.5;
    public static double SHOOTER_TARGET_RPM = 3600.0;
    public static double SHOOTER_RPM_STEP = 25.0;

    // Temporary fixed target until distance-based shot calculation is restored.
    public static double SHOOT_TARGET_X_INCHES = 144.0;
    public static double SHOOT_TARGET_Y_INCHES = 136.0;
    public static double SHOOT_TARGET_Y_SPLIT_INCHES = 106.0;
    public static double SHOOT_TARGET_Y_OFFSET_BELOW_SPLIT = 3.0;
    public static double SHOOT_TARGET_Y_OFFSET_ABOVE_SPLIT = -1.0;
    public static boolean AUTO_AIM_STARTS_SHOOTER = true;

    public static double AIM_TURN_KP_PER_DEGREE = 0.0165;
    public static double CARDINAL_TURN_KP_PER_DEGREE = 0.009;
    public static double AUTO_TURN_MIN_POWER = 0.075;
    public static double AUTO_TURN_MAX_POWER = 0.80;
    public static double AUTO_TURN_TOLERANCE_DEGREES = 1.0;
    public static boolean ENABLE_DPAD_TURNING = false;

    // Historical GP1 D-pad field-heading mapping.
    public static double DPAD_UP_HEADING_DEGREES = 0.0;
    public static double DPAD_LEFT_HEADING_DEGREES = 90.0;
    public static double DPAD_DOWN_HEADING_DEGREES = 180.0;
    public static double DPAD_RIGHT_HEADING_DEGREES = -90.0;

    private Pose startingPose;
    private boolean selectAllianceSide;
    private boolean startedFromAuton;
    private final ShotSequenceController2026 shotSequence =
            new ShotSequenceController2026();

    public Pickles2026Teleop() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(
                        ShooterSubsystem2026.INSTANCE,
                        IntakeWithSensorsSubsystem2026.INSTANCE
                ),
                BulkReadComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        IntakeWithSensorsSubsystem2026.INSTANCE.stopAll();
        ShooterSubsystem2026.INSTANCE.stop();
        shotSequence.reset();

        startedFromAuton =
                GlobalRobotData.hasAutonRun && GlobalRobotData.endAutonPose != null;

        if (startedFromAuton) {
            startingPose = GlobalRobotData.endAutonPose.copy();
            selectAllianceSide = false;

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
        telemetry.addLine("GP1: hold LB to face shooting target");
        telemetry.addLine("GP1 D-pad: face absolute 0/90/180/-90 headings");
        telemetry.addLine("GP2 D-pad up/down: adjust shooter RPM");
        telemetry.addLine("GP1/GP2: A intake/re-arm, hold B reverse, X stop");
        telemetry.addLine("GP1/GP2: RB shooter toggle, press RT for timed feed");
        telemetry.addData("Starting pose source", startedFromAuton ? "AUTON" : "DEFAULT");
        telemetry.addData(
                "Starting pose",
                "(%.1f, %.1f, %.1f deg)",
                startingPose.getX(),
                startingPose.getY(),
                Math.toDegrees(startingPose.getHeading())
        );
        telemetry.addData("Top beam", intake.isTopBeamBroken() ? "BROKEN" : "CLEAR");
        telemetry.addData("Shooter gate", "%.3f", intake.getGatePosition());
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        PedroComponent.follower().setStartingPose(startingPose);
        PedroComponent.follower().startTeleopDrive();
        shotSequence.reset();
    }

    @Override
    public void onUpdate() {
        IntakeWithSensorsSubsystem2026 intake = IntakeWithSensorsSubsystem2026.INSTANCE;
        ShooterSubsystem2026 shooter = ShooterSubsystem2026.INSTANCE;

        if (gamepad2.dpadUpWasPressed()) {
            SHOOTER_TARGET_RPM += Math.abs(SHOOTER_RPM_STEP);
        }
        if (gamepad2.dpadDownWasPressed()) {
            SHOOTER_TARGET_RPM = Math.max(
                    0.0,
                    SHOOTER_TARGET_RPM - Math.abs(SHOOTER_RPM_STEP)
            );
        }

        double drive = -gamepad1.left_stick_y * DRIVE_POWER;
        double strafe = -gamepad1.left_stick_x * DRIVE_POWER;
        double turn = -gamepad1.right_stick_x * TURN_POWER;

        Pose pose = PedroComponent.follower().getPose();
        double targetY = SHOOT_TARGET_Y_INCHES
                + (pose.getY() > SHOOT_TARGET_Y_SPLIT_INCHES
                ? SHOOT_TARGET_Y_OFFSET_ABOVE_SPLIT
                : SHOOT_TARGET_Y_OFFSET_BELOW_SPLIT);
        double targetDx = SHOOT_TARGET_X_INCHES - pose.getX();
        double targetDy = targetY - pose.getY();
        double targetDistanceInches = Math.hypot(targetDx, targetDy);
        double targetHeadingRadians = Math.atan2(targetDy, targetDx);

        String turnMode = "MANUAL";
        double commandedHeadingRadians = Double.NaN;
        double headingErrorRadians = 0.0;

        if (gamepad1.left_bumper) {
            commandedHeadingRadians = targetHeadingRadians;
            headingErrorRadians = normalizeRadians(
                    commandedHeadingRadians - pose.getHeading()
            );
            turn = calculateAutomaticTurn(
                    headingErrorRadians,
                    AIM_TURN_KP_PER_DEGREE
            );
            turnMode = "SHOOT TARGET";

            if (AUTO_AIM_STARTS_SHOOTER) {
                shooter.spinUp(calculateShooterRpmForDistance(
                        targetDistanceInches
                ));
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
                        headingErrorRadians,
                        CARDINAL_TURN_KP_PER_DEGREE
                );
            }
        } */

        PedroComponent.follower().setTeleOpDrive(
                drive,
                strafe,
                turn,
                false // Field centric
        );

        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
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
        if (gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed()) {
            shooter.toggle(SHOOTER_TARGET_RPM);
        }
        if (shooter.isEnabled()) {
            shooter.spinUp(SHOOTER_TARGET_RPM);
        }
        shotSequence.update(
                shotSequence.isTriggerPressed(
                        gamepad1.right_trigger,
                        gamepad2.right_trigger
                )
        );

        telemetry.addData("Pose X", "%.1f in", pose.getX());
        telemetry.addData("Pose Y", "%.1f in", pose.getY());
        telemetry.addData("Heading", "%.1f deg", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Turn mode", turnMode);
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
                "(%.1f, %.1f), distance %.1f in",
                SHOOT_TARGET_X_INCHES,
                targetY,
                targetDistanceInches
        );
        telemetry.addData("Alliance", GlobalRobotData.allianceSide);
        telemetry.addData("Pose source", startedFromAuton ? "AUTON" : "DEFAULT");
        telemetry.addData("Top beam", intake.isTopBeamBroken() ? "BROKEN" : "CLEAR");
        telemetry.addData(
                "Break beam control",
                IntakeWithSensorsSubsystem2026.USE_TOP_BREAK_BEAM ? "ENABLED" : "IGNORED"
        );
        telemetry.addData("Intake mode", intake.getIntakeMode());
        telemetry.addData("Jam reverse", intake.isJamReversing());
        telemetry.addData(
                "Transfer",
                intake.isTransferBlockedByBeam()
                        ? "STOPPED BY BEAM"
                        : (intake.isFeedingToShooter() ? "FEEDING SHOOTER" : "NORMAL")
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

    /**
     * Placeholder for the future distance-to-RPM calibration table.
     */
    private static double calculateShooterRpmForDistance(double distanceInches) {
        return SHOOTER_TARGET_RPM;
    }

    private static double calculateAutomaticTurn(
            double headingErrorRadians,
            double proportionalGainPerDegree
    ) {
        double errorDegrees = Math.toDegrees(
                normalizeRadians(headingErrorRadians)
        );
        if (Math.abs(errorDegrees)
                <= Math.max(0.0, AUTO_TURN_TOLERANCE_DEGREES)) {
            return 0.0;
        }

        double turn = errorDegrees * proportionalGainPerDegree;
        turn += Math.copySign(
                Math.max(0.0, AUTO_TURN_MIN_POWER),
                turn
        );
        return Range.clip(
                turn,
                -Math.abs(AUTO_TURN_MAX_POWER),
                Math.abs(AUTO_TURN_MAX_POWER)
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
}
