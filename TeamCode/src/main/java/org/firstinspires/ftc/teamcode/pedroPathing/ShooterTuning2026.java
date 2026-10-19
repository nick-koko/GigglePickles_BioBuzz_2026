package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.IntakeWithSensorsSubsystem2026;
import org.firstinspires.ftc.teamcode.subsystems.ShotSequenceController2026;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem2026;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/**
 * Panels-enabled tuning OpMode for the 2026 single-motor flywheel.
 *
 * <p>Edit ShooterSubsystem2026's configurable PIDF and compensation values in
 * Panels. This OpMode supplies the target and publishes graph-friendly control
 * loop diagnostics without initializing the drivetrain or intake.</p>
 */
@Configurable
@TeleOp(name = "Shooter Tuning 2026", group = "Test")
public class ShooterTuning2026 extends NextFTCOpMode {

    public static double TARGET_FLYWHEEL_RPM = 3100.0;
    public static double RPM_STEP = 25.0;
    public static double LARGE_RPM_STEP = 100.0;
    public static double MIN_TARGET_RPM = 0.0;
    public static double MAX_TARGET_RPM = 6000.0;
    public static double AT_SPEED_TOLERANCE_RPM = 75.0;
    public static double RPM_DROP_DETECTION_THRESHOLD = 200.0;
    public static double RECOVERY_TIMEOUT_SECONDS = 5.0;

    private TelemetryManager panelsTelemetry;
    private final ShotSequenceController2026 shotSequence =
            new ShotSequenceController2026();
    private int completedShotSequenceCount;
    private boolean rpmDropThresholdDetected;
    private double rpmDropDetectedAtSeconds = -1.0;
    private boolean previousFeeding;
    private double feedBaselineRpm;
    private double feedMinimumRpm;
    private double feedMaximumDropRpm;

    // Persistent summary of the most recently completed automatic feed.
    private boolean shotSummaryAvailable;
    private double summaryTargetRpm;
    private double summaryBaselineRpm;
    private double summaryMinimumRpm;
    private double summaryMaximumDropRpm;
    private double summaryDropDetectedAtSeconds = -1.0;
    private double summaryStartingPower;
    private double summaryMaximumPower;
    private double summaryStartingBatteryVoltage = Double.NaN;
    private double summaryMinimumBatteryVoltage = Double.NaN;
    private double summaryMaximumCurrentAmps;
    private boolean summaryBoostEnabled;
    private double summaryFeedDurationSeconds;
    private double summaryDropThresholdRpm;
    private double summaryBoostDelaySeconds;
    private double summaryBoostDurationSeconds;
    private double summaryBoostMinimumPower;
    private double summaryKP;
    private double summaryKI;
    private double summaryKD;
    private double summaryKS;
    private double summaryKV;
    private double summaryKA;
    private boolean recoveryMeasurementActive;
    private long recoveryStartNanos;
    private double summaryRecoverySeconds = -1.0;
    private boolean summaryRecoveryTimedOut;

    public ShooterTuning2026() {
        addComponents(
                new SubsystemComponent(
                        ShooterSubsystem2026.INSTANCE,
                        IntakeWithSensorsSubsystem2026.INSTANCE
                ),
                BulkReadComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        ShooterSubsystem2026.INSTANCE.stop();
        IntakeWithSensorsSubsystem2026.INSTANCE.stopAll();
        resetAutomaticShotMeasurement();
        resetFeedDropMeasurement();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void onWaitForStart() {
        panelsTelemetry.addData("Status", "READY - SHOOTER STOPPED");
        panelsTelemetry.addData(
                "Shooter controls",
                "GP1/GP2 RB toggle; X leaves flywheel running"
        );
        panelsTelemetry.addData(
                "Intake controls",
                "GP1/GP2 A intake, hold B reverse, press RT for timed feed"
        );
        panelsTelemetry.addData("Automatic feed seconds", automaticFeedSeconds());
        panelsTelemetry.addData(
                "Timed boost enabled",
                ShotSequenceController2026.ENABLE_TIMED_BOOST
        );
        panelsTelemetry.addData("Boost delay seconds", boostDelaySeconds());
        panelsTelemetry.addData("Boost duration seconds", boostDurationSeconds());
        panelsTelemetry.addData("Boost minimum power", clippedBoostPower());
        panelsTelemetry.addData("RPM controls", "D-pad up/down fine, right/left coarse");
        panelsTelemetry.addData("Target flywheel RPM", clippedTargetRpm());
        panelsTelemetry.addData("Motor RPM per flywheel RPM",
                ShooterSubsystem2026.MOTOR_RPM_PER_FLYWHEEL_RPM);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void onStartButtonPressed() {
        ShooterSubsystem2026.INSTANCE.stop();
        IntakeWithSensorsSubsystem2026.INSTANCE.stopAll();
        resetAutomaticShotMeasurement();
        resetFeedDropMeasurement();
    }

    @Override
    public void onUpdate() {
        ShooterSubsystem2026 shooter = ShooterSubsystem2026.INSTANCE;

        if (gamepad1.dpadUpWasPressed()) {
            TARGET_FLYWHEEL_RPM += Math.abs(RPM_STEP);
        }
        if (gamepad1.dpadDownWasPressed()) {
            TARGET_FLYWHEEL_RPM -= Math.abs(RPM_STEP);
        }
        if (gamepad1.dpadRightWasPressed()) {
            TARGET_FLYWHEEL_RPM += Math.abs(LARGE_RPM_STEP);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            TARGET_FLYWHEEL_RPM -= Math.abs(LARGE_RPM_STEP);
        }
        TARGET_FLYWHEEL_RPM = clippedTargetRpm();

        if (gamepad1.rightBumperWasPressed()) {
            shooter.toggle(TARGET_FLYWHEEL_RPM);
        } else if (gamepad2.rightBumperWasPressed()) {
            shooter.toggle(TARGET_FLYWHEEL_RPM);
        }

        IntakeWithSensorsSubsystem2026 intake =
                IntakeWithSensorsSubsystem2026.INSTANCE;

        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            intake.intakeForward();
        }

        boolean stopWasPressed =
                gamepad1.xWasPressed() || gamepad2.xWasPressed();
        if (stopWasPressed) {
            cancelAutomaticFeed();
            intake.stopIntake();
        }

        boolean shootButton = shotSequence.isTriggerPressed(
                gamepad1.right_trigger,
                gamepad2.right_trigger
        );
        boolean feeding = updateAutomaticFeed(
                shooter,
                shootButton,
                stopWasPressed
        );
        if (!stopWasPressed) {
            intake.setJamReversing(gamepad1.b || gamepad2.b);
        }

        // Apply gamepad or live Panels target changes without restarting the
        // controller or clearing its integral state.
        if (shooter.isEnabled()) {
            shooter.spinUp(TARGET_FLYWHEEL_RPM);
        }

        updateFeedDropMeasurement(shooter, feeding);
        updateRecoveryMeasurement(shooter);
        addTuningTelemetry(shooter, intake, feeding);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void onStop() {
        shotSequence.cancel();
        IntakeWithSensorsSubsystem2026.INSTANCE.stopAll();
        ShooterSubsystem2026.INSTANCE.stop();
        super.onStop();
    }

    private void addShotSummaryTelemetry() {
        panelsTelemetry.addData(
                "SHOT SUMMARY available",
                shotSummaryAvailable
        );
        if (!shotSummaryAvailable) {
            return;
        }

        panelsTelemetry.addData(
                "SHOT SUMMARY sequence",
                completedShotSequenceCount
        );
        panelsTelemetry.addData("SHOT SUMMARY target RPM", summaryTargetRpm);
        panelsTelemetry.addData("SHOT SUMMARY baseline RPM", summaryBaselineRpm);
        panelsTelemetry.addData("SHOT SUMMARY minimum RPM", summaryMinimumRpm);
        panelsTelemetry.addData(
                "SHOT SUMMARY maximum RPM drop",
                summaryMaximumDropRpm
        );
        panelsTelemetry.addData(
                "SHOT SUMMARY seconds to threshold drop",
                summaryDropDetectedAtSeconds
        );
        panelsTelemetry.addData(
                "SHOT SUMMARY recovery seconds after feed",
                summaryRecoverySeconds
        );
        panelsTelemetry.addData(
                "SHOT SUMMARY recovery timed out",
                summaryRecoveryTimedOut
        );
        panelsTelemetry.addData(
                "SHOT SUMMARY starting / maximum power",
                String.format("%.3f / %.3f",
                        summaryStartingPower,
                        summaryMaximumPower)
        );
        panelsTelemetry.addData(
                "SHOT SUMMARY starting / minimum battery V",
                String.format("%.2f / %.2f",
                        summaryStartingBatteryVoltage,
                        summaryMinimumBatteryVoltage)
        );
        panelsTelemetry.addData(
                "SHOT SUMMARY maximum current A",
                summaryMaximumCurrentAmps
        );
        panelsTelemetry.addData(
                "SHOT SUMMARY boost enabled",
                summaryBoostEnabled
        );
        panelsTelemetry.addData(
                "SHOT SUMMARY feed duration sec",
                summaryFeedDurationSeconds
        );
        panelsTelemetry.addData(
                "SHOT SUMMARY drop threshold RPM",
                summaryDropThresholdRpm
        );
        panelsTelemetry.addData(
                "SHOT SUMMARY boost delay / duration sec",
                String.format("%.3f / %.3f",
                        summaryBoostDelaySeconds,
                        summaryBoostDurationSeconds)
        );
        panelsTelemetry.addData(
                "SHOT SUMMARY boost minimum power",
                summaryBoostMinimumPower
        );
        panelsTelemetry.addData(
                "SHOT SUMMARY PID",
                String.format(
                        "P %.6f  I %.6f  D %.6f",
                        summaryKP,
                        summaryKI,
                        summaryKD
                )
        );
        panelsTelemetry.addData(
                "SHOT SUMMARY feedforward",
                String.format(
                        "S %.6f  V %.6f  A %.6f",
                        summaryKS,
                        summaryKV,
                        summaryKA
                )
        );
    }

    private void addTuningTelemetry(
            ShooterSubsystem2026 shooter,
            IntakeWithSensorsSubsystem2026 intake,
            boolean feeding
    ) {
        addShotSummaryTelemetry();

        panelsTelemetry.addData("Shooter enabled", shooter.isEnabled());
        panelsTelemetry.addData("At speed",
                shooter.isAtSpeed(AT_SPEED_TOLERANCE_RPM));

        panelsTelemetry.addData("Target flywheel RPM",
                shooter.getTargetFlywheelRpm());
        panelsTelemetry.addData("Measured flywheel RPM",
                shooter.getMeasuredFlywheelRpm());
        panelsTelemetry.addData("RPM error", shooter.getErrorRpm());
        panelsTelemetry.addData("Encoder-delta flywheel RPM",
                shooter.getEncoderDeltaFlywheelRpm());

        panelsTelemetry.addData("Target motor RPM", shooter.getTargetMotorRpm());
        panelsTelemetry.addData("Measured motor RPM", shooter.getMeasuredMotorRpm());
        panelsTelemetry.addData("Encoder-delta motor RPM",
                shooter.getEncoderDeltaMotorRpm());

        panelsTelemetry.addData("P term", shooter.getPTerm());
        panelsTelemetry.addData("I term", shooter.getITerm());
        panelsTelemetry.addData("D term", shooter.getDTerm());
        panelsTelemetry.addData("Static FF", shooter.getStaticFeedforward());
        panelsTelemetry.addData("Velocity FF", shooter.getVelocityFeedforward());
        panelsTelemetry.addData("Acceleration FF",
                shooter.getAccelerationFeedforward());
        panelsTelemetry.addData("Total FF", shooter.getFeedforward());

        panelsTelemetry.addData("Requested power", shooter.getOutputBeforeSlew());
        panelsTelemetry.addData("Power after slew", shooter.getOutputAfterSlew());
        panelsTelemetry.addData("Applied power", shooter.getAppliedPower());
        panelsTelemetry.addData("Slew limited", shooter.wasSlewLimited());
        panelsTelemetry.addData("Output saturated", shooter.isCommandSaturated());

        panelsTelemetry.addData("Battery voltage raw",
                shooter.getBatteryVoltageRaw());
        panelsTelemetry.addData("Battery voltage filtered",
                shooter.getBatteryVoltageFiltered());
        panelsTelemetry.addData("Voltage compensation gain",
                shooter.getVoltageCompGain());
        panelsTelemetry.addData("Motor current amps",
                shooter.getMotorCurrentAmps());
        panelsTelemetry.addData("Controller loop dt seconds",
                shooter.getLoopDtSeconds());

        panelsTelemetry.addData("kP", ShooterSubsystem2026.kP);
        panelsTelemetry.addData("kI", ShooterSubsystem2026.kI);
        panelsTelemetry.addData("kD", ShooterSubsystem2026.kD);
        panelsTelemetry.addData("kS", ShooterSubsystem2026.kS);
        panelsTelemetry.addData("kV", ShooterSubsystem2026.kV);
        panelsTelemetry.addData("kA", ShooterSubsystem2026.kA);

        panelsTelemetry.addData("Feed active", feeding);
        panelsTelemetry.addData(
                "Automatic feed active",
                shotSequence.isActive()
        );
        panelsTelemetry.addData(
                "Seconds since shoot pressed",
                shotSequence.getElapsedSeconds()
        );
        panelsTelemetry.addData(
                "RPM drop detection threshold",
                Math.max(0.0, RPM_DROP_DETECTION_THRESHOLD)
        );
        panelsTelemetry.addData(
                "RPM drop threshold detected",
                rpmDropThresholdDetected
        );
        panelsTelemetry.addData(
                "Seconds to RPM drop threshold",
                rpmDropDetectedAtSeconds
        );
        panelsTelemetry.addData(
                "Current target RPM drop",
                Math.max(
                        0.0,
                        shooter.getTargetFlywheelRpm()
                                - shooter.getMeasuredFlywheelRpm()
                )
        );
        panelsTelemetry.addData(
                "Timed boost active",
                shooter.isMinimumPowerOverrideActive()
        );
        panelsTelemetry.addData(
                "Timed boost minimum power",
                shooter.getMinimumPowerOverride()
        );
        panelsTelemetry.addData(
                "Timed boost bypasses slew",
                shooter.isPowerOverrideBypassingSlew()
        );
        panelsTelemetry.addData("Feed baseline RPM", feedBaselineRpm);
        panelsTelemetry.addData("Feed minimum RPM", feedMinimumRpm);
        panelsTelemetry.addData("Feed maximum RPM drop", feedMaximumDropRpm);
        panelsTelemetry.addData("Intake mode", intake.getIntakeMode());
        panelsTelemetry.addData("Jam reverse", intake.isJamReversing());
        panelsTelemetry.addData("Top beam broken", intake.isTopBeamBroken());
        panelsTelemetry.addData(
                "Transfer stopped by beam",
                intake.isTransferBlockedByBeam()
        );
        panelsTelemetry.addData("Intake applied power",
                intake.getAppliedIntakePower());
        panelsTelemetry.addData("Transfer applied power",
                intake.getAppliedTransferPower());
        panelsTelemetry.addData("Shooter gate position",
                intake.getGatePosition());
    }

    private boolean updateAutomaticFeed(
            ShooterSubsystem2026 shooter,
            boolean shootButton,
            boolean stopWasPressed
    ) {
        if (stopWasPressed) {
            return false;
        }

        shotSequence.update(shootButton);

        if (shotSequence.startedThisUpdate()) {
            rpmDropThresholdDetected = false;
            rpmDropDetectedAtSeconds = -1.0;
            resetFeedDropMeasurement();
            beginShotSummary(shooter);
        }

        if (shotSequence.isActive()
                || shotSequence.completedThisUpdate()) {
            double rpmDrop = shooter.getTargetFlywheelRpm()
                    - shooter.getMeasuredFlywheelRpm();
            if (!rpmDropThresholdDetected
                    && rpmDrop >= Math.max(
                    0.0,
                    RPM_DROP_DETECTION_THRESHOLD
            )) {
                rpmDropThresholdDetected = true;
                rpmDropDetectedAtSeconds =
                        shotSequence.getElapsedSeconds();
            }
            captureShotExtremes(shooter);
        }

        if (shotSequence.completedThisUpdate()) {
            completeShotSummary(System.nanoTime());
        }

        return shotSequence.isActive();
    }

    private void beginShotSummary(ShooterSubsystem2026 shooter) {
        shotSummaryAvailable = false;
        recoveryMeasurementActive = false;
        summaryRecoverySeconds = -1.0;
        summaryRecoveryTimedOut = false;

        summaryTargetRpm = shooter.getTargetFlywheelRpm();
        summaryBaselineRpm = shooter.getMeasuredFlywheelRpm();
        summaryMinimumRpm = summaryBaselineRpm;
        summaryMaximumDropRpm = 0.0;
        summaryDropDetectedAtSeconds = -1.0;
        summaryStartingPower = shooter.getAppliedPower();
        summaryMaximumPower = summaryStartingPower;
        summaryStartingBatteryVoltage = shooter.getBatteryVoltageFiltered();
        summaryMinimumBatteryVoltage = summaryStartingBatteryVoltage;
        summaryMaximumCurrentAmps = shooter.getMotorCurrentAmps();
        summaryBoostEnabled = shotSequence.isActiveBoostEnabled();
        summaryFeedDurationSeconds = shotSequence.getActiveFeedSeconds();
        summaryDropThresholdRpm = Math.max(
                0.0,
                RPM_DROP_DETECTION_THRESHOLD
        );
        summaryBoostDelaySeconds =
                shotSequence.getActiveBoostDelaySeconds();
        summaryBoostDurationSeconds =
                shotSequence.getActiveBoostDurationSeconds();
        summaryBoostMinimumPower =
                shotSequence.getActiveBoostMinimumPower();
        summaryKP = ShooterSubsystem2026.kP;
        summaryKI = ShooterSubsystem2026.kI;
        summaryKD = ShooterSubsystem2026.kD;
        summaryKS = ShooterSubsystem2026.kS;
        summaryKV = ShooterSubsystem2026.kV;
        summaryKA = ShooterSubsystem2026.kA;
    }

    private void captureShotExtremes(ShooterSubsystem2026 shooter) {
        double measuredRpm = shooter.getMeasuredFlywheelRpm();
        summaryMinimumRpm = Math.min(summaryMinimumRpm, measuredRpm);
        summaryMaximumDropRpm = Math.max(
                summaryMaximumDropRpm,
                summaryBaselineRpm - measuredRpm
        );
        summaryMaximumPower = Math.max(
                summaryMaximumPower,
                shooter.getAppliedPower()
        );
        summaryMaximumCurrentAmps = Math.max(
                summaryMaximumCurrentAmps,
                shooter.getMotorCurrentAmps()
        );

        double batteryVoltage = shooter.getBatteryVoltageFiltered();
        if (Double.isFinite(batteryVoltage)) {
            if (!Double.isFinite(summaryMinimumBatteryVoltage)) {
                summaryMinimumBatteryVoltage = batteryVoltage;
            } else {
                summaryMinimumBatteryVoltage = Math.min(
                        summaryMinimumBatteryVoltage,
                        batteryVoltage
                );
            }
        }
    }

    private void completeShotSummary(long nowNanos) {
        completedShotSequenceCount++;
        shotSummaryAvailable = true;
        summaryDropDetectedAtSeconds = rpmDropDetectedAtSeconds;
        summaryRecoverySeconds = -1.0;
        summaryRecoveryTimedOut = false;
        recoveryMeasurementActive = true;
        recoveryStartNanos = nowNanos;
    }

    private void updateRecoveryMeasurement(ShooterSubsystem2026 shooter) {
        if (!recoveryMeasurementActive) {
            return;
        }

        double elapsedSeconds =
                (System.nanoTime() - recoveryStartNanos) / 1e9;
        if (shooter.isAtSpeed(AT_SPEED_TOLERANCE_RPM)) {
            summaryRecoverySeconds = elapsedSeconds;
            recoveryMeasurementActive = false;
            return;
        }

        double timeoutSeconds = Math.max(0.0, RECOVERY_TIMEOUT_SECONDS);
        if (elapsedSeconds >= timeoutSeconds) {
            summaryRecoverySeconds = timeoutSeconds;
            summaryRecoveryTimedOut = true;
            recoveryMeasurementActive = false;
        }
    }

    private void cancelAutomaticFeed() {
        shotSequence.cancel();
    }

    private void resetAutomaticShotMeasurement() {
        shotSequence.reset();
        rpmDropThresholdDetected = false;
        rpmDropDetectedAtSeconds = -1.0;
        completedShotSequenceCount = 0;
        shotSummaryAvailable = false;
        recoveryMeasurementActive = false;
        summaryRecoverySeconds = -1.0;
        summaryRecoveryTimedOut = false;
    }

    private void updateFeedDropMeasurement(
            ShooterSubsystem2026 shooter,
            boolean feeding
    ) {
        double measuredRpm = shooter.getMeasuredFlywheelRpm();

        if (feeding && !previousFeeding) {
            feedBaselineRpm = measuredRpm;
            feedMinimumRpm = measuredRpm;
            feedMaximumDropRpm = 0.0;
        } else if (feeding) {
            feedMinimumRpm = Math.min(feedMinimumRpm, measuredRpm);
            feedMaximumDropRpm = Math.max(
                    feedMaximumDropRpm,
                    feedBaselineRpm - measuredRpm
            );
        }

        previousFeeding = feeding;
    }

    private void resetFeedDropMeasurement() {
        previousFeeding = false;
        feedBaselineRpm = 0.0;
        feedMinimumRpm = 0.0;
        feedMaximumDropRpm = 0.0;
    }

    private static double clippedTargetRpm() {
        double minimum = Math.max(0.0, Math.min(MIN_TARGET_RPM, MAX_TARGET_RPM));
        double maximum = Math.max(minimum, Math.max(MIN_TARGET_RPM, MAX_TARGET_RPM));
        return Range.clip(TARGET_FLYWHEEL_RPM, minimum, maximum);
    }

    private static double automaticFeedSeconds() {
        return Math.max(0.0, ShotSequenceController2026.FEED_SECONDS);
    }

    private static double boostDelaySeconds() {
        return Math.max(
                0.0,
                ShotSequenceController2026.BOOST_DELAY_SECONDS
        );
    }

    private static double boostDurationSeconds() {
        return Math.max(
                0.0,
                ShotSequenceController2026.BOOST_DURATION_SECONDS
        );
    }

    private static double clippedBoostPower() {
        return Range.clip(
                ShotSequenceController2026.BOOST_MINIMUM_POWER,
                0.0,
                1.0
        );
    }
}
