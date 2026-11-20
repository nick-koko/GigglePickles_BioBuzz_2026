package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * 2026 intake/transfer subsystem.
 *
 * <p>Four break beams represent the four final ball positions, ordered from
 * the shooter back toward the intake entrance. Position detections latch so a
 * hole in a ball cannot make the count decrease. Starting a shooting feed
 * clears all four tracked positions. Sustained top-down blocked-beam stacks
 * reconcile missed edges, without repeatedly counting a blocked sensor.</p>
 */
@Configurable
public class IntakeWithSensorsSubsystem2026 implements Subsystem {

    public static final IntakeWithSensorsSubsystem2026 INSTANCE =
            new IntakeWithSensorsSubsystem2026();

    public enum IntakeMode {
        STOPPED,
        FORWARD
    }

    public enum ManualMotorTestMode {
        NONE,
        INTAKE_FORWARD,
        TRANSFER_FORWARD,
        INTAKE_REVERSE
    }

    public static double INTAKE_FORWARD_RPM = 4000.0;
    public static double TRANSFER_FORWARD_RPM = 700.0;//main value 4000 => 1000
    public static double INTAKE_REVERSE_RPM = 3000.0;
    public static double TRANSFER_REVERSE_RPM = 700.0;
    public static double SHOOT_INTAKE_POWER = 0.5; //0.5
    public static double SHOOT_TRANSFER_POWER = 0.5; //0.5
    public static boolean ENABLE_BALL_TRACKING = true;
    // After a feed or reset, balls that never moved produce no new beam edges.
    // Once the beams hold still this long, count the stack they show.
    public static double STATIONARY_RESYNC_SECONDS = 0.25;
    public static boolean ENABLE_SUSTAINED_BEAM_RECOVERY = true;
    public static double SUSTAINED_BEAM_CONFIRM_SECONDS = 0.10;

    public static double GATE_CLOSED_POSITION = 0.15; //0.15 => 0.42
    public static double GATE_OPEN_POSITION = 0.35; //0.35 => 0.15

    private static final double ENCODER_TICKS_PER_MOTOR_REV = 28.0;
    private static final double SHAFT_REDUCTION = 3.0;

    private DcMotorEx intakeMotor;
    private DcMotorEx transferMotor;
    private DigitalChannel ballAtShooterBeam;
    private DigitalChannel ballInMiddleHighBeam;
    private DigitalChannel ballInMiddleLowBeam;
    private DigitalChannel ballAtIntakeEntryBeam;
    private Servo shooterGate;

    private IntakeMode intakeMode = IntakeMode.STOPPED;
    private boolean feedingToShooter;
    private boolean jamReversing;
    private boolean intakeCapacityOverrideHeld;
    private boolean shooterPositionOccupied;
    private boolean middleHighPositionOccupied;
    private boolean middleLowPositionOccupied;
    private boolean intakeEntryPositionOccupied;
    private boolean previousShooterBeamBroken;
    private boolean previousMiddleHighBeamBroken;
    private boolean previousMiddleLowBeamBroken;
    private boolean previousIntakeEntryBeamBroken;
    private boolean simulatedUpperStorageFull;
    private boolean previousFeedingToShooter;
    private boolean stationaryResyncPending;
    private int previousBeamMask = -1;
    private long beamsStableSinceNanos;
    private final long[] blockedSinceNanos = new long[4];
    private int confirmedBeamStackCount, ballCountRecoveryEvents;
    private int lastRecoveryFrom, lastRecoveryTo, lastRecoveryBeamMask;
    private ManualMotorTestMode manualMotorTestMode =
            ManualMotorTestMode.NONE;
    private double appliedIntakeRpm;
    private double appliedTransferRpm;
    private double appliedIntakePower;
    private double appliedTransferPower;
    private int cachedBeamMask;
    private long beamSampleNanos;
    private String appliedControlMode = "UNAVAILABLE";

    private IntakeWithSensorsSubsystem2026() {
    }

    @Override
    public void initialize() {
        validateConfiguration();
        resetRuntimeState();

        intakeMotor = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "intake_motor2");
        transferMotor = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "intake_motor");
        ballAtShooterBeam = ActiveOpMode.hardwareMap().get(
                DigitalChannel.class,
                "ball_at_shooter"
        );
        ballInMiddleHighBeam = ActiveOpMode.hardwareMap().get(
                DigitalChannel.class,
                "ball_in_middle_high"
        );
        ballInMiddleLowBeam = ActiveOpMode.hardwareMap().get(
                DigitalChannel.class,
                "ball_in_middle_low"
        );
        ballAtIntakeEntryBeam = ActiveOpMode.hardwareMap().get(
                DigitalChannel.class,
                "ball_at_intake_entry"
        );
        shooterGate = ActiveOpMode.hardwareMap().get(Servo.class, "shooter_gate");

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Preserve the corresponding 2025 motor directions.
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterGate.setDirection(Servo.Direction.REVERSE);
        ballAtShooterBeam.setMode(DigitalChannel.Mode.INPUT);
        ballInMiddleHighBeam.setMode(DigitalChannel.Mode.INPUT);
        ballInMiddleLowBeam.setMode(DigitalChannel.Mode.INPUT);
        ballAtIntakeEntryBeam.setMode(DigitalChannel.Mode.INPUT);
        synchronizePreviousBeamStates();
        shooterGate.setPosition(clippedGatePosition(GATE_CLOSED_POSITION));
        applyOutputs();
    }

    @Override
    public void periodic() {
        applyOutputs();
    }

    public void intakeForward() {
        manualMotorTestMode = ManualMotorTestMode.NONE;
        intakeMode = IntakeMode.FORWARD;
        if (simulatedUpperStorageFull) {
            shooterPositionOccupied = false;
            middleHighPositionOccupied = false;
            simulatedUpperStorageFull = false;
        }
    }

    public void stopIntake() {
        intakeCapacityOverrideHeld = false;
        manualMotorTestMode = ManualMotorTestMode.NONE;
        feedingToShooter = false;
        jamReversing = false;
        intakeMode = IntakeMode.STOPPED;
    }

    /** Bypasses only the intake capacity stop. Does not start a stopped intake or open the gate. */
    public void setIntakeCapacityOverrideHeld(boolean held) {
        intakeCapacityOverrideHeld = held;
    }

    /** Level override is separate from edge-triggered start; X has priority over every input here. */
    public void updateTeleopCollectionInputs(boolean startPressed, boolean controller2AHeld,
            boolean stopPressed, boolean reverseHeld, boolean flowerDown) {
        if (stopPressed) {
            stopIntake();
            return;
        }
        if (startPressed || flowerDown) intakeForward();
        setIntakeCapacityOverrideHeld(controller2AHeld);
        setJamReversing(reverseHeld);
    }

    /**
     * Temporarily reverses both motors while held. Releasing the button returns
     * to the previously selected STOPPED or FORWARD intake mode.
     */
    public void setJamReversing(boolean reversing) {
        manualMotorTestMode = ManualMotorTestMode.NONE;
        jamReversing = reversing;
    }

    /**
     * Opens the gate and advances the transfer toward the shooter while true.
     * This intentional feed command bypasses both capacity-based motor stops.
     */
    public void setFeedingToShooter(boolean feeding) {
        manualMotorTestMode = ManualMotorTestMode.NONE;
        if (feeding && !feedingToShooter) {
            clearTrackedBalls();
        }
        feedingToShooter = feeding;
    }

    /**
     * Test hook that reproduces both upper ball positions becoming occupied.
     * The intake motor keeps running, while the transfer stops. Calling
     * {@link #intakeForward()} clears only these simulated positions.
     */
    public void simulateUpperStorageFullForTesting() {
        if (intakeMode == IntakeMode.FORWARD
                && !jamReversing
                && !feedingToShooter) {
            shooterPositionOccupied = true;
            middleHighPositionOccupied = true;
            simulatedUpperStorageFull = true;
        }
    }

    /**
     * Diagnostic controls that run one motor at a time while keeping the
     * shooter gate closed. The selected command remains active until another
     * diagnostic command or {@link #stopAll()} is called.
     */
    public void runOnlyIntakeForward() {
        selectManualMotorTestMode(ManualMotorTestMode.INTAKE_FORWARD);
    }

    public void runOnlyTransferForward() {
        selectManualMotorTestMode(ManualMotorTestMode.TRANSFER_FORWARD);
    }

    public void runOnlyIntakeReverse() {
        selectManualMotorTestMode(ManualMotorTestMode.INTAKE_REVERSE);
    }

    public void stopAll() {
        resetRuntimeState();
        synchronizePreviousBeamStates();

        if (intakeMotor != null) {
            intakeMotor.setPower(0.0);
        }
        if (transferMotor != null) {
            transferMotor.setPower(0.0);
        }
        if (shooterGate != null) {
            shooterGate.setPosition(clippedGatePosition(GATE_CLOSED_POSITION));
        }
    }

    private void resetRuntimeState() {
        intakeCapacityOverrideHeld = false;
        clearBeamConfirmation();
        ballCountRecoveryEvents = lastRecoveryFrom = lastRecoveryTo = lastRecoveryBeamMask = 0;
        intakeMode = IntakeMode.STOPPED;
        feedingToShooter = false;
        jamReversing = false;
        shooterPositionOccupied = false;
        middleHighPositionOccupied = false;
        middleLowPositionOccupied = false;
        intakeEntryPositionOccupied = false;
        previousShooterBeamBroken = false;
        previousMiddleHighBeamBroken = false;
        previousMiddleLowBeamBroken = false;
        previousIntakeEntryBeamBroken = false;
        simulatedUpperStorageFull = false;
        previousFeedingToShooter = false;
        stationaryResyncPending = true;
        previousBeamMask = -1;
        beamsStableSinceNanos = 0L;
        manualMotorTestMode = ManualMotorTestMode.NONE;
        appliedIntakeRpm = 0.0;
        appliedTransferRpm = 0.0;
        appliedIntakePower = 0.0;
        appliedTransferPower = 0.0;

        cachedBeamMask = 0;
        beamSampleNanos = 0L;
        appliedControlMode = "UNAVAILABLE";
    }

    private void applyOutputs() {
        applyOutputs(System.nanoTime());
    }

    private void applyOutputs(long nowNanos) {
        if (intakeMotor == null || transferMotor == null
                || ballAtShooterBeam == null
                || ballInMiddleHighBeam == null
                || ballInMiddleLowBeam == null
                || ballAtIntakeEntryBeam == null
                || shooterGate == null) {
            return;
        }

        updateBallTracking(nowNanos);

        appliedIntakeRpm = 0.0;
        appliedTransferRpm = 0.0;
        appliedIntakePower = 0.0;
        appliedTransferPower = 0.0;

        double collectingIntakeRpm = shouldStopIntakeForCapacity()
                ? 0.0 : positiveRpm(INTAKE_FORWARD_RPM);
        double collectingTransferRpm = shouldStopTransferForCapacity()
                ? 0.0 : positiveRpm(TRANSFER_FORWARD_RPM);

        if (manualMotorTestMode != ManualMotorTestMode.NONE) {
            appliedControlMode = "VELOCITY";
            if (manualMotorTestMode == ManualMotorTestMode.INTAKE_FORWARD) {
                appliedIntakeRpm = positiveRpm(INTAKE_FORWARD_RPM);
            } else if (manualMotorTestMode
                    == ManualMotorTestMode.TRANSFER_FORWARD) {
                appliedTransferRpm = positiveRpm(TRANSFER_FORWARD_RPM);
            } else if (manualMotorTestMode
                    == ManualMotorTestMode.INTAKE_REVERSE) {
                appliedIntakeRpm = -positiveRpm(INTAKE_REVERSE_RPM);
            }

            setMotorVelocities(appliedIntakeRpm, appliedTransferRpm);
            shooterGate.setPosition(clippedGatePosition(GATE_CLOSED_POSITION));
            return;
        }

        if (isJamReversing()) {
            appliedControlMode = "VELOCITY";
            appliedIntakeRpm = -positiveRpm(INTAKE_REVERSE_RPM);
            appliedTransferRpm = -positiveRpm(TRANSFER_REVERSE_RPM);
            setMotorVelocities(appliedIntakeRpm, appliedTransferRpm);
        } else if (feedingToShooter) {
            appliedControlMode = "POWER";
            appliedIntakePower = clippedPower(SHOOT_INTAKE_POWER);
            appliedTransferPower = clippedPower(SHOOT_TRANSFER_POWER);
            intakeMotor.setPower(appliedIntakePower);
            transferMotor.setPower(appliedTransferPower);
        } else if (intakeMode == IntakeMode.FORWARD) {
            appliedControlMode = "VELOCITY";
            appliedIntakeRpm = collectingIntakeRpm;
            appliedTransferRpm = collectingTransferRpm;
            setMotorVelocities(appliedIntakeRpm, appliedTransferRpm);
        } else {
            appliedControlMode = "VELOCITY";
            setMotorVelocities(0.0, 0.0);
        }

        shooterGate.setPosition(clippedGatePosition(
                feedingToShooter && !isJamReversing()
                        ? GATE_OPEN_POSITION
                        : GATE_CLOSED_POSITION
        ));
    }

    private void setMotorVelocities(double intakeMotorRpm, double transferMotorRpm) {
        intakeMotor.setVelocity(rpmToTicksPerSecond(
                intakeMotorRpm,
                ENCODER_TICKS_PER_MOTOR_REV
        ));
        transferMotor.setVelocity(rpmToTicksPerSecond(
                transferMotorRpm,
                ENCODER_TICKS_PER_MOTOR_REV
        ));
    }

    private void selectManualMotorTestMode(ManualMotorTestMode mode) {
        intakeCapacityOverrideHeld = false;
        intakeMode = IntakeMode.STOPPED;
        feedingToShooter = false;
        jamReversing = false;
        simulatedUpperStorageFull = false;
        manualMotorTestMode = mode;
    }

    private void updateBallTracking(long nowNanos) {
        boolean shooterBroken = isBallAtShooterBeamBroken();
        boolean middleHighBroken = isMiddleHighBeamBroken();
        boolean middleLowBroken = isMiddleLowBeamBroken();
        boolean intakeEntryBroken = isIntakeEntryBeamBroken();
        cachedBeamMask = (shooterBroken ? 1 : 0) | (middleHighBroken ? 2 : 0)
                | (middleLowBroken ? 4 : 0) | (intakeEntryBroken ? 8 : 0);
        beamSampleNanos = nowNanos;

        if (cachedBeamMask != previousBeamMask) {
            previousBeamMask = cachedBeamMask;
            beamsStableSinceNanos = beamSampleNanos;
        }
        if (previousFeedingToShooter && !feedingToShooter) {
            // If the gate never opened, the balls are still sitting on beams
            // that were already broken when the feed cleared the count.
            stationaryResyncPending = true;
            beamsStableSinceNanos = beamSampleNanos;
            clearBeamConfirmation();
        }
        previousFeedingToShooter = feedingToShooter;

        boolean mayLatchNewBalls =
                ENABLE_BALL_TRACKING
                        && !feedingToShooter
                        && !isJamReversing()
                        && manualMotorTestMode == ManualMotorTestMode.NONE;

        if (mayLatchNewBalls) {
            confirmedBeamStackCount = 0;
            for (int bit = 0; bit < 4; bit++) {
                if ((cachedBeamMask & (1 << bit)) == 0) blockedSinceNanos[bit] = 0;
                else if (blockedSinceNanos[bit] == 0) blockedSinceNanos[bit] = nowNanos;
            }
            // Confirm each top-down position independently. Lower beam blips
            // cannot erase confirmation of upper positions. A gap stops the stack.
            for (int bit = 0; bit < 4; bit++) {
                if (blockedSinceNanos[bit] == 0
                        || (nowNanos-blockedSinceNanos[bit])/1e9 < SUSTAINED_BEAM_CONFIRM_SECONDS) break;
                confirmedBeamStackCount++;
            }
            /*
             * Each incoming ball passes the lower sensors before reaching its
             * final position. Only accept the next expected final position so
             * one ball cannot be counted several times while moving upward.
             */
            if (!shooterPositionOccupied
                    && shooterBroken
                    && !previousShooterBeamBroken) {
                shooterPositionOccupied = true;
            } else if (shooterPositionOccupied
                    && !middleHighPositionOccupied
                    && middleHighBroken
                    && !previousMiddleHighBeamBroken) {
                middleHighPositionOccupied = true;
            } else if (shooterPositionOccupied
                    && middleHighPositionOccupied
                    && !middleLowPositionOccupied
                    && middleLowBroken
                    && !previousMiddleLowBeamBroken) {
                middleLowPositionOccupied = true;
            } else if (shooterPositionOccupied
                    && middleHighPositionOccupied
                    && middleLowPositionOccupied
                    && !intakeEntryPositionOccupied
                    && intakeEntryBroken
                    && !previousIntakeEntryBeamBroken) {
                intakeEntryPositionOccupied = true;
            }

            if (ENABLE_SUSTAINED_BEAM_RECOVERY) recoverBlockedStack(confirmedBeamStackCount);

            double stableSeconds = (beamSampleNanos - beamsStableSinceNanos) / 1e9;
            if (stationaryResyncPending
                    && stableSeconds >= STATIONARY_RESYNC_SECONDS) {
                stationaryResyncPending = false;
                // Retain the old one-shot fallback when ongoing recovery is
                // disabled. It must not bypass a longer configured new delay.
                if (!ENABLE_SUSTAINED_BEAM_RECOVERY) {
                    recoverBlockedStack(stackedBeamCount(shooterBroken, middleHighBroken,
                            middleLowBroken, intakeEntryBroken));
                }
            }
        } else {
            // Feeding/reversing/test time must not pre-qualify the next collection.
            clearBeamConfirmation();
            beamsStableSinceNanos = beamSampleNanos;
        }

        previousShooterBeamBroken = shooterBroken;
        previousMiddleHighBeamBroken = middleHighBroken;
        previousMiddleLowBeamBroken = middleLowBroken;
        previousIntakeEntryBeamBroken = intakeEntryBroken;
    }

    private void synchronizePreviousBeamStates() {
        previousShooterBeamBroken = isBallAtShooterBeamBroken();
        previousMiddleHighBeamBroken = isMiddleHighBeamBroken();
        previousMiddleLowBeamBroken = isMiddleLowBeamBroken();
        previousIntakeEntryBeamBroken = isIntakeEntryBeamBroken();
    }

    /**
     * Clears the latched occupancy model when a shooting feed begins.
     */
    public void clearTrackedBalls() {
        clearBeamConfirmation();
        shooterPositionOccupied = false;
        middleHighPositionOccupied = false;
        middleLowPositionOccupied = false;
        intakeEntryPositionOccupied = false;
        simulatedUpperStorageFull = false;
        stationaryResyncPending = false;
        synchronizePreviousBeamStates();
    }

    /**
     * Restores a top-down ball stack, primarily for autonomous-to-TeleOp
     * handoff. A count of one occupies the shooter position; additional balls
     * fill toward the intake entrance.
     */
    public void setBallCount(int count) {
        clearBeamConfirmation();
        occupyTopPositions(count);
        simulatedUpperStorageFull = false;
        synchronizePreviousBeamStates();
    }

    private void occupyTopPositions(int count) {
        int clippedCount = Math.max(0, Math.min(4, count));
        shooterPositionOccupied = clippedCount >= 1;
        middleHighPositionOccupied = clippedCount >= 2;
        middleLowPositionOccupied = clippedCount >= 3;
        intakeEntryPositionOccupied = clippedCount >= 4;
    }

    /** Broken beams counted down from the shooter, stopping at the first gap. */
    private static int stackedBeamCount(
            boolean shooterBroken,
            boolean middleHighBroken,
            boolean middleLowBroken,
            boolean intakeEntryBroken
    ) {
        if (!shooterBroken) {
            return 0;
        }
        if (!middleHighBroken) {
            return 1;
        }
        if (!middleLowBroken) {
            return 2;
        }
        return intakeEntryBroken ? 4 : 3;
    }

    private boolean shouldStopTransferForCapacity() {
        return simulatedUpperStorageFull
                || (ENABLE_BALL_TRACKING && isUpperStorageFull());
    }

    private boolean shouldStopIntakeForCapacity() {
        return isIntakeCapacityReached() && !intakeCapacityOverrideHeld;
    }

    private void recoverBlockedStack(int count) {
        int previous = getBallCount();
        if (count <= previous) return;
        lastRecoveryFrom = previous;
        lastRecoveryTo = count;
        lastRecoveryBeamMask = cachedBeamMask;
        ballCountRecoveryEvents++;
        occupyTopPositions(count);
    }

    private void clearBeamConfirmation() {
        for (int bit = 0; bit < 4; bit++) blockedSinceNanos[bit] = 0;
        confirmedBeamStackCount = 0;
    }

    public static void validateConfiguration() {
        if (!Double.isFinite(STATIONARY_RESYNC_SECONDS) || STATIONARY_RESYNC_SECONDS < 0
                || !Double.isFinite(SUSTAINED_BEAM_CONFIRM_SECONDS) || SUSTAINED_BEAM_CONFIRM_SECONDS < 0)
            throw new IllegalArgumentException("Intake beam confirmation times must be finite and nonnegative");
    }

    private static double positiveRpm(double rpm) {
        return Math.max(0.0, rpm);
    }

    private static double rpmToTicksPerSecond(double rpm, double ticksPerRevolution) {
        return rpm * ticksPerRevolution / 60.0;
    }

    private static double ticksPerSecondToRpm(double ticksPerSecond, double ticksPerRevolution) {
        return ticksPerSecond * 60.0 / ticksPerRevolution;
    }

    private static double clippedGatePosition(double position) {
        return Range.clip(position, 0.0, 1.0);
    }

    private static double clippedPower(double power) {
        return Range.clip(power, 0.0, 1.0);
    }

    public IntakeMode getIntakeMode() {
        return intakeMode;
    }

    public boolean isBallAtShooterBeamBroken() {
        return ballAtShooterBeam != null && !ballAtShooterBeam.getState();
    }

    public boolean isMiddleHighBeamBroken() {
        return ballInMiddleHighBeam != null
                && !ballInMiddleHighBeam.getState();
    }

    public boolean isMiddleLowBeamBroken() {
        return ballInMiddleLowBeam != null
                && !ballInMiddleLowBeam.getState();
    }

    public boolean isIntakeEntryBeamBroken() {
        return ballAtIntakeEntryBeam != null
                && !ballAtIntakeEntryBeam.getState();
    }

    public boolean isShooterPositionOccupied() {
        return shooterPositionOccupied;
    }

    public boolean isMiddleHighPositionOccupied() {
        return middleHighPositionOccupied;
    }

    public boolean isMiddleLowPositionOccupied() {
        return middleLowPositionOccupied;
    }

    public boolean isIntakeEntryPositionOccupied() {
        return intakeEntryPositionOccupied;
    }

    public int getBallCount() {
        int count = 0;
        if (shooterPositionOccupied) {
            count++;
        }
        if (middleHighPositionOccupied) {
            count++;
        }
        if (middleLowPositionOccupied) {
            count++;
        }
        if (intakeEntryPositionOccupied) {
            count++;
        }
        return count;
    }

    public boolean isUpperStorageFull() {
        return shooterPositionOccupied && middleHighPositionOccupied;
    }

    public boolean isTransferStoppedForCapacity() {
        return shouldStopTransferForCapacity();
    }

    public boolean isIntakeStoppedForCapacity() {
        return shouldStopIntakeForCapacity();
    }

    public boolean isIntakeCapacityReached() { return ENABLE_BALL_TRACKING && getBallCount() >= 4; }
    public boolean isIntakeCapacityOverrideHeld() { return intakeCapacityOverrideHeld; }
    public boolean isIntakeCapacityOverrideActive() {
        return intakeCapacityOverrideHeld && intakeMode == IntakeMode.FORWARD
                && !feedingToShooter && !jamReversing && manualMotorTestMode == ManualMotorTestMode.NONE;
    }
    public int getConfirmedBeamStackCount() { return confirmedBeamStackCount; }
    public int getBallCountRecoveryEvents() { return ballCountRecoveryEvents; }
    public int getLastRecoveryFrom() { return lastRecoveryFrom; }
    public int getLastRecoveryTo() { return lastRecoveryTo; }
    public int getLastRecoveryBeamMask() { return lastRecoveryBeamMask; }
    /** Uses the same cached sample time as the mask, without reading hardware again. */
    public double getBeamBlockedSeconds(int bit) {
        return blockedSinceNanos[bit] == 0 ? 0.0 : (beamSampleNanos-blockedSinceNanos[bit])/1e9;
    }

    public boolean isUpperStorageFullSimulated() {
        return simulatedUpperStorageFull;
    }

    public boolean isFeedingToShooter() {
        return feedingToShooter;
    }

    public boolean isJamReversing() {
        return jamReversing;
    }

    public ManualMotorTestMode getManualMotorTestMode() {
        return manualMotorTestMode;
    }

    public double getAppliedIntakeRpm() {
        return appliedIntakeRpm;
    }

    public double getAppliedTransferRpm() {
        return appliedTransferRpm;
    }

    public double getAppliedIntakePower() {
        return appliedIntakePower;
    }

    public double getAppliedTransferPower() {
        return appliedTransferPower;
    }

    public double getMeasuredIntakeMotorRpm() {
        if (intakeMotor == null) {
            return 0.0;
        }
        return ticksPerSecondToRpm(
                intakeMotor.getVelocity(),
                ENCODER_TICKS_PER_MOTOR_REV
        );
    }

    public double getMeasuredTransferMotorRpm() {
        if (transferMotor == null) {
            return 0.0;
        }
        return ticksPerSecondToRpm(
                transferMotor.getVelocity(),
                ENCODER_TICKS_PER_MOTOR_REV
        );
    }

    public double getMeasuredIntakeShaftRpm() {
        return getMeasuredIntakeMotorRpm() / SHAFT_REDUCTION;
    }

    public double getMeasuredTransferShaftRpm() {
        return getMeasuredTransferMotorRpm() / SHAFT_REDUCTION;
    }

    public double getGatePosition() {
        return shooterGate == null
                ? clippedGatePosition(GATE_CLOSED_POSITION)
                : shooterGate.getPosition();
    }

    // Reuse the normal control loop's sensor sample; logging adds no digital reads.
    public int getCachedBeamMask() { return cachedBeamMask; }
    public long getBeamSampleNanos() { return beamSampleNanos; }
    public String getAppliedControlMode() { return appliedControlMode; }
    public double getShaftReduction() { return SHAFT_REDUCTION; }
    public double getIntakeControllerPowerReadback() {
        return intakeMotor == null ? Double.NaN : intakeMotor.getPower();
    }
    public double getTransferControllerPowerReadback() {
        return transferMotor == null ? Double.NaN : transferMotor.getPower();
    }
}
