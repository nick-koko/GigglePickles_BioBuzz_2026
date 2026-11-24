package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.ShooterPowerProfile2026;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * Single-motor flywheel shooter with a software PIDF controller.
 *
 * <p>The public target is always flywheel RPM. The motor/flywheel ratio is
 * applied internally so callers do not need to change if the gearing changes.
 * Ball detection, boost profiles, and hood control intentionally live outside
 * this subsystem.</p>
 */
@Configurable
public class ShooterSubsystem2026 implements Subsystem {

    public static final ShooterSubsystem2026 INSTANCE = new ShooterSubsystem2026();

    // PID + feedforward values copied from the proven single-output portion of
    // the previous shooter. Tune these for the new flywheel.
    public static double kP = 0.0009;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kS =0.09;
    public static double kV =0.000134;
    public static double kA = 0.0;

    public static double SLEW_PER_SECOND = 1.5;
    // Nov 23 tested profile: fast cold starts, then 1.5/s for running adjustments.
    public static boolean ENABLE_ASYMMETRIC_SLEW = true;
    public static double RUNNING_SLEW_UP_PER_SECOND = 1.5;
    public static double RUNNING_SLEW_DOWN_PER_SECOND = 1.5;
    public static boolean ENABLE_STARTUP_PROFILE = true;
    public static double STARTUP_SLEW_UP_PER_SECOND = 6.0;
    public static double STARTUP_SLEW_DOWN_PER_SECOND = 6.0;
    public static double STARTUP_ENTRY_MAX_RPM = 200.0;
    public static double STARTUP_EXIT_TOLERANCE_RPM = 50.0;
    public static double STARTUP_EXIT_MAX_SLOPE_RPM_PER_SECOND = 200.0;
    public static double STARTUP_EXIT_STABLE_SECONDS = 0.15;
    public static double STARTUP_MAX_SECONDS = 2.0;
    public static double I_ZONE_RPM = 500.0;
    public static double INTEGRAL_MIN = -1.0;
    public static double INTEGRAL_MAX = 1.0;

    public static double ENCODER_TICKS_PER_MOTOR_REV = 28.0;

    /**
     * Motor revolutions required for one flywheel revolution.
     * Current direct drive is 1:1. For a motor spinning 23/17 times faster than
     * the flywheel, set this to 23.0 / 17.0.
     */
    public static double MOTOR_RPM_PER_FLYWHEEL_RPM = 1.0;

    public static boolean ENABLE_VOLTAGE_COMPENSATION = true;
    public static double VOLTAGE_COMP_NOMINAL_V = 12.5;
    public static double VOLTAGE_COMP_FILTER_TAU_SEC = 0.5;
    public static double VOLTAGE_COMP_MIN_GAIN = 0.90;
    public static double VOLTAGE_COMP_MAX_GAIN = 1.10;
    public static double VOLTAGE_COMP_MIN_VALID_V = 7.0;

    private DcMotorEx shooterMotor;

    private double targetFlywheelRpm;
    private boolean enabled;
    private boolean controllerJustEnabled;
    private double minimumPowerOverride = Double.NaN;
    private boolean powerOverrideBypassesSlew;
    private boolean boostVoltageScaling, shotFeeding;
    private double boostNominalVoltage = 12.5, boostOverspeedTaperRpm;
    private final ShooterPowerProfile2026 powerProfile = new ShooterPowerProfile2026();
    private double lastSlewUp, lastSlewDown, lastOverrideFloor;
    private double lastBoostVoltageGain = 1.0, lastBoostScaledFloor;
    private double lastBoostOverspeedMultiplier = 1.0, lastBoostEffectiveFloor;
    private boolean lastBoostBypassedSlew;

    // Controller state and cached diagnostic/logging values.
    private long lastTimestampNanos;
    private int lastEncoderTicks;
    private double lastTargetFlywheelRpm;
    private double lastMeasuredMotorRpm;
    private double lastMeasuredFlywheelRpm;
    private double lastEncoderDeltaMotorRpm;
    private double lastEncoderDeltaFlywheelRpm;
    private double lastLoopDtSeconds;
    private double lastErrorRpm;
    private double integral;
    private double lastDerivativeRpmPerSecond;
    private double lastTargetAccelerationRpmPerSecond;
    private double lastPTerm;
    private double lastITerm;
    private double lastDTerm;
    private double lastStaticFeedforward;
    private double lastVelocityFeedforward;
    private double lastAccelerationFeedforward;
    private double lastFeedforward;
    private double lastOutputBeforeSlew;
    private double lastOutputAfterSlew;
    private double lastOutputBeforeVoltageComp;
    private double lastControllerOutputBeforeBoost;
    private double lastOutputWithoutBoostThisLoop;
    private double lastAppliedPower;
    private boolean lastSlewLimited;
    private boolean lastCommandSaturated;
    private boolean lastPowerOverrideActive;
    private double lastMotorCurrentAmps;
    private double lastBatteryVoltageRaw = Double.NaN;
    private double lastBatteryVoltageFiltered = Double.NaN;
    private double lastVoltageCompGain = 1.0;

    private ShooterSubsystem2026() {
    }

    @Override
    public void initialize() {
        validateConfiguration();
        resetRuntimeState();

        shooterMotor = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "shooter_motor1");

        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor.setPower(0.0);
    }

    @Override
    public void periodic() {
        update();
    }

    public void update() {
        if (shooterMotor == null) {
            return;
        }

        long nowNanos = System.nanoTime();
        double measuredMotorRpm = readMeasuredMotorRpm();
        double measuredFlywheelRpm = motorToFlywheelRpm(measuredMotorRpm);
        int encoderTicks = shooterMotor.getCurrentPosition();

        lastMotorCurrentAmps = shooterMotor.getCurrent(CurrentUnit.AMPS);

        if (lastTimestampNanos == 0L) {
            lastTimestampNanos = nowNanos;
            lastEncoderTicks = encoderTicks;
            lastMeasuredMotorRpm = measuredMotorRpm;
            lastMeasuredFlywheelRpm = measuredFlywheelRpm;
            lastTargetFlywheelRpm = targetFlywheelRpm;
            updateBatteryVoltageEstimate(0.0);
            if (!enabled) {
                applyPower(0.0);
            }
            return;
        }

        double dt = (nowNanos - lastTimestampNanos) / 1e9;
        if (!Double.isFinite(dt) || dt <= 0.0) {
            return;
        }

        lastLoopDtSeconds = dt;
        lastEncoderDeltaMotorRpm = Math.abs(
                ticksPerSecondToMotorRpm((encoderTicks - lastEncoderTicks) / dt)
        );
        lastEncoderDeltaFlywheelRpm =
                motorToFlywheelRpm(lastEncoderDeltaMotorRpm);
        lastEncoderTicks = encoderTicks;

        lastMeasuredMotorRpm = measuredMotorRpm;
        lastMeasuredFlywheelRpm = measuredFlywheelRpm;
        updateBatteryVoltageEstimate(dt);

        if (!enabled || targetFlywheelRpm <= 0.0) {
            powerProfile.reset();
            applyPower(0.0);
            clearControllerTerms();
            lastTimestampNanos = nowNanos;
            lastTargetFlywheelRpm = 0.0;
            return;
        }

        updateController(nowNanos, measuredFlywheelRpm, dt);
    }

    /** Uses this loop's cached samples only; separated for deterministic desktop checks. */
    private void updateController(long nowNanos, double measuredFlywheelRpm, double dt) {
        double error = targetFlywheelRpm - measuredFlywheelRpm;

        if (controllerJustEnabled) {
            powerProfile.begin(nowNanos, measuredFlywheelRpm, ENABLE_STARTUP_PROFILE,
                    STARTUP_ENTRY_MAX_RPM, shotFeeding);
            lastErrorRpm = error;
            lastTargetFlywheelRpm = targetFlywheelRpm;
            controllerJustEnabled = false;
        }
        powerProfile.update(nowNanos, measuredFlywheelRpm, targetFlywheelRpm, dt,
                ENABLE_STARTUP_PROFILE, shotFeeding, STARTUP_MAX_SECONDS,
                STARTUP_EXIT_TOLERANCE_RPM, STARTUP_EXIT_MAX_SLOPE_RPM_PER_SECOND,
                STARTUP_EXIT_STABLE_SECONDS);
        boolean startup = powerProfile.getState() == ShooterPowerProfile2026.State.STARTUP;
        lastSlewUp = startup ? STARTUP_SLEW_UP_PER_SECOND
                : ENABLE_ASYMMETRIC_SLEW ? RUNNING_SLEW_UP_PER_SECOND : SLEW_PER_SECOND;
        lastSlewDown = startup ? STARTUP_SLEW_DOWN_PER_SECOND
                : ENABLE_ASYMMETRIC_SLEW ? RUNNING_SLEW_DOWN_PER_SECOND : SLEW_PER_SECOND;

        if (Math.abs(error) <= Math.max(0.0, I_ZONE_RPM)) {
            integral += error * dt;
            integral = Range.clip(
                    integral,
                    Math.min(INTEGRAL_MIN, INTEGRAL_MAX),
                    Math.max(INTEGRAL_MIN, INTEGRAL_MAX)
            );
        } else {
            integral = 0.0;
        }

        double derivative = (error - lastErrorRpm) / dt;
        double targetAcceleration =
                (targetFlywheelRpm - lastTargetFlywheelRpm) / dt;

        lastPTerm = kP * error;
        lastITerm = kI * integral;
        lastDTerm = kD * derivative;
        lastStaticFeedforward = kS * Math.signum(targetFlywheelRpm);
        lastVelocityFeedforward = kV * targetFlywheelRpm;
        lastAccelerationFeedforward = kA * targetAcceleration;
        lastFeedforward = lastStaticFeedforward
                + lastVelocityFeedforward
                + lastAccelerationFeedforward;

        // Voltage compensation belongs on feedforward (the estimated motor power),
        // NOT on the PID feedback terms. Keep the target RPM unchanged.
        lastVoltageCompGain = computeVoltageCompGain();
        double compensatedFeedforward = lastFeedforward * lastVoltageCompGain;

        // Telemetry: estimated output without voltage compensation.
        lastOutputBeforeVoltageComp = Range.clip(
                lastPTerm + lastITerm + lastDTerm + lastFeedforward,
                0.0,
                1.0
        );

        double requestedOutput = Range.clip(
                lastPTerm + lastITerm + lastDTerm + compensatedFeedforward,
                0.0,
                1.0
        );
        lastControllerOutputBeforeBoost = requestedOutput;
        // A same-loop comparison from the same previous actual output, not an
        // independent simulation of how RPM would have evolved without boost.
        double upStep = Math.max(0.0, lastSlewUp) * dt;
        double downStep = Math.max(0.0, lastSlewDown) * dt;
        lastOutputWithoutBoostThisLoop = Range.clip(requestedOutput,
                lastAppliedPower - downStep, lastAppliedPower + upStep);
        lastPowerOverrideActive = Double.isFinite(minimumPowerOverride);
        lastOverrideFloor = lastBoostScaledFloor = lastBoostEffectiveFloor = 0.0;
        lastBoostVoltageGain = lastBoostOverspeedMultiplier = 1.0;
        if (lastPowerOverrideActive) {
            lastOverrideFloor = Range.clip(minimumPowerOverride, 0.0, 1.0);
            lastBoostVoltageGain = ShooterPowerProfile2026.boostVoltageGain(boostVoltageScaling,
                    lastBatteryVoltageFiltered, VOLTAGE_COMP_MIN_VALID_V, boostNominalVoltage);
            lastBoostScaledFloor = Range.clip(lastOverrideFloor * lastBoostVoltageGain, 0.0, 1.0);
            lastBoostOverspeedMultiplier = ShooterPowerProfile2026.overspeedMultiplier(
                    measuredFlywheelRpm, targetFlywheelRpm, boostOverspeedTaperRpm);
            // Taper only the extra floor above feedback. With taper disabled, keep
            // the original max(requested, floor) arithmetic exactly.
            lastBoostEffectiveFloor = boostOverspeedTaperRpm > 0
                    ? requestedOutput + Math.max(0, lastBoostScaledFloor-requestedOutput)
                        * lastBoostOverspeedMultiplier : lastBoostScaledFloor;
            requestedOutput = Math.max(requestedOutput, lastBoostEffectiveFloor);
        }
        lastOutputBeforeSlew = requestedOutput;

        // Limit the ACTUAL commanded motor power, after voltage compensation.
        double finalOutput;
        lastBoostBypassedSlew = lastPowerOverrideActive && powerOverrideBypassesSlew;
        if (lastPowerOverrideActive && powerOverrideBypassesSlew) {
            finalOutput = requestedOutput;
        } else {
            finalOutput = Range.clip(
                    requestedOutput,
                    lastAppliedPower - downStep,
                    lastAppliedPower + upStep
            );
        }
        finalOutput = Range.clip(finalOutput, 0.0, 1.0);
        lastSlewLimited = Math.abs(finalOutput - requestedOutput) > 1e-9;
        lastOutputAfterSlew = finalOutput;
        lastCommandSaturated = finalOutput >= 0.999;
        applyPower(finalOutput);

        lastErrorRpm = error;
        lastDerivativeRpmPerSecond = derivative;
        lastTargetAccelerationRpmPerSecond = targetAcceleration;
        lastTargetFlywheelRpm = targetFlywheelRpm;
        lastTimestampNanos = nowNanos;
    }

    /**
     * Starts or updates the shooter using a flywheel-RPM target.
     */
    public void spinUp(double flywheelRpm) {
        boolean wasEnabled = enabled;
        targetFlywheelRpm = Math.max(0.0, flywheelRpm);
        enabled = targetFlywheelRpm > 0.0;
        if (enabled && !wasEnabled) {
            validateConfiguration();
            controllerJustEnabled = true;
            integral = 0.0;
        }
    }

    public void toggle(double flywheelRpm) {
        if (enabled) {
            stop();
        } else {
            spinUp(flywheelRpm);
        }
    }

    /**
     * Temporarily prevents controller output from falling below the supplied
     * power. Intended for timed shot-recovery experiments.
     */
    public void setMinimumPowerOverride(double minimumPower, boolean bypassSlew) {
        setMinimumPowerOverride(minimumPower, bypassSlew, false, 12.5, 0.0);
    }

    /** Shot profile supplied by the sequence; voltage/RPM shaping uses controller-loop samples. */
    public void setMinimumPowerOverride(double minimumPower, boolean bypassSlew,
            boolean voltageScaling, double nominalVoltage, double taperRpm) {
        minimumPowerOverride = Range.clip(minimumPower, 0.0, 1.0);
        powerOverrideBypassesSlew = bypassSlew;
        boostVoltageScaling = voltageScaling;
        boostNominalVoltage = nominalVoltage;
        boostOverspeedTaperRpm = taperRpm;
    }

    /** Notification only: does not command a gate, motor, or permission to shoot. */
    public void setShotFeeding(boolean feeding) { shotFeeding = feeding; }

    public void clearMinimumPowerOverride() {
        minimumPowerOverride = Double.NaN;
        powerOverrideBypassesSlew = false;
        boostVoltageScaling = false;
        boostOverspeedTaperRpm = 0.0;
    }

    public void stop() {
        resetRuntimeState();
        if (shooterMotor != null) {
            shooterMotor.setPower(0.0);
        }
    }

    private void resetRuntimeState() {
        powerProfile.reset();
        shotFeeding = false;
        boostVoltageScaling = false;
        boostNominalVoltage = 12.5;
        boostOverspeedTaperRpm = 0.0;
        targetFlywheelRpm = 0.0;
        enabled = false;
        controllerJustEnabled = false;
        minimumPowerOverride = Double.NaN;
        powerOverrideBypassesSlew = false;
        lastTimestampNanos = 0L;
        lastEncoderTicks = 0;
        lastTargetFlywheelRpm = 0.0;
        lastMeasuredMotorRpm = 0.0;
        lastMeasuredFlywheelRpm = 0.0;
        lastEncoderDeltaMotorRpm = 0.0;
        lastEncoderDeltaFlywheelRpm = 0.0;
        lastLoopDtSeconds = 0.0;
        lastBatteryVoltageRaw = Double.NaN;
        lastBatteryVoltageFiltered = Double.NaN;
        lastVoltageCompGain = 1.0;
        lastMotorCurrentAmps = 0.0;
        lastAppliedPower = 0.0;
        clearControllerTerms();
    }

    private void clearControllerTerms() {
        lastErrorRpm = 0.0;
        integral = 0.0;
        lastDerivativeRpmPerSecond = 0.0;
        lastTargetAccelerationRpmPerSecond = 0.0;
        lastPTerm = 0.0;
        lastITerm = 0.0;
        lastDTerm = 0.0;
        lastStaticFeedforward = 0.0;
        lastVelocityFeedforward = 0.0;
        lastAccelerationFeedforward = 0.0;
        lastFeedforward = 0.0;
        lastOutputBeforeSlew = 0.0;
        lastOutputAfterSlew = 0.0;
        lastOutputBeforeVoltageComp = 0.0;
        lastControllerOutputBeforeBoost = 0.0;
        lastOutputWithoutBoostThisLoop = 0.0;
        lastSlewLimited = false;
        lastCommandSaturated = false;
        lastPowerOverrideActive = false;
        lastSlewUp = lastSlewDown = lastOverrideFloor = lastBoostScaledFloor = lastBoostEffectiveFloor = 0.0;
        lastBoostVoltageGain = lastBoostOverspeedMultiplier = 1.0;
        lastBoostBypassedSlew = false;
    }

    public static void validateConfiguration() {
        double[] nonnegative = {SLEW_PER_SECOND, RUNNING_SLEW_UP_PER_SECOND, RUNNING_SLEW_DOWN_PER_SECOND,
                STARTUP_SLEW_UP_PER_SECOND, STARTUP_SLEW_DOWN_PER_SECOND, STARTUP_ENTRY_MAX_RPM,
                STARTUP_EXIT_TOLERANCE_RPM, STARTUP_EXIT_MAX_SLOPE_RPM_PER_SECOND,
                STARTUP_EXIT_STABLE_SECONDS, STARTUP_MAX_SECONDS};
        for (double value : nonnegative) if (!Double.isFinite(value) || value < 0)
            throw new IllegalArgumentException("Shooter slew/startup settings must be finite and nonnegative");
        if (!Double.isFinite(ENCODER_TICKS_PER_MOTOR_REV) || ENCODER_TICKS_PER_MOTOR_REV <= 0
                || !Double.isFinite(MOTOR_RPM_PER_FLYWHEEL_RPM) || MOTOR_RPM_PER_FLYWHEEL_RPM <= 0)
            throw new IllegalArgumentException("Shooter encoder resolution and motor ratio must be positive");
        if (!Double.isFinite(VOLTAGE_COMP_MIN_GAIN) || !Double.isFinite(VOLTAGE_COMP_MAX_GAIN)
                || VOLTAGE_COMP_MIN_GAIN < 0 || VOLTAGE_COMP_MAX_GAIN < VOLTAGE_COMP_MIN_GAIN
                || !Double.isFinite(VOLTAGE_COMP_NOMINAL_V) || VOLTAGE_COMP_NOMINAL_V <= 0
                || !Double.isFinite(VOLTAGE_COMP_FILTER_TAU_SEC) || VOLTAGE_COMP_FILTER_TAU_SEC < 0
                || !Double.isFinite(VOLTAGE_COMP_MIN_VALID_V) || VOLTAGE_COMP_MIN_VALID_V <= 0)
            throw new IllegalArgumentException("Invalid shooter voltage compensation settings");
    }

    private void applyPower(double power) {
        lastAppliedPower = Range.clip(power, 0.0, 1.0);
        shooterMotor.setPower(lastAppliedPower);
    }

    private double readMeasuredMotorRpm() {
        if (ENCODER_TICKS_PER_MOTOR_REV <= 0.0) {
            return 0.0;
        }
        return Math.abs(ticksPerSecondToMotorRpm(shooterMotor.getVelocity()));
    }

    private static double ticksPerSecondToMotorRpm(double ticksPerSecond) {
        if (ENCODER_TICKS_PER_MOTOR_REV <= 0.0) {
            return 0.0;
        }
        return ticksPerSecond * 60.0 / ENCODER_TICKS_PER_MOTOR_REV;
    }

    private static double safeMotorRpmPerFlywheelRpm() {
        return Math.max(1e-6, Math.abs(MOTOR_RPM_PER_FLYWHEEL_RPM));
    }

    private static double flywheelToMotorRpm(double flywheelRpm) {
        return flywheelRpm * safeMotorRpmPerFlywheelRpm();
    }

    private static double motorToFlywheelRpm(double motorRpm) {
        return motorRpm / safeMotorRpmPerFlywheelRpm();
    }

    private double sampleBatteryVoltage() {
        double minimumVoltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : ActiveOpMode.hardwareMap().voltageSensor) {
            if (sensor == null) {
                continue;
            }
            double voltage = sensor.getVoltage();
            if (Double.isFinite(voltage)
                    && voltage >= VOLTAGE_COMP_MIN_VALID_V
                    && voltage < minimumVoltage) {
                minimumVoltage = voltage;
            }
        }
        return minimumVoltage == Double.POSITIVE_INFINITY
                ? Double.NaN
                : minimumVoltage;
    }

    private void updateBatteryVoltageEstimate(double dtSeconds) {
        lastBatteryVoltageRaw = sampleBatteryVoltage();
        if (!Double.isFinite(lastBatteryVoltageRaw)) {
            return;
        }
        if (!Double.isFinite(lastBatteryVoltageFiltered)) {
            lastBatteryVoltageFiltered = lastBatteryVoltageRaw;
            return;
        }

        double tau = Math.max(0.01, VOLTAGE_COMP_FILTER_TAU_SEC);
        double alpha = dtSeconds <= 0.0
                ? 1.0
                : Range.clip(dtSeconds / (tau + dtSeconds), 0.0, 1.0);
        lastBatteryVoltageFiltered +=
                alpha * (lastBatteryVoltageRaw - lastBatteryVoltageFiltered);
    }

    private double computeVoltageCompGain() {
        if (!ENABLE_VOLTAGE_COMPENSATION
                || !Double.isFinite(lastBatteryVoltageFiltered)
                || lastBatteryVoltageFiltered < VOLTAGE_COMP_MIN_VALID_V) {
            return 1.0;
        }
        return Range.clip(
                Math.max(0.1, VOLTAGE_COMP_NOMINAL_V)
                        / lastBatteryVoltageFiltered,
                Math.max(0.0, VOLTAGE_COMP_MIN_GAIN),
                Math.max(VOLTAGE_COMP_MIN_GAIN, VOLTAGE_COMP_MAX_GAIN)
        );
    }

    public boolean isEnabled() {
        return enabled;
    }

    public boolean isAtSpeed(double toleranceRpm) {
        return enabled
                && Math.abs(lastErrorRpm) <= Math.max(0.0, toleranceRpm);
    }

    public double getTargetFlywheelRpm() {
        return targetFlywheelRpm;
    }

    public double getTargetMotorRpm() {
        return flywheelToMotorRpm(targetFlywheelRpm);
    }

    public double getMeasuredFlywheelRpm() {
        return lastMeasuredFlywheelRpm;
    }

    public double getMeasuredMotorRpm() {
        return lastMeasuredMotorRpm;
    }

    public double getEncoderDeltaFlywheelRpm() {
        return lastEncoderDeltaFlywheelRpm;
    }

    public double getEncoderDeltaMotorRpm() {
        return lastEncoderDeltaMotorRpm;
    }

    public double getLoopDtSeconds() {
        return lastLoopDtSeconds;
    }

    public double getErrorRpm() {
        return lastErrorRpm;
    }

    public double getIntegralAccumulator() {
        return integral;
    }

    public double getDerivativeRpmPerSecond() {
        return lastDerivativeRpmPerSecond;
    }

    public double getTargetAccelerationRpmPerSecond() {
        return lastTargetAccelerationRpmPerSecond;
    }

    public double getPTerm() {
        return lastPTerm;
    }

    public double getITerm() {
        return lastITerm;
    }

    public double getDTerm() {
        return lastDTerm;
    }

    public double getStaticFeedforward() {
        return lastStaticFeedforward;
    }

    public double getVelocityFeedforward() {
        return lastVelocityFeedforward;
    }

    public double getAccelerationFeedforward() {
        return lastAccelerationFeedforward;
    }

    public double getFeedforward() {
        return lastFeedforward;
    }

    public double getOutputBeforeSlew() {
        return lastOutputBeforeSlew;
    }

    public double getOutputAfterSlew() {
        return lastOutputAfterSlew;
    }

    public double getOutputBeforeVoltageComp() {
        return lastOutputBeforeVoltageComp;
    }

    public double getAppliedPower() {
        return lastAppliedPower;
    }

    public long getControlSampleNanos() { return lastTimestampNanos; }
    /** Target associated with the cached controller update, before a later OpMode request. */
    public double getControllerTargetFlywheelRpm() { return lastTargetFlywheelRpm; }
    public double getControllerOutputBeforeBoost() { return lastControllerOutputBeforeBoost; }
    public double getOutputWithoutBoostThisLoop() { return lastOutputWithoutBoostThisLoop; }
    public double getBoostRequestedPowerIncrease() {
        return Math.max(0.0, lastOutputBeforeSlew - lastControllerOutputBeforeBoost);
    }
    public double getBoostAppliedPowerDifference() {
        return lastAppliedPower - lastOutputWithoutBoostThisLoop;
    }

    public boolean wasSlewLimited() {
        return lastSlewLimited;
    }

    public boolean isCommandSaturated() {
        return lastCommandSaturated;
    }

    public boolean isMinimumPowerOverrideActive() {
        return lastPowerOverrideActive;
    }

    public double getMinimumPowerOverride() {
        return Double.isFinite(minimumPowerOverride)
                ? Range.clip(minimumPowerOverride, 0.0, 1.0)
                : 0.0;
    }

    public boolean isPowerOverrideBypassingSlew() {
        return lastPowerOverrideActive && powerOverrideBypassesSlew;
    }

    public double getMotorCurrentAmps() {
        return lastMotorCurrentAmps;
    }

    public double getBatteryVoltageRaw() {
        return lastBatteryVoltageRaw;
    }

    public double getBatteryVoltageFiltered() {
        return lastBatteryVoltageFiltered;
    }

    public double getVoltageCompGain() {
        return lastVoltageCompGain;
    }

    public String getPowerProfileState() { return powerProfile.getState().name(); }
    public String getStartupExitReason() { return powerProfile.getExitReason(); }
    public double getStartupElapsedSeconds() { return powerProfile.getElapsedSeconds(); }
    public double getMeasuredAccelerationRpmPerSecond() { return powerProfile.getMeasuredSlope(); }
    public double getActiveSlewUpPerSecond() { return lastSlewUp; }
    public double getActiveSlewDownPerSecond() { return lastSlewDown; }
    public double getSampleOverrideFloor() { return lastOverrideFloor; }
    public double getBoostVoltageGain() { return lastBoostVoltageGain; }
    public double getBoostScaledFloor() { return lastBoostScaledFloor; }
    public double getBoostOverspeedMultiplier() { return lastBoostOverspeedMultiplier; }
    public double getBoostEffectiveFloor() { return lastBoostEffectiveFloor; }
    public boolean didBoostBypassSlew() { return lastBoostBypassedSlew; }
}
