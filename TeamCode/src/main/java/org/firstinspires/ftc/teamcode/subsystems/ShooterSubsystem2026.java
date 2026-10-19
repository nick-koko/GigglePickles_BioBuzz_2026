package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

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

    public static boolean ENABLE_VOLTAGE_COMPENSATION = false;
    public static double VOLTAGE_COMP_NOMINAL_V = 12.5;
    public static double VOLTAGE_COMP_FILTER_TAU_SEC = 0.5;
    public static double VOLTAGE_COMP_MIN_GAIN = 1.0;
    public static double VOLTAGE_COMP_MAX_GAIN = 1.10;
    public static double VOLTAGE_COMP_MIN_VALID_V = 7.0;

    private DcMotorEx shooterMotor;

    private double targetFlywheelRpm;
    private boolean enabled;
    private boolean controllerJustEnabled;
    private double minimumPowerOverride = Double.NaN;
    private boolean powerOverrideBypassesSlew;

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
            applyPower(0.0);
            clearControllerTerms();
            lastTimestampNanos = nowNanos;
            lastTargetFlywheelRpm = 0.0;
            return;
        }

        double error = targetFlywheelRpm - measuredFlywheelRpm;

        if (controllerJustEnabled) {
            lastErrorRpm = error;
            lastTargetFlywheelRpm = targetFlywheelRpm;
            controllerJustEnabled = false;
        }

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

        double requestedOutput = Range.clip(
                lastPTerm + lastITerm + lastDTerm + lastFeedforward,
                0.0,
                1.0
        );
        lastPowerOverrideActive = Double.isFinite(minimumPowerOverride);
        if (lastPowerOverrideActive) {
            requestedOutput = Math.max(
                    requestedOutput,
                    Range.clip(minimumPowerOverride, 0.0, 1.0)
            );
        }
        lastOutputBeforeSlew = requestedOutput;

        double maxStep = Math.max(0.0, SLEW_PER_SECOND) * dt;
        double outputAfterSlew;
        if (lastPowerOverrideActive && powerOverrideBypassesSlew) {
            outputAfterSlew = requestedOutput;
        } else {
            outputAfterSlew = Range.clip(
                    requestedOutput,
                    lastAppliedPower - maxStep,
                    lastAppliedPower + maxStep
            );
        }
        outputAfterSlew = Range.clip(outputAfterSlew, 0.0, 1.0);
        lastSlewLimited = Math.abs(outputAfterSlew - requestedOutput) > 1e-9;
        lastOutputAfterSlew = outputAfterSlew;
        lastOutputBeforeVoltageComp = outputAfterSlew;

        lastVoltageCompGain = computeVoltageCompGain();
        double finalOutput = Range.clip(
                outputAfterSlew * lastVoltageCompGain,
                0.0,
                1.0
        );
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
        minimumPowerOverride = Range.clip(minimumPower, 0.0, 1.0);
        powerOverrideBypassesSlew = bypassSlew;
    }

    public void clearMinimumPowerOverride() {
        minimumPowerOverride = Double.NaN;
        powerOverrideBypassesSlew = false;
    }

    public void stop() {
        resetRuntimeState();
        if (shooterMotor != null) {
            shooterMotor.setPower(0.0);
        }
    }

    private void resetRuntimeState() {
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
        lastSlewLimited = false;
        lastCommandSaturated = false;
        lastPowerOverrideActive = false;
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
}
