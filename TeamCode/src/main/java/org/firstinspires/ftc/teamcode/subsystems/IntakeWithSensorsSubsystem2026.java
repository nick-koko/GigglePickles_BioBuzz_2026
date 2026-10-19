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
 * <p>Current hardware consists of one intake motor, one transfer motor, one
 * break beam immediately before the shooter, and a servo gate. The class is
 * intentionally small now, but keeps those responsibilities separate from the
 * shooter so more sensors and staged-ball logic can be added later.</p>
 */
@Configurable
public class IntakeWithSensorsSubsystem2026 implements Subsystem {

    public static final IntakeWithSensorsSubsystem2026 INSTANCE =
            new IntakeWithSensorsSubsystem2026();

    public enum IntakeMode {
        STOPPED,
        FORWARD
    }

    public static double INTAKE_FORWARD_RPM = 4000.0;
    public static double TRANSFER_FORWARD_RPM = 4000.0;
    public static double INTAKE_REVERSE_RPM = 3000.0;
    public static double TRANSFER_REVERSE_RPM = 1000.0;
    public static double SHOOT_INTAKE_POWER = 1.0;
    public static double SHOOT_TRANSFER_POWER = 1.0;
    public static boolean USE_TOP_BREAK_BEAM =false;

    public static double GATE_CLOSED_POSITION = 0.15;
    public static double GATE_OPEN_POSITION = 0.35;

    private static final double ENCODER_TICKS_PER_MOTOR_REV = 28.0;
    private static final double SHAFT_REDUCTION = 3.0;

    private DcMotorEx intakeMotor;
    private DcMotorEx transferMotor;
    private DigitalChannel topBreakBeam;
    private Servo shooterGate;

    private IntakeMode intakeMode = IntakeMode.STOPPED;
    private boolean feedingToShooter;
    private boolean jamReversing;
    private boolean transferStoppedByBeam;
    private boolean previousTopBeamBroken;
    private double appliedIntakeRpm;
    private double appliedTransferRpm;
    private double appliedIntakePower;
    private double appliedTransferPower;

    private IntakeWithSensorsSubsystem2026() {
    }

    @Override
    public void initialize() {
        resetRuntimeState();

        intakeMotor = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "intake_motor2");
        transferMotor = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "intake_motor");
        topBreakBeam = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "breakbeam0");
        shooterGate = ActiveOpMode.hardwareMap().get(Servo.class, "shooter_gate");

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Preserve the corresponding 2025 motor directions.
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterGate.setDirection(Servo.Direction.REVERSE);
        topBreakBeam.setMode(DigitalChannel.Mode.INPUT);
        previousTopBeamBroken = isTopBeamBroken();
        shooterGate.setPosition(clippedGatePosition(GATE_CLOSED_POSITION));
        applyOutputs();
    }

    @Override
    public void periodic() {
        applyOutputs();
    }

    public void intakeForward() {
        intakeMode = IntakeMode.FORWARD;
        transferStoppedByBeam = false;
    }

    public void stopIntake() {
        feedingToShooter = false;
        jamReversing = false;
        intakeMode = IntakeMode.STOPPED;
    }

    /**
     * Temporarily reverses both motors while held. Releasing the button returns
     * to the previously selected STOPPED or FORWARD intake mode.
     */
    public void setJamReversing(boolean reversing) {
        jamReversing = reversing;
    }

    /**
     * Opens the gate and advances the transfer toward the shooter while true.
     * This intentional feed command bypasses the normal top-beam transfer stop.
     */
    public void setFeedingToShooter(boolean feeding) {
        feedingToShooter = feeding;
    }

    public void stopAll() {
        resetRuntimeState();

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
        intakeMode = IntakeMode.STOPPED;
        feedingToShooter = false;
        jamReversing = false;
        transferStoppedByBeam = false;
        previousTopBeamBroken = false;
        appliedIntakeRpm = 0.0;
        appliedTransferRpm = 0.0;
        appliedIntakePower = 0.0;
        appliedTransferPower = 0.0;
    }

    private void applyOutputs() {
        if (intakeMotor == null || transferMotor == null
                || topBreakBeam == null || shooterGate == null) {
            return;
        }

        boolean topBeamBroken = isTopBeamBroken();
        boolean topBeamJustBroke = topBeamBroken && !previousTopBeamBroken;
        previousTopBeamBroken = topBeamBroken;

        if (!USE_TOP_BREAK_BEAM) {
            transferStoppedByBeam = false;
        } else if (intakeMode == IntakeMode.FORWARD
                && !jamReversing
                && !feedingToShooter
                && topBeamJustBroke) {
            // Latch the stop. A hole in a ball may make the beam read clear later,
            // but the transfer must stay stopped until the operator presses A again.
            transferStoppedByBeam = true;
        }

        appliedIntakeRpm = 0.0;
        appliedTransferRpm = 0.0;
        appliedIntakePower = 0.0;
        appliedTransferPower = 0.0;

        if (jamReversing) {
            appliedIntakeRpm = -positiveRpm(INTAKE_REVERSE_RPM);
            appliedTransferRpm = positiveRpm(TRANSFER_FORWARD_RPM);
            setMotorVelocities(appliedIntakeRpm, appliedTransferRpm);
        } else if (feedingToShooter) {
            appliedIntakePower = clippedPower(SHOOT_INTAKE_POWER);
            appliedTransferPower = clippedPower(SHOOT_TRANSFER_POWER);
            intakeMotor.setPower(appliedIntakePower);
            transferMotor.setPower(appliedTransferPower);
        } else if (intakeMode == IntakeMode.FORWARD) {
            appliedIntakeRpm = positiveRpm(INTAKE_FORWARD_RPM);
            appliedTransferRpm = transferStoppedByBeam
                    ? 0.0
                    : positiveRpm(TRANSFER_FORWARD_RPM);
            setMotorVelocities(appliedIntakeRpm, appliedTransferRpm);
        } else {
            setMotorVelocities(0.0, 0.0);
        }

        shooterGate.setPosition(clippedGatePosition(
                feedingToShooter && !jamReversing
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

    public boolean isTopBeamBroken() {
        return topBreakBeam != null && !topBreakBeam.getState();
    }

    public boolean isTransferBlockedByBeam() {
        return USE_TOP_BREAK_BEAM && transferStoppedByBeam;
    }

    public boolean isFeedingToShooter() {
        return feedingToShooter;
    }

    public boolean isJamReversing() {
        return jamReversing;
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
}
