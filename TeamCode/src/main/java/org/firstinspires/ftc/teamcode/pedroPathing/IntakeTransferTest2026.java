package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeWithSensorsSubsystem2026;
import org.firstinspires.ftc.teamcode.subsystems.LEDControlSubsystem2026;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/**
 * Tests the normal intake sequence without requiring a physical top break
 * beam. A starts or re-arms intake, while B simulates the beam becoming
 * blocked so only the transfer stops.
 */
@TeleOp(name = "Intake Transfer Test 2026", group = "Test")
public class IntakeTransferTest2026 extends NextFTCOpMode {

    public IntakeTransferTest2026() {
        addComponents(
                new SubsystemComponent(
                        IntakeWithSensorsSubsystem2026.INSTANCE,
                        LEDControlSubsystem2026.INSTANCE
                ),
                BulkReadComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        IntakeWithSensorsSubsystem2026.INSTANCE.stopAll();
    }

    @Override
    public void onWaitForStart() {
        telemetry.addLine("Intake/transfer motor test ready");
        telemetry.addLine("Gamepad 1 A: start/re-arm intake and transfer");
        telemetry.addLine("Gamepad 1 B: simulate top beam (stop transfer)");
        telemetry.addLine("Gamepad 1 Y: start jam reverse");
        telemetry.addLine("Gamepad 1 X: stop both motors");
        telemetry.update();
    }

    @Override
    public void onUpdate() {
        IntakeWithSensorsSubsystem2026 intake =
                IntakeWithSensorsSubsystem2026.INSTANCE;

        if (gamepad1.xWasPressed()) {
            intake.stopAll();
        } else if (gamepad1.yWasPressed()) {
            intake.setJamReversing(true);
        } else if (gamepad1.aWasPressed()) {
            intake.intakeForward();
        } else if (gamepad1.bWasPressed()) {
            intake.simulateUpperStorageFullForTesting();
        }

        telemetry.addData("Intake mode", intake.getIntakeMode());
        telemetry.addData("Jam reverse", intake.isJamReversing());
        telemetry.addData("Tracked balls", "%d / 4", intake.getBallCount());
        telemetry.addData(
                "Simulated upper storage",
                intake.isUpperStorageFullSimulated() ? "FULL" : "CLEAR"
        );
        telemetry.addData(
                "Transfer state",
                intake.isTransferStoppedForCapacity()
                        ? "STOPPED BY SIMULATED BEAM"
                        : "RUNNING WHEN INTAKE IS ACTIVE"
        );
        telemetry.addData(
                "Motor A (intake) target",
                "%.0f RPM",
                intake.getAppliedIntakeRpm()
        );
        telemetry.addData(
                "Motor A (intake) measured",
                "%.0f RPM",
                intake.getMeasuredIntakeMotorRpm()
        );
        telemetry.addData(
                "Motor B (transfer) target",
                "%.0f RPM",
                intake.getAppliedTransferRpm()
        );
        telemetry.addData(
                "Motor B (transfer) measured",
                "%.0f RPM",
                intake.getMeasuredTransferMotorRpm()
        );
        telemetry.addData(
                "Shooter gate",
                "%.3f (closed while testing)",
                intake.getGatePosition()
        );
        telemetry.update();
    }

    @Override
    public void onStop() {
        IntakeWithSensorsSubsystem2026.INSTANCE.stopAll();
        super.onStop();
    }
}
