package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;

/**
 * Runs each drivetrain motor independently without Pedro.
 *
 * <p>Put the robot on blocks before running this test. Each button applies the
 * same positive logical power after using the motor directions from Constants.
 * The selected wheel should rotate in the direction that would drive that
 * corner of the robot forward.</p>
 */
@Configurable
@TeleOp(name = "Pickles Drive Motor Test", group = "Test")
public class PicklesDriveMotorTest extends OpMode {

    public static double TEST_POWER = 0.30;

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right");

        // Keep this test aligned with the active Pedro drivetrain configuration.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        configureMotor(frontLeft);
        configureMotor(frontRight);
        configureMotor(backLeft);
        configureMotor(backRight);

        stopAllMotors();
        showInstructions();
    }

    @Override
    public void init_loop() {
        showInstructions();
    }

    @Override
    public void loop() {
        double power = Range.clip(Math.abs(TEST_POWER), 0.0, 1.0);

        // Face-button layout mirrors the four robot corners:
        // X/Y are the front row; A/B are the rear row.
        double frontLeftPower = gamepad1.x ? power : 0.0;
        double frontRightPower = gamepad1.y ? power : 0.0;
        double backLeftPower = gamepad1.a ? power : 0.0;
        double backRightPower = gamepad1.b ? power : 0.0;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        List<String> activeMotors = new ArrayList<>();
        if (frontLeftPower > 0.0) activeMotors.add("front_left (X)");
        if (frontRightPower > 0.0) activeMotors.add("front_right (Y)");
        if (backLeftPower > 0.0) activeMotors.add("back_left (A)");
        if (backRightPower > 0.0) activeMotors.add("back_right (B)");

        telemetry.addLine("ROBOT MUST BE ON BLOCKS");
        telemetry.addLine("Hold buttons; releasing them stops the motor.");
        telemetry.addLine("X=front_left  Y=front_right  A=back_left  B=back_right");
        telemetry.addData(
                "Should be spinning",
                activeMotors.isEmpty() ? "NONE" : String.join(", ", activeMotors)
        );
        telemetry.addData("Commanded positive power", "%.2f", power);
        telemetry.addLine();

        addMotorTelemetry("X front_left", frontLeft, frontLeftPower);
        addMotorTelemetry("Y front_right", frontRight, frontRightPower);
        addMotorTelemetry("A back_left", backLeft, backLeftPower);
        addMotorTelemetry("B back_right", backRight, backRightPower);

        telemetry.addLine();
        telemetry.addLine("Expected: the named wheel turns in the robot-forward direction.");
        telemetry.addLine("Wrong wheel = hardware-map name/port issue.");
        telemetry.addLine("Right wheel, wrong direction = direction setting issue.");
        telemetry.update();
    }

    @Override
    public void stop() {
        stopAllMotors();
    }

    private static void configureMotor(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void stopAllMotors() {
        if (frontLeft != null) frontLeft.setPower(0.0);
        if (frontRight != null) frontRight.setPower(0.0);
        if (backLeft != null) backLeft.setPower(0.0);
        if (backRight != null) backRight.setPower(0.0);
    }

    private void showInstructions() {
        telemetry.addLine("Pickles Drive Motor Test");
        telemetry.addLine("ROBOT MUST BE ON BLOCKS");
        telemetry.addLine("Press Play, then hold one button at a time:");
        telemetry.addLine("X=front_left  Y=front_right");
        telemetry.addLine("A=back_left   B=back_right");
        telemetry.addData("Positive test power", "%.2f", Math.abs(TEST_POWER));
        telemetry.update();
    }

    private void addMotorTelemetry(String label, DcMotorEx motor, double commandedPower) {
        telemetry.addData(
                label,
                "command=%+.2f  direction=%s  encoder=%d  velocity=%+.0f",
                commandedPower,
                motor.getDirection(),
                motor.getCurrentPosition(),
                motor.getVelocity()
        );
    }
}
