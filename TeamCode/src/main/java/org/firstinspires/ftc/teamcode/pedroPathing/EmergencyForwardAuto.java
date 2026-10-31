package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.GlobalRobotData;

/**
 * Competition fallback autonomous that does not use Pedro to control motion.
 *
 * <p>Pedro updates the Pinpoint pose for telemetry and the TeleOp handoff, but
 * the four drive motors receive direct, equal power for a fixed duration.</p>
 */
@Configurable
@Autonomous(name = "Emergency Reverse 2 Seconds", group = "Comp")
public class EmergencyForwardAuto extends LinearOpMode {

    public static double START_X_INCHES = 12.6;
    public static double START_Y_INCHES = 132.3;
    public static double START_HEADING_DEGREES = 90.0;

    public static double FORWARD_POWER = 0.30;
    public static double DRIVE_SECONDS = 0.7;

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.createFollower(hardwareMap);
        // Configure after follower construction because Pedro initializes the
        // same motors to FLOAT while creating its drivetrain.
        configureDriveMotors();
        follower.setStartingPose(new Pose(
                START_X_INCHES,
                START_Y_INCHES,
                Math.toRadians(START_HEADING_DEGREES)
        ));

        GlobalRobotData.hasAutonRun = false;
        GlobalRobotData.endAutonPose = null;

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) {
                GlobalRobotData.allianceSide = GlobalRobotData.COLOR.BLUE;
            } else if (gamepad1.b) {
                GlobalRobotData.allianceSide = GlobalRobotData.COLOR.RED;
            }

            follower.updatePose();
            Pose pose = follower.getPose();

            telemetry.addLine("Hello Pickle of the robot");
            telemetry.addLine("This is an Mr. Todone Speaking,");
            telemetry.addLine("----------------------------------------------");
            telemetry.addLine(
                    GlobalRobotData.allianceSide == GlobalRobotData.COLOR.BLUE
                            ? "Favorite fruit: Blueberries!!! (Blue)"
                            : "Favorite fruit: Raspberries!!! (Red)"
            );
            telemetry.addLine("Press X for BLUE or B for RED");
            telemetry.addLine("Direct motor fallback: Pedro does not control driving");
            telemetry.addData("Power", "%.2f", positivePower());
            telemetry.addData("Drive time", "%.2f sec", positiveDriveSeconds());
            addPoseTelemetry(pose);
            telemetry.update();

            sleep(20);
        }

        if (isStopRequested()) {
            stopDriveMotors();
            return;
        }

        boolean autonomousStarted = true;
        ElapsedTime driveTimer = new ElapsedTime();

        try {
            driveTimer.reset();

            while (opModeIsActive()
                    && driveTimer.seconds() < positiveDriveSeconds()) {
                setAllDrivePower(negativePower());
                follower.updatePose();

                telemetry.addLine("DRIVING WITH DIRECT MOTOR POWER");
                telemetry.addData(
                        "Time",
                        "%.2f / %.2f sec",
                        driveTimer.seconds(),
                        positiveDriveSeconds()
                );
                addPoseTelemetry(follower.getPose());
                telemetry.update();
                idle();
            }
        } finally {
            stopDriveMotors();

            if (autonomousStarted) {
                follower.updatePose();
                GlobalRobotData.endAutonPose = follower.getPose().copy();
                GlobalRobotData.endAutonBallCount = -1;
                GlobalRobotData.endAutonTurretAngleDegrees = Double.NaN;
                GlobalRobotData.endAutonTurretServoCommandAngleDegrees = Double.NaN;
                GlobalRobotData.endAutonTurretServoOffsetDegrees = Double.NaN;
                GlobalRobotData.hasAutonRun = true;
            }
        }
    }

    private void configureDriveMotors() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right");

        // These are the directions verified by Pickles Drive Motor Test.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        configureMotor(frontLeft);
        configureMotor(frontRight);
        configureMotor(backLeft);
        configureMotor(backRight);
        stopDriveMotors();
    }

    private static void configureMotor(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void setAllDrivePower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    private void stopDriveMotors() {
        if (frontLeft != null) frontLeft.setPower(0.0);
        if (frontRight != null) frontRight.setPower(0.0);
        if (backLeft != null) backLeft.setPower(0.0);
        if (backRight != null) backRight.setPower(0.0);
    }

    private static double positivePower() {
        return Range.clip(Math.abs(FORWARD_POWER), 0.0, 1.0);
    }
    private static double negativePower() {
        return FORWARD_POWER;
    }

    private static double positiveDriveSeconds() {
        return Math.max(0.0, DRIVE_SECONDS);
    }

    private void addPoseTelemetry(Pose pose) {
        telemetry.addData("X", "%.2f in", pose.getX());
        telemetry.addData("Y", "%.2f in", pose.getY());
        telemetry.addData("Heading", "%.1f deg", Math.toDegrees(pose.getHeading()));
    }
}
