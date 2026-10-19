package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Constants {

    /* public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.106)
            .forwardZeroPowerAcceleration(-30.5)
            .lateralZeroPowerAcceleration(-67)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .centripetalScaling(0.0004)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(3, 0, 0.1, 0))
            .drivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.04, 0, 0.0002, 0.6, 0)
            ); */
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.106)
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.11, 0)) // tuned constants
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.1, 0.09629215, 0.001609466)) // (kP, kLinear, kQuadratic)
            .centripetalScaling(0);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("front_left")
            .leftRearMotorName("back_left")
            .rightFrontMotorName("front_right")
            .rightRearMotorName("back_right")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(78) //TODONE: Recheck with new battery 😎👌👌
            .yVelocity(62.5) //TODONE: Recheck with new battery 😎👌👌
            .useBrakeModeInTeleOp(true);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2.75)  //Based on CAD, looks like -3.15 inches
            .strafePodX(-6.75)  //Based on CAD, looks like -6.85 inches
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
            )
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.95,
            100,
            1.0,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}

