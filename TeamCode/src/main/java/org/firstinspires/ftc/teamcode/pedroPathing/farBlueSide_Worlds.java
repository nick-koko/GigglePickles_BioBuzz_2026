package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.GlobalRobotData;
import org.firstinspires.ftc.teamcode.subsystems.IntakeWithSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDControlSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;

@Disabled
@Configurable
@Autonomous(name = "WorldsfarBlueSide", group = "Comp")
public class farBlueSide_Worlds extends farAutonPaths_Worlds {
    public farBlueSide_Worlds() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(ShooterSubsystem.INSTANCE, IntakeWithSensorsSubsystem.INSTANCE, TurretSubsystem.INSTANCE)
        );
    }
    public double intAmount = 18;
    public double pushLever = 1;

    private Pose finalStartPose = new Pose();

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void onInit() {
        TurretSubsystem.INSTANCE.beginStartupCentering();
        ShooterSubsystem.INSTANCE.shooterHoodDrive(autonShooterHoodServoPos);
        ShooterSubsystem.INSTANCE.stop();

        GlobalRobotData.allianceSide = GlobalRobotData.COLOR.BLUE;
        PedroComponent.follower().setStartingPose(startPoseBlue);

        // Seed ball count for auton: assume robot starts loaded with 3
        IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3);

        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void onWaitForStart() {
        boolean turretStartupCalibrated = TurretSubsystem.INSTANCE.updateStartupCalibrationFromExpected(
                TurretSubsystem.INITIAL_ANGLE_DEGREES
        );

            /*if (gamepad1.x) {
                GlobalRobotData.allianceSide = GlobalRobotData.COLOR.BLUE;
                finalStartPose = startPoseBlue.copy();
            } else if (gamepad1.b) {
                GlobalRobotData.allianceSide = GlobalRobotData.COLOR.RED;
                finalStartPose = startPoseRed.copy();
            }*/

        // If dpad Up/Down is pressed, increase or decrease ball count
        if ((gamepad1.dpadUpWasPressed()) && (intAmount < 24)) {
            intAmount = intAmount + 3;
        } else if ((gamepad1.dpadDownWasPressed()) && (intAmount > 18)) {
            intAmount = intAmount - 3;
        }
        // If dpad left/right is pressed add or subtract a row until push lever
        if ((gamepad1.dpadRightWasPressed()) && (pushLever < 2)) {
            pushLever = pushLever + 1;
        } else if ((gamepad1.dpadLeftWasPressed()) && (pushLever > 0)) {
            pushLever = pushLever - 1;
        }

        //TODO Add dpad Left/Right to set when to hit gate lever (after 1st pickup, second pickup, or both)

        telemetry.addLine("Hello Pickle of the robot");
        telemetry.addLine("This is an Mr. Todone Speaking,");
        telemetry.addLine("----------------------------------------------");
        if (GlobalRobotData.allianceSide == GlobalRobotData.COLOR.BLUE) {
            telemetry.addLine("Favorite fruit: Blueberries!!! (Blue)");
        } else {
            telemetry.addLine("Favorite fruit: Raspberries!!! (Red)");
        }
        telemetry.addLine();
        telemetry.addData("Eating this number of balls: ", intAmount);
        telemetry.addData("Eating from the basket this number: ", pushLever);

        telemetry.addData("heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
        telemetry.addData("turretStartupCal", turretStartupCalibrated);
        telemetry.addData("turretStartupState", TurretSubsystem.INSTANCE.getStartupCalibrationStateName());

        telemetry.update();

    }


    public Command Close3Ball() {
        return new SequentialGroup(
                CloseShootPreload(),
                CloseMoveOffLine()
        );
    }

    public Command Close6Ball() {
        return new SequentialGroup(
                CloseShootPreload(),
                CloseGoToFirstPickupLine(),
                ClosePickupAndShootFirstRow(),
                CloseMoveOffLine()
        );

    }

    public Command Close9Ball() {
        return new SequentialGroup(
                CloseShootPreload(),
                CloseGoToFirstPickupLine(),
                ClosePickupAndShootFirstRow(),
                CloseGoTo2ndPickupLine(),
                //ClosePickupAndShoot2ndRow(),
                CloseMoveOffLineToLever()
        );

    }
    public Command Close12Ball() {
        return new SequentialGroup(
                CloseShootPreload(),
                CloseGoToFirstPickupLine(),
                ClosePickupAndShootFirstRow(),
                CloseGoTo2ndPickupLine(),
                //ClosePickupAndShoot2ndRow(),
                CloseGoTo3rdPickupLine(),
                ClosePickupAndShoot3rdRow(),
                CloseMoveOffLineToLever()
        );
    }

    public Command Close15BallLeverAfter6() {
        return new SequentialGroup(
                CloseShootPreload(),

                CloseGoToFirstPickupLine(),
                ClosePickupAndShootFirstRow(),

                CloseGoTo2ndPickupLine(),
                ClosePickupAndGateLever2ndRow(),
                new Delay(.1),
                ClosePickupShootAfterGateLever2ndRow(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseGoToTZonePickupLine(),
                ClosePickupAndShootTZoneRow(),

                CloseMoveOffLineToLever()
        );
    }

    public Command Close18Ball() {
        return new SequentialGroup(
                CloseShootPreload(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseGoToTZonePickupLine(),
                ClosePickupAndShootTZoneRow(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseGoToTZonePickupLine(),
                ClosePickupAndShootTZoneRow(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseMoveOffLineToLever()
        );
    }

    public Command Close18BallNo2ndSpike() {
        return new SequentialGroup(
                CloseShootPreload(),

                CloseGoToFirstPickupLine(),
                ClosePickupAndShootFirstRow(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseGoToTZonePickupLine(),
                ClosePickupAndShootTZoneRow(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseGoToTZonePickupLine(),
                ClosePickupAndShootTZoneRow(),

                CloseMoveOffLineToLever()
        );
    }

    public Command Close18BallLeverAfter6() {
        return new SequentialGroup(
                CloseShootPreload(),

                CloseGoToFirstPickupLine(),
                ClosePickupAndShootFirstRow(),

                CloseGoTo2ndPickupLine(),
                ClosePickupAndGateLever2ndRow(),
                new Delay(.1),
                ClosePickupShootAfterGateLever2ndRow(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseGoToTZonePickupLine(),
                ClosePickupAndShootTZoneRow(),

                CloseGoToTZonePickupLine(),
                ClosePickupAndShootTZoneRow(),

                CloseMoveOffLineToLever()
        );
    }
    public Command Close21Ball() {
        return new SequentialGroup(
                CloseShootPreload(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseGoToTZonePickupLine(),
                ClosePickupAndShootTZoneRow(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseGoToTZonePickupLine(),
                ClosePickupAndShootTZoneRow(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseMoveOffLineToLever()
        );
    }

    public Command Close21BallNo2ndSpike() {
        return new SequentialGroup(
                CloseShootPreload(),

                CloseGoToFirstPickupLine(),
                ClosePickupAndShootFirstRow(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseGoToTZonePickupLine(),
                ClosePickupAndShootTZoneRow(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseGoToTZonePickupLine(),
                ClosePickupAndShootTZoneRow(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseMoveOffLineToLever()
        );
    }

    public Command Close21BallLeverAfter6() {
        return new SequentialGroup(
                CloseShootPreload(),

                CloseGoToFirstPickupLine(),
                ClosePickupAndShootFirstRow(),

                CloseGoTo2ndPickupLine(),
                ClosePickupAndGateLever2ndRow(),
                new Delay(.1),
                ClosePickupShootAfterGateLever2ndRow(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseGoToTZonePickupLine(),
                ClosePickupAndShootTZoneRow(),

                CloseGoToTZonePickupLine(),
                ClosePickupAndShootTZoneRow(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseMoveOffLineToLever()
        );
    }
    public Command Close24Ball() {
        return new SequentialGroup(
                CloseShootPreload(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseGoToTZonePickupLine(),
                ClosePickupAndShootTZoneRow(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseGoToTZonePickupLine(),
                ClosePickupAndShootTZoneRow(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseMoveOffLineToLever()
        );
    }

    public Command Close24BallNo2ndSpike() {
        return new SequentialGroup(
                CloseShootPreload(),

                CloseGoToFirstPickupLine(),
                ClosePickupAndShootFirstRow(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseGoToTZonePickupLine(),
                ClosePickupAndShootTZoneRow(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseGoToTZonePickupLine(),
                ClosePickupAndShootTZoneRow(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseMoveOffLineToLever()
        );
    }

    public Command Close24BallLeverAfter6() {
        return new SequentialGroup(
                CloseShootPreload(),

                CloseGoToFirstPickupLine(),
                ClosePickupAndShootFirstRow(),

                CloseGoTo2ndPickupLine(),
                ClosePickupAndGateLever2ndRow(),
                new Delay(.1),
                ClosePickupShootAfterGateLever2ndRow(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseGoToTZonePickupLine(),
                ClosePickupAndShootTZoneRow(),

                CloseGoToTZonePickupLine(),
                ClosePickupAndShootTZoneRow(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseGoToBZonePickupLine(),
                ClosePickupAndShootBZoneRow(),

                CloseMoveOffLineToLever()
        );
    }


    /** This method is called once at the start of the OhhpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void onStartButtonPressed() {
        TurretSubsystem.INSTANCE.forceStartupCalibrationFromExpected(TurretSubsystem.INITIAL_ANGLE_DEGREES);
        startAutonLogger();
        if (intAmount == 3) {
            Close3Ball().schedule();
        }
        else if (intAmount == 6){
            Close6Ball().schedule();
        }
        else if (intAmount == 9){
            Close9Ball().schedule();
        }
        else if (intAmount == 12){
            Close12Ball().schedule();
        }
        else if (intAmount == 15){
            Close15BallLeverAfter6().schedule();
        }
        else if (intAmount == 18){
            if (pushLever == 0) {
                Close18Ball().schedule();
            } else if (pushLever == 1){
                Close18BallNo2ndSpike().schedule();
            }  else {
                Close18BallLeverAfter6().schedule();
            }
        }
        else if (intAmount == 21){
            if (pushLever == 0) {
                Close21Ball().schedule();
            } else if (pushLever == 1){
                Close21BallNo2ndSpike().schedule();
            }  else {
                Close21BallLeverAfter6().schedule();
            }
        }
        else if (intAmount == 24){
            if (pushLever == 0) {
                Close24Ball().schedule();
            } else if (pushLever == 1){
                Close24BallNo2ndSpike().schedule();
            }  else {
                Close24BallLeverAfter6().schedule();
            }
        }

        // Persist ball count (and optionally pose) for TeleOp

        GlobalRobotData.endAutonBallCount = IntakeWithSensorsSubsystem.INSTANCE.getBallCount();
        GlobalRobotData.endAutonPose = currentGoodPose;
        GlobalRobotData.endAutonTurretAngleDegrees = turretOffset;
        GlobalRobotData.hasAutonRun = true;
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void onUpdate() {
        logAutonLoop();

        int balls = IntakeWithSensorsSubsystem.INSTANCE.getBallCount();
        if (balls >= 3) {
            LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.GREEN);
        } else if (balls == 2) {
            LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.YELLOW);
        } else if (balls == 1) {
            LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.ORANGE);
        } else {
            LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.RED);
        }
        // These loop the movements of the robot, these must be called continuously in order to work

        // Feedback to Driver Hub for debugging
        telemetry.addData("x", PedroComponent.follower().getPose().getX());
        telemetry.addData("y", PedroComponent.follower().getPose().getY());
        telemetry.addData("heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
        telemetry.addData("shooter 1 power", ShooterSubsystem.INSTANCE.getShooter1Power());
        telemetry.addData("shooter 2 power", ShooterSubsystem.INSTANCE.getShooter2Power());
        telemetry.update();
    }

    /** We shouldn't need this because everything should automatically disable **/
    @Override
    public void onStop() {
        //logAutonLoop();
        saveAutonLogger();
         //TurretSubsystem.INSTANCE.setTargetAngleFromRobotFrontRelativeDegrees(180);
        // Persist ball count (and optionally pose) for TeleOp
        ShooterSubsystem.INSTANCE.stop();
        GlobalRobotData.endAutonBallCount = IntakeWithSensorsSubsystem.INSTANCE.getBallCount();
        GlobalRobotData.endAutonPose = currentGoodPose;
        GlobalRobotData.endAutonTurretAngleDegrees = turretOffset;
        GlobalRobotData.hasAutonRun = true;
    }


}