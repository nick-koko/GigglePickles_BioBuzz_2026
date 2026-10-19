package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.GlobalRobotData;
import org.firstinspires.ftc.teamcode.subsystems.IntakeWithSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;

/* notes:
    - Start up shooter immediately
    - (save .75) right now, first shot at 2.75 s, thinking we can reduce to 2.0s?
    - (save 1.0) first set of shots done at about 3.35s, we don't start moving until 4.4s, so can either cut a second off here, or maybe use break beam sensor as an indication that we are done?
    - (save 1.5) we are at back at the shooting point at about 8.0s, but don't shoot until about 9.48s, assuming we start the shooter when driving, we could shoot earlier - maybe the timers are off, or maybe need to stop path earlier?
    - (save 1.1) done shooting at 9.82s, don't start moving until 11.0s
    - (save ??) path to go back to shooter is wide (a bit on purpose to avoid gate) not needed if we are hitting the gate already, and can save time by going straight
    - (save 0.7) back to shooting point about 16.1s, don't shoot until 16.9s
    - (save 1.2) done shooting at 17.25s, don't start moving until 18.49s
    - (??) maybe can take a less wide path to the last spike mark to possibly avoid robots that are just driving off the line?
    - (save 1.0) back to shooting spot at 24.25, don't shoot until 25.25
    - (save 1.0) done shooting at 25.62s, don't move until 26.7s
    - I wonder after, if we should have the robot turned 180deg, and ready to intake as soon as teleop starts and they hit the gate?
    - after we are done, watch time says 2s left, but match timer says 3
    - (total possible perfect savings of 8.25s for a total of 10.25 ) Need minimum 7s

 */
@Disabled
@Configurable
@Autonomous(name = "WorldsTopTriangle_closeBlueSide", group = "Comp")
public class closeBlueSide_WorldsTopTriangle extends closeAutonPaths_WorldsTopTriangle{
    public closeBlueSide_WorldsTopTriangle() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(ShooterSubsystem.INSTANCE, IntakeWithSensorsSubsystem.INSTANCE, TurretSubsystem.INSTANCE)
        );
    }
    public double intAmount = 18;
    public double pushLever = 3;

    private Pose finalStartPose = new Pose();

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void onInit() {
        TurretSubsystem.INSTANCE.beginStartupCentering();
        ShooterSubsystem.INSTANCE.shooterHoodDrive(nearAutonShooterHoodServoPos);
        ShooterSubsystem.INSTANCE.stop();

        GlobalRobotData.allianceSide = GlobalRobotData.COLOR.BLUE;
        PedroComponent.follower().setStartingPose(startPoseBlue);

        // Seed ball count for auton: assume robot starts loaded with 3
        IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3);

        buildPaths();
    }

    private void updateGateIntakeWaitDelayForSelection() {
        activeGateIntakeWaitDelay = (intAmount == 21) ? gateIntakeShortWaitDelay : gateIntakeWaitDelay;
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
        if ((gamepad1.dpadUpWasPressed()) && (intAmount < 21)) {
            intAmount = intAmount + 3;
        } else if ((gamepad1.dpadDownWasPressed()) && (intAmount > 18)) {
            intAmount = intAmount - 3;
        }
        // If dpad left/right is pressed add or subtract a row until push lever
        if ((gamepad1.dpadRightWasPressed()) && (pushLever < 4)) {
            pushLever = pushLever + 1;
        } else if ((gamepad1.dpadLeftWasPressed()) && (pushLever > 2)) {
            pushLever = pushLever - 1;
        }

        //TODO Add dpad Left/Right to set when to hit gate lever (after 1st pickup, second pickup, or both)
        updateGateIntakeWaitDelayForSelection();

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
        telemetry.addData("Gate intake wait delay: ", activeGateIntakeWaitDelay);

        telemetry.addData("heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
        telemetry.addData("turretStartupCal", turretStartupCalibrated);
        telemetry.addData("turretStartupState", TurretSubsystem.INSTANCE.getStartupCalibrationStateName());

        telemetry.update();

    }


    public Command Close3Ball() {
        return new SequentialGroup(
                CloseShootMovingPreload()
                //CloseMoveOffLine()
        );
    }

    public Command Close6Ball() {
        return new SequentialGroup(
                CloseShootMovingPreload(),
                CloseGoTo2ndPickupLine(),
                ClosePickupAndShoot2ndRow()
                //CloseMoveOffLine()
        );

    }
    public Command Close18Ball3Gate() {
        return new SequentialGroup(
                CloseShootPreload(),
                ClosePickupAndShoot2ndRowRace(),
                ClosePickupAndShootGateRace(),
                ClosePickupAndShootGateRace(),
                ClosePickupAndShootGateBefore1stRowRace(),

                //CloseGoTo3rdPickupLine(),
                //ClosePickupAndShoot3rdRowRace(),
                ClosePickupAndShootFirstRowRace()

                //CloseMoveOffLine() //If we want to end at lever instead
        );
    }
    public Command Close18Ball3GateExtra() {
        return new SequentialGroup(
                CloseShootPreload(),
                //ClosePickupAndShoot2ndRowRace(),
                ClosePickupAndShoot2ndRowRaceAndGate(),
                ClosePickupAndShootGateRace(),
                ClosePickupAndShootGateRace(),
                ClosePickupAndShootGateBefore1stRowRace(),

                //CloseGoTo3rdPickupLine(),
                //ClosePickupAndShoot3rdRowRace(),
                ClosePickupAndShootFirstRowRace()

                //CloseMoveOffLine() //If we want to end at lever instead
        );
    }


    public Command Close18Ball2GateThirdSpike() {
        return new SequentialGroup(
                CloseShootPreload(),
                ClosePickupAndShoot2ndRowRace(),
                ClosePickupAndShootGateRace(),
                ClosePickupAndShootGateRace(),
                //ClosePickupAndShootGateBefore1stRowRace(),

                ClosePickupAndShoot3rdRowBefore1stRowRace(),
                ClosePickupAndShootFirstRowRace()

                //CloseMoveOffLine() //If we want to end at lever instead
        );
    }

    public Command Close21Ball4Gate() {
        return new SequentialGroup(
                CloseShootPreload(),
                ClosePickupAndShoot2ndRowRace(),
                ClosePickupAndShootGateRace(),
                ClosePickupAndShootGateRace(),
                ClosePickupAndShootGateRace(),
                ClosePickupAndShootGateBefore1stRowRace(),

                //CloseGoTo3rdPickupLine(),
                //ClosePickupAndShoot3rdRowRace(),
                ClosePickupAndShootFirstRowRace()

                //CloseMoveOffLine() //If we want to end at lever instead
        );
    }

    public Command Close21Ball3GateThirdSpike() {
        return new SequentialGroup(
                CloseShootPreload(),
                ClosePickupAndShoot2ndRowRace(),
                ClosePickupAndShootGateRace(),
                ClosePickupAndShootGateRace(),
                ClosePickupAndShootGateRace(),
                //ClosePickupAndShootGateBefore1stRowRace(),

                ClosePickupAndShoot3rdRowBefore1stRowRace(),
                ClosePickupAndShootFirstRowRace()

                //CloseMoveOffLine() //If we want to end at lever instead
        );
    }

    public Command Close9Ball() {
        return new SequentialGroup(
                CloseShootPreload(),
                CloseGoToFirstPickupLine(),
                ClosePickupAndShootFirstRowRace(),
                CloseGoTo2ndPickupLine(),
                ClosePickupAndShoot2ndRow(),
                CloseMoveOffLineToLever()
        );

    }
    public Command Close12Ball() {
        return new SequentialGroup(
                CloseShootPreload(),
                CloseGoToFirstPickupLine(),
                ClosePickupAndShootFirstRowRace(),
                CloseGoTo2ndPickupLine(),
                ClosePickupAndShoot2ndRow(),
                CloseGoTo3rdPickupLine(),
                ClosePickupAndShoot3rdRowRace(),
                CloseMoveOffLineToLever()
        );
    }

    public Command Close12BallLeverAfter3() {
        return new SequentialGroup(
                CloseShootPreload(),
                CloseGoToFirstPickupLine(),
                ClosePickupAndGateLeverFirstRow(),
                ClosePickupShootAfterGateLever1stRow(),
                CloseGoTo2ndPickupLine(),
                ClosePickupAndShoot2ndRow(),
                CloseGoTo3rdPickupLine(),
                ClosePickupAndShoot3rdRowRace(),
                CloseMoveOffLineToLever()
        );
    }

    public Command Close12BallLeverAfter6() {
        return new SequentialGroup(
                CloseShootPreload(),
                CloseGoToFirstPickupLine(),
                ClosePickupAndShootFirstRowRace(),
                CloseGoTo2ndPickupLine(),
                ClosePickupAndGateLever2ndRow(),
                new Delay(.1),
                ClosePickupShootAfterGateLever2ndRow(),
                CloseGoTo3rdPickupLine(),
                ClosePickupAndShoot3rdRowRace(),
                CloseMoveOffLineToLever()
        );
    }

    public Command Close12BallLeverBoth() {
        return new SequentialGroup(
                CloseShootPreload(),
                CloseGoToFirstPickupLine(),
                ClosePickupAndGateLeverFirstRow(),
                ClosePickupShootAfterGateLever1stRow(),
                CloseGoTo2ndPickupLine(),
                ClosePickupAndGateLever2ndRow(),
                ClosePickupShootAfterGateLever2ndRow(),
                CloseGoTo3rdPickupLine(),
                ClosePickupAndShoot3rdRowRace(),
                CloseMoveOffLineToLever()
        );
    }

    public Command Close15Ball() {
        return new SequentialGroup(
                CloseShootPreload(),
                CloseGoToFirstPickupLine(),
                ClosePickupAndShootFirstRowRace(),
                CloseGoTo2ndPickupLine(),
                ClosePickupAndShoot2ndRow(),
                CloseGoTo3rdPickupLine(),
                ClosePickupAndShoot3rdRowRace(),
                CloseGoToZonePickupLine(),
                FollowZonePickupEndUntilFull(),
                CloseShootZoneRow(),
                CloseMoveOffLineToLever()
        );
    }

    public Command Close15BallLeverAfter3() {
        return new SequentialGroup(
                CloseShootPreload(),
                CloseGoToFirstPickupLine(),
                ClosePickupAndGateLeverFirstRow(),
                ClosePickupShootAfterGateLever1stRow(),
                CloseGoTo2ndPickupLine(),
                ClosePickupAndShoot2ndRow(),
                CloseGoTo3rdPickupLine(),
                ClosePickupAndShoot3rdRowRace(),
                CloseGoToZonePickupLine(),
                FollowZonePickupEndUntilFull(),
                CloseShootZoneRow(),
                CloseMoveOffLineToLever()
        );
    }

    public Command Close15BallLeverAfter6() {
        return new SequentialGroup(
                CloseShootPreload(),
                CloseGoToFirstPickupLine(),
                ClosePickupAndShootFirstRowRace(),
                CloseGoTo2ndPickupLine(),
                ClosePickupAndGateLever2ndRow(),
                new Delay(.1),
                ClosePickupShootAfterGateLever2ndRow(),
                CloseGoTo3rdPickupLine(),
                ClosePickupAndShoot3rdRowRace(),
                CloseGoToZonePickupLine(),
                FollowZonePickupEndUntilFull(),
                CloseShootZoneRow(),
                CloseMoveOffLineToLever()
        );
    }

    public Command Close15BallLeverBoth() {
        return new SequentialGroup(
                CloseShootPreload(),
                CloseGoToFirstPickupLine(),
                ClosePickupAndGateLeverFirstRow(),
                new Delay(.5),
                ClosePickupShootAfterGateLever1stRow(),
                CloseGoTo2ndPickupLine(),
                ClosePickupAndGateLever2ndRow(),
                new Delay(1.5),
                ClosePickupShootAfterGateLever2ndRow(),
                CloseGoTo3rdPickupLine(),
                ClosePickupAndShootAfter3rdRowWithDelay(),
                CloseGoToZonePickupLine(),
                FollowZonePickupEndUntilFull(),
                CloseShootZoneRow(),
                CloseMoveOffLineToLever()
        );
    }

    public Command Close15BallLeverTriple() {
        return new SequentialGroup(
                CloseShootPreload(),
                CloseGoToFirstPickupLine(),
                ClosePickupAndGateLeverFirstRow(),
                new Delay(firstGatePushDelay),
                ClosePickupShootAfterGateLever1stRow(),
                CloseGoTo2ndPickupLine(),
                ClosePickupAndGateLever2ndRow(),
                new Delay(secondGatePushDelay),
                ClosePickupShootAfterGateLever2ndRow(),

                CloseGoToGateLeverBefore3rdPickup(),
                new Delay(thirdGatePushDelay),
                CloseGoTo3rdPickupAfterGateLever(),
                ClosePickupAndShoot3rdRowRace(),
                CloseMoveOffLineToLeverExtraTime()

                /*CloseGoToZonePickupLine(),
                FollowZonePickupEndUntilFull(),
                CloseShootZoneRow(),
                CloseMoveOffLineToLever() */
        );
    }

    /** This method is called once at the start of the OhhpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void onStartButtonPressed() {
        TurretSubsystem.INSTANCE.forceStartupCalibrationFromExpected(TurretSubsystem.INITIAL_ANGLE_DEGREES);
        updateGateIntakeWaitDelayForSelection();
        startAutonLogger();
        if (intAmount == 12){
                if (pushLever == 0) {
                    Close12Ball().schedule();
                } else if (pushLever == 1){
                    Close12BallLeverAfter3().schedule();
                }  else if (pushLever == 2){
                    Close12BallLeverAfter6().schedule();
                }  else if (pushLever == 3){
                    Close12BallLeverBoth().schedule();
                }
            }
            else if (intAmount == 15) {
                if (pushLever == 0) {
                    Close15Ball().schedule();
                } else if (pushLever == 1){
                    Close15BallLeverAfter3().schedule();
                }  else if (pushLever == 2){
                    Close15BallLeverAfter6().schedule();
                }  else if (pushLever == 3){
                    Close18Ball3Gate().schedule();
                }  else if (pushLever == 4){
                    Close15BallLeverTriple().schedule();
                }
            }
            else if (intAmount == 18) {
                if (pushLever == 0) {
                    Close18Ball2GateThirdSpike().schedule();
                } else if (pushLever == 1) {
                    Close18Ball2GateThirdSpike().schedule();
                } else if (pushLever == 2) {
                    //Close18Ball2GateThirdSpike().schedule();
                    Close18Ball3GateExtra().schedule();
                } else if (pushLever == 3) {
                    Close18Ball3Gate().schedule();
                } else if (pushLever == 4) {
                    Close18Ball3Gate().schedule();
                }
            }
            else if (intAmount == 21) {
                if (pushLever == 0) {
                    Close21Ball3GateThirdSpike().schedule();
                } else if (pushLever == 1) {
                    Close21Ball3GateThirdSpike().schedule();
                } else if (pushLever == 2) {
                    Close21Ball3GateThirdSpike().schedule();
                } else if (pushLever == 3) {
                    Close21Ball3GateThirdSpike().schedule();
                } else if (pushLever == 4) {
                    Close21Ball4Gate().schedule();
                }
            }


        // Persist ball count (and optionally pose) for TeleOp
        GlobalRobotData.endAutonBallCount = IntakeWithSensorsSubsystem.INSTANCE.getBallCount();
        GlobalRobotData.endAutonPose = currentGoodPose;
        GlobalRobotData.endAutonTurretAngleDegrees = turretOffset;
        GlobalRobotData.endAutonTurretServoCommandAngleDegrees = TurretSubsystem.INSTANCE.getServoCommandAngleDegrees();
        GlobalRobotData.hasAutonRun = true;
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
        @Override
        public void onUpdate() {
            logAutonLoop();

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
            // Persist ball count (and optionally pose) for TeleOp
            ShooterSubsystem.INSTANCE.stop();
            GlobalRobotData.endAutonBallCount = IntakeWithSensorsSubsystem.INSTANCE.getBallCount();
            GlobalRobotData.endAutonPose = currentGoodPose;
            GlobalRobotData.endAutonTurretAngleDegrees = turretOffset;
            GlobalRobotData.endAutonTurretServoCommandAngleDegrees = TurretSubsystem.INSTANCE.getServoCommandAngleDegrees();
            GlobalRobotData.hasAutonRun = true;
        }


    }

