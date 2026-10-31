package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalRobotData;
import org.firstinspires.ftc.teamcode.subsystems.FlowerIntakeSubsystem2026;
import org.firstinspires.ftc.teamcode.subsystems.IntakeWithSensorsSubsystem2026;
import org.firstinspires.ftc.teamcode.subsystems.LEDControlSubsystem2026;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem2026;
import org.firstinspires.ftc.teamcode.subsystems.ShotSequenceController2026;

import java.util.ArrayList;
import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/**
 * Minimal red-side autonomous scaffold for the current 2026 robot.
 *
 * <p>The generated Pedro poses and paths stay in this file. Replace the
 * clearly marked placeholder sections with output from the path visualizer.</p>
 */
@Configurable
@Autonomous(name = "Blue Side Front Of Hive", group = "Comp")
public class BlueSideFrontOfHive extends NextFTCOpMode {

    public static double SHOOTER_FLYWHEEL_RPM = 3200.0;
    public static double SHOOTER_SPINUP_SECONDS = 2.0;
    public static double FLOWERY_WAIT_SECONDS = 1.0;

    // -------------------------------------------------------------------------
    // PEDRO VISUALIZER POSES: paste the Pedro 3 PoseFactory declarations here.
    // The temporary compatibility helper below converts degrees for Pedro 2.
    // -------------------------------------------------------------------------
    //TODO TODO TODO
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(54.178, 7.7849, 90);
    private final Pose flowerNear = poseFactory.of(14, 45.633, 180);
    private final Pose flowerNearControl1 = poseFactory.of(48.1286, 20.3037, 0);
    private final Pose flowerNearControl2 = poseFactory.of(36.8512, 49.0427, 0);
    private final Pose flowerNearSegment1Start = poseFactory.of(14, 45.633, 90);
    private final Pose flowerNearSegment1End = poseFactory.of(14, 45.633, 100);
    private final Pose flowerNearSegment2Start = poseFactory.of(14, 45.633, 100);
    private final Pose flowerNearSegment2End = poseFactory.of(14, 45.633, 180);
    private final Pose flowerNearSegment3Heading = poseFactory.of(14, 45.633, 180);
    private final Pose farShooting = poseFactory.of(33.9576, 121.426, 305);
    private final Pose farShootingSegment1Start = poseFactory.of(33.9576, 121.426, 180);
    private final Pose farShootingSegment1End = poseFactory.of(33.9576, 121.426, 270);
    private final Pose farShootingSegment2Start = poseFactory.of(33.9576, 121.426, 270);
    private final Pose farShootingSegment2End = poseFactory.of(33.9576, 121.426, 305);
    private final Pose flowerfar = poseFactory.of(44.716, 128.8, 90);
    private final Pose flowerfarControl1 = poseFactory.of(46.8042, 101.17, 0);
    private final Pose flowerfarSegment1Start = poseFactory.of(44.716, 127.3024, 305);
    private final Pose flowerfarSegment1End = poseFactory.of(44.716, 127.3024, 90);
    private final Pose flowerfarSegment2Start = poseFactory.of(44.716, 127.3024, 90);
    private final Pose flowerfarSegment2End = poseFactory.of(44.716, 127.3024, 90);
    private final Pose farshooting2 = poseFactory.of(33.96, 121.43, -55);
    private final Pose farshooting2Control1 = poseFactory.of(43.4511, 114.4166, 0);
    private final Pose farshooting2Segment1Heading = poseFactory.of(33.96, 121.43, 90);
    private final Pose farshooting2Segment2Start = poseFactory.of(33.96, 121.43, 90);
    private final Pose farshooting2Segment2End = poseFactory.of(33.96, 121.43, 305);
    private final Pose parkingStart = poseFactory.of(33.96, 121.43, 305);
    private final Pose parking = poseFactory.of(11.1451, 95.3931, 0);
    private final Pose parkingControl1 = poseFactory.of(36.0595, 92.6635, 0);

    public Path flowerNear() {
        return curve(start, flowerNearControl1, flowerNearControl2, flowerNear).heading(Interpolator.piecewise().until(0.2095, Interpolator.linear(flowerNearSegment1Start, flowerNearSegment1End)).until(0.7896, Interpolator.linear(flowerNearSegment2Start, flowerNearSegment2End)).until(1, Interpolator.constant(flowerNearSegment3Heading)));
    }

    public Path farShooting() {
        return line(flowerNear, farShooting).heading(Interpolator.piecewise().until(0.184, Interpolator.linear(farShootingSegment1Start, farShootingSegment1End)).until(1, Interpolator.linear(farShootingSegment2Start, farShootingSegment2End)));
    }

    public Path flowerfar() {
        return curve(farShooting, flowerfarControl1, flowerfar).heading(Interpolator.piecewise().until(0.5, Interpolator.linear(flowerfarSegment1Start, flowerfarSegment1End)).until(1, Interpolator.linear(flowerfarSegment2Start, flowerfarSegment2End)));
    }

    public Path farshooting2() {
        return curve(flowerfar, farshooting2Control1, farshooting2).heading(Interpolator.piecewise().until(0.1646, Interpolator.constant(farshooting2Segment1Heading)).until(1, Interpolator.linear(farshooting2Segment2Start, farshooting2Segment2End)));
    }

    public Path parking() {
        return curve(parkingStart, parkingControl1, parking).linear(parkingStart, parking);
    }


//todo todo
    // -------------------------------------------------------------------------
    // PEDRO VISUALIZER PATH FIELDS: add generated Path/PathChain fields here.
    // -------------------------------------------------------------------------
    private PathChain pastedPath;
    private PathChain flowerNearPath;
    private PathChain farShootingPath;
    private PathChain parkingPath;
    private PathChain farshooting2Path;
    private PathChain flowerfarPath;

    private boolean autonStarted;
    private enum AutonStage {
        READY,
        SPIN_UP,
        SHOOT_PRELOADS,
        DRIVE_TO_FLOWER,
        COLLECT,
        DRIVE_TO_SHOOT,
        SHOOT,
        PARK,
        DONE
    }

    private AutonStage autonStage = AutonStage.READY;
    private long stageStartNanos;
    private final ShotSequenceController2026 shotSequence =
            new ShotSequenceController2026();

    public BlueSideFrontOfHive() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(
                        ShooterSubsystem2026.INSTANCE,
                        IntakeWithSensorsSubsystem2026.INSTANCE,
                        LEDControlSubsystem2026.INSTANCE,
                        FlowerIntakeSubsystem2026.INSTANCE
                ),
                BulkReadComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        autonStarted = false;
        autonStage = AutonStage.READY;
        stageStartNanos = 0L;
        LEDControlSubsystem2026.INSTANCE.clearStagePulse();
        GlobalRobotData.allianceSide = GlobalRobotData.COLOR.RED;

        // Clear any handoff left from an earlier run. This autonomous publishes
        // a fresh final pose only after Play has actually been pressed.
        GlobalRobotData.hasAutonRun = false;
        GlobalRobotData.endAutonPose = null;
        GlobalRobotData.endAutonBallCount = -1;

        ShooterSubsystem2026.INSTANCE.stop();
        IntakeWithSensorsSubsystem2026.INSTANCE.stopAll();
        FlowerIntakeSubsystem2026.INSTANCE.moveUp();
        shotSequence.reset();

        PedroComponent.follower().setStartingPose(start);
        buildPaths();
    }

    /**
     * Build paths in this file instead of maintaining a separate season path
     * class. Translate Pedro 3 visualizer output into the Pedro 2 pathBuilder
     * syntax shown here.
     */
    //TODO TODO TODO
    private void buildPaths() {
        // ---------------------------------------------------------------------
        // BEGIN PEDRO VISUALIZER BUILD-PATHS PASTE SECTION
        //
        // Add one addPath(...) call for each pasted visualizer path method.
        // ---------------------------------------------------------------------
        flowerNearPath = PedroComponent.follower().pathBuilder()
                .addPath(flowerNear())
                .build();

        farShootingPath = PedroComponent.follower().pathBuilder()
                .addPath(farShooting())
                .build();

        parkingPath = PedroComponent.follower().pathBuilder()
                .addPath(parking())
                .build();

        farshooting2Path = PedroComponent.follower().pathBuilder()
                .addPath(farshooting2())
                .build();

        flowerfarPath = PedroComponent.follower().pathBuilder()
                .addPath(flowerfar())
                .build();

        // END PEDRO VISUALIZER BUILD-PATHS PASTE SECTION
    }



    /**
     * Temporary Pedro 2 compatibility shim for pose declarations exported by
     * the Pedro 3 visualizer. When this project moves to Pedro 3, delete this
     * nested class and import com.pedropathing.api.PoseFactory instead.
     */
    private static final class PoseFactory {
        private static PoseFactory degrees() {
            return new PoseFactory();
        }

        private Pose of(double x, double y, double headingDegrees) {
            return new Pose(x, y, Math.toRadians(headingDegrees));
        }

        private Pose of(double x, double y) {
            return new Pose(x, y);
        }
    }

    /**
     * Temporary Pedro 2 compatibility wrapper for path methods exported by the
     * Pedro 3 visualizer. Delete this class and curve()/line() when upgrading.
     */
    private static final class CompatiblePath extends Path {
        private CompatiblePath(BezierCurve curve) {
            super(curve);
        }

        private CompatiblePath(BezierLine line) {
            super(line);
        }

        private CompatiblePath linear(Pose startPose, Pose endPose) {
            setLinearHeadingInterpolation(
                    startPose.getHeading(),
                    endPose.getHeading()
            );
            return this;
        }

        private CompatiblePath reverseTangent() {
            setTangentHeadingInterpolation();
            reverseHeadingInterpolation();
            return this;
        }

        private CompatiblePath heading(HeadingInterpolator interpolator) {
            setHeadingInterpolation(interpolator);
            return this;
        }

        private CompatiblePath heading(Interpolator.Piecewise piecewise) {
            return heading(piecewise.build());
        }
    }

    /**
     * Temporary Pedro 2 compatibility wrapper for the visualizer's Pedro 3
     * heading syntax. Each piecewise segment runs from the previous until()
     * value to its own, and its interpolator spans that segment's 0-1 range.
     * Delete this class when upgrading to Pedro 3.
     */
    private static final class Interpolator {
        private static HeadingInterpolator linear(Pose startPose, Pose endPose) {
            return HeadingInterpolator.linear(
                    startPose.getHeading(),
                    endPose.getHeading()
            );
        }

        private static HeadingInterpolator constant(Pose headingPose) {
            return HeadingInterpolator.constant(headingPose.getHeading());
        }

        private static HeadingInterpolator tangent() {
            return HeadingInterpolator.tangent;
        }

        private static Piecewise piecewise() {
            return new Piecewise();
        }

        private static final class Piecewise {
            private final List<HeadingInterpolator.PiecewiseNode> nodes =
                    new ArrayList<>();
            private double segmentStartT = 0.0;

            private Piecewise until(double segmentEndT, HeadingInterpolator interpolator) {
                nodes.add(new HeadingInterpolator.PiecewiseNode(
                        segmentStartT,
                        segmentEndT,
                        interpolator
                ));
                segmentStartT = segmentEndT;
                return this;
            }

            private HeadingInterpolator build() {
                return HeadingInterpolator.piecewise(
                        nodes.toArray(new HeadingInterpolator.PiecewiseNode[0])
                );
            }
        }
    }

    private static CompatiblePath curve(Pose... controlPoints) {
        return new CompatiblePath(new BezierCurve(controlPoints));
    }

    private static CompatiblePath line(Pose startPose, Pose endPose) {
        return new CompatiblePath(new BezierLine(startPose, endPose));
    }

    /**
     * This is the section to expand when the visualizer produces multiple path
     * chains or when mechanism commands need to run between paths.
     */
    private Command pathFollowingToFlowerNear() {
        // ---------------------------------------------------------------------
        // BEGIN PATH-FOLLOWING SEQUENCE SECTION
        //
        // For one generated PathChain, the default return is sufficient.
        // For multiple paths, return a SequentialGroup of FollowPath commands.
        // ---------------------------------------------------------------------

        return new FollowPath(flowerNearPath);
        // END PATH-FOLLOWING SEQUENCE SECTION
    }
    private Command pathFollowingCommandToFarShooting() {
        // ---------------------------------------------------------------------
        // BEGIN PATH-FOLLOWING SEQUENCE SECTION
        //
        // For one generated PathChain, the default return is sufficient.
        // For multiple paths, return a SequentialGroup of FollowPath commands.
        // ---------------------------------------------------------------------

        return new FollowPath(farShootingPath);
        // END PATH-FOLLOWING SEQUENCE SECTION
    }
    private Command pathFollowingToPark() {
        // ---------------------------------------------------------------------
        // BEGIN PATH-FOLLOWING SEQUENCE SECTION
        //
        // For one generated PathChain, the default return is sufficient.
        // For multiple paths, return a SequentialGroup of FollowPath commands.
        // ---------------------------------------------------------------------

        return new FollowPath(parkingPath);
        // END PATH-FOLLOWING SEQUENCE SECTION
    }

    private Command pathFollowingToFlowerFar() {
        // ---------------------------------------------------------------------
        // BEGIN PATH-FOLLOWING SEQUENCE SECTION
        //
        // For one generated PathChain, the default return is sufficient.
        // For multiple paths, return a SequentialGroup of FollowPath commands.
        // ---------------------------------------------------------------------

        return new FollowPath(flowerfarPath);
        // END PATH-FOLLOWING SEQUENCE SECTION
    }

    private Command pathFollowingToShootingFar2() {
        // ---------------------------------------------------------------------
        // BEGIN PATH-FOLLOWING SEQUENCE SECTION
        //
        // For one generated PathChain, the default return is sufficient.
        // For multiple paths, return a SequentialGroup of FollowPath commands.
        // ---------------------------------------------------------------------

        return new FollowPath(farShootingPath);
        // END PATH-FOLLOWING SEQUENCE SECTION
    }

    //TODO TODO TODO
    private Command autonomousSequence() {
        return new SequentialGroup(
                enterStage(AutonStage.SPIN_UP),
                new InstantCommand(() ->
                        ShooterSubsystem2026.INSTANCE.spinUp(
                                SHOOTER_FLYWHEEL_RPM
                        )
                ),
                new Delay( SHOOTER_SPINUP_SECONDS),
                enterStage(AutonStage.SHOOT_PRELOADS),
                new InstantCommand(shotSequence::start),
                new WaitUntil(() -> !shotSequence.isActive()),
                enterStage(AutonStage.DRIVE_TO_FLOWER),
                new ParallelGroup(
                    pathFollowingToFlowerNear(),
                    new InstantCommand(() ->
                        IntakeWithSensorsSubsystem2026.INSTANCE.intakeForward()),
                    new InstantCommand(() ->
                        FlowerIntakeSubsystem2026.INSTANCE.moveDown())
                ),
                enterStage(AutonStage.COLLECT),
                new Delay( FLOWERY_WAIT_SECONDS),
                enterStage(AutonStage.DRIVE_TO_SHOOT),
                new ParallelGroup(
                    pathFollowingCommandToFarShooting(),
                   new InstantCommand(() ->
                            FlowerIntakeSubsystem2026.INSTANCE.moveUp())
                ),
                enterStage(AutonStage.SHOOT),
                new InstantCommand(shotSequence::start),
                new WaitUntil(() -> !shotSequence.isActive()),

                enterStage(AutonStage.DRIVE_TO_FLOWER),
                new ParallelGroup(
                        pathFollowingToFlowerFar(),
                        new InstantCommand(() ->
                                IntakeWithSensorsSubsystem2026.INSTANCE.intakeForward()),
                        new InstantCommand(() ->
                                FlowerIntakeSubsystem2026.INSTANCE.moveDown())
                ),
                enterStage(AutonStage.COLLECT),
                new Delay( FLOWERY_WAIT_SECONDS),
                enterStage(AutonStage.DRIVE_TO_SHOOT),
                new ParallelGroup(
                        pathFollowingToShootingFar2(),
                        new InstantCommand(() ->
                                FlowerIntakeSubsystem2026.INSTANCE.moveUp())
                ),
                new Delay(0.75),
                enterStage(AutonStage.SHOOT),
                new InstantCommand(shotSequence::start),
                new WaitUntil(() -> !shotSequence.isActive()),


                enterStage(AutonStage.PARK),
                pathFollowingToPark(),
                new InstantCommand(() ->
                    ShooterSubsystem2026.INSTANCE.stop()),
                new InstantCommand(() ->
                        IntakeWithSensorsSubsystem2026.INSTANCE.stopIntake()),
                enterStage(AutonStage.DONE)

        );
    }

    private Command enterStage(AutonStage stage) {
        return new InstantCommand(() -> {
            // This indicates command progression, not verified physical arrival.
            autonStage = stage;
            stageStartNanos = System.nanoTime();
            LEDControlSubsystem2026.INSTANCE.signalStageChange();
        });
    }

    private void addStageTelemetry() {
        telemetry.addData("Auton stage", "%d / %d: %s",
                autonStage.ordinal(), AutonStage.DONE.ordinal(), autonStage.name());
        double elapsedSeconds = stageStartNanos == 0L
                ? 0.0 : (System.nanoTime() - stageStartNanos) / 1e9;
        telemetry.addData("Time in stage", "%.2f sec", elapsedSeconds);
        telemetry.addData("Stage LED pulse", LEDControlSubsystem2026.INSTANCE.isStagePulseActive());
    }

    @Override
    public void onWaitForStart() {
        Pose pose = PedroComponent.follower().getPose();
        addStageTelemetry();

        telemetry.addLine("Hello Pickle of the robot");
        telemetry.addLine("This is an Mr. Todone Speaking,");
        telemetry.addLine("----------------------------------------------");
        telemetry.addLine("Favorite fruit: Raspberries!!! (Red)");
        telemetry.addData("Path loaded", pastedPath != null ? "YES" : "NO - WILL NOT MOVE");
        telemetry.addData(
                "Shooter flywheel target",
                "%.0f RPM",
                SHOOTER_FLYWHEEL_RPM
        );
        telemetry.addData("Spin-up delay", "%.2f sec", SHOOTER_SPINUP_SECONDS);
        telemetry.addData(
                "Flower intake",
                FlowerIntakeSubsystem2026.INSTANCE.getPosition()
        );
        telemetry.addData(
                "Starting pose",
                "(%.2f, %.2f, %.1f deg)",
                pose.getX(),
                pose.getY(),
                Math.toDegrees(pose.getHeading())
        );
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        autonStarted = true;
        autonomousSequence().schedule();
    }

    @Override
    public void onUpdate() {
        // The shared controller owns feed timing and the timed shooter boost.
        shotSequence.update();

        Pose pose = PedroComponent.follower().getPose();

        addStageTelemetry();
        telemetry.addData("Path loaded", pastedPath != null);
        telemetry.addData("Follower busy", PedroComponent.follower().isBusy());
        telemetry.addData("X", "%.2f in", pose.getX());
        telemetry.addData("Y", "%.2f in", pose.getY());
        telemetry.addData("Heading", "%.1f deg", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Shot sequence active", shotSequence.isActive());
        telemetry.addData(
                "Shot sequence time",
                "%.2f / %.2f sec",
                shotSequence.getElapsedSeconds(),
                shotSequence.getActiveFeedSeconds()
        );
        telemetry.addData("Shot boost active", shotSequence.isBoostActive());
        telemetry.addData(
                "Flower intake",
                FlowerIntakeSubsystem2026.INSTANCE.getPosition()
        );
        telemetry.addData(
                "Shooter motor RPM",
                "target %.0f / measured %.0f",
                ShooterSubsystem2026.INSTANCE.getTargetMotorRpm(),
                ShooterSubsystem2026.INSTANCE.getMeasuredMotorRpm()
        );
        telemetry.addData(
                "Shooter flywheel RPM",
                "target %.0f / measured %.0f",
                ShooterSubsystem2026.INSTANCE.getTargetFlywheelRpm(),
                ShooterSubsystem2026.INSTANCE.getMeasuredFlywheelRpm()
        );
        telemetry.update();
    }

    @Override
    public void onStop() {
        int finalBallCount =
                IntakeWithSensorsSubsystem2026.INSTANCE.getBallCount();
        shotSequence.cancel();
        ShooterSubsystem2026.INSTANCE.stop();
        IntakeWithSensorsSubsystem2026.INSTANCE.stopIntake();
        LEDControlSubsystem2026.INSTANCE.clearStagePulse();

        if (autonStarted) {
            Pose finalPose = PedroComponent.follower().getPose();
            GlobalRobotData.allianceSide = GlobalRobotData.COLOR.RED;
            GlobalRobotData.endAutonPose = finalPose.copy();
            GlobalRobotData.endAutonBallCount = finalBallCount;
            GlobalRobotData.endAutonTurretAngleDegrees = Double.NaN;
            GlobalRobotData.endAutonTurretServoCommandAngleDegrees = Double.NaN;
            GlobalRobotData.endAutonTurretServoOffsetDegrees = Double.NaN;
            GlobalRobotData.hasAutonRun = true;
        }

        super.onStop();
    }
}
