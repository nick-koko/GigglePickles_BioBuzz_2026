package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalRobotData;
import org.firstinspires.ftc.teamcode.subsystems.IntakeWithSensorsSubsystem2026;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem2026;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
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
@Autonomous(name = "leav and park", group = "Comp")
public class leavandpark extends NextFTCOpMode {

    public static double SHOOTER_MOTOR_RPM = 4700.0;
    public static double SHOOTER_SPINUP_SECONDS = 2.0;

    // -------------------------------------------------------------------------
    // PEDRO VISUALIZER POSES: paste the Pedro 3 PoseFactory declarations here.
    // The temporary compatibility helper below converts degrees for Pedro 2.
    // -------------------------------------------------------------------------
    //TODO TODO TODO
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(60.4247, 7.8072, 90);
    private final Pose refrence = poseFactory.of(60.1567, 10.0208, 90);
    private final Pose path = poseFactory.of(12.6848, 106.1132, 90);
    private final Pose pathControl1 = poseFactory.of(18.2166, 18.4229, 0);

    public Path refrence() {
        return line(start, refrence).linear(start, refrence);
    }

    public Path path() {
        return curve(refrence, pathControl1, path).linear(path,path);
    }
    //todo todo
    // -------------------------------------------------------------------------
    // PEDRO VISUALIZER PATH FIELDS: add generated Path/PathChain fields here.
    // -------------------------------------------------------------------------
    private PathChain pastedPath;

    private boolean autonStarted;

    public leavandpark() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(
                        ShooterSubsystem2026.INSTANCE,
                        IntakeWithSensorsSubsystem2026.INSTANCE
                ),
                BulkReadComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        GlobalRobotData.allianceSide = GlobalRobotData.COLOR.RED;

        // Clear any handoff left from an earlier run. This autonomous publishes
        // a fresh final pose only after Play has actually been pressed.
        GlobalRobotData.hasAutonRun = false;
        GlobalRobotData.endAutonPose = null;
        GlobalRobotData.endAutonBallCount = -1;

        ShooterSubsystem2026.INSTANCE.stop();
        IntakeWithSensorsSubsystem2026.INSTANCE.stopAll();

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
        pastedPath = PedroComponent.follower().pathBuilder()
                .addPath(refrence())
                .addPath(path())
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
    private Command pathFollowingCommand() {
        // ---------------------------------------------------------------------
        // BEGIN PATH-FOLLOWING SEQUENCE SECTION
        //
        // For one generated PathChain, the default return is sufficient.
        // For multiple paths, return a SequentialGroup of FollowPath commands.
        // ---------------------------------------------------------------------
        if (pastedPath == null) {
            // Safe placeholder: spin up and wait, but do not move.
            return new InstantCommand(() -> {
            });
        }
        return new FollowPath(pastedPath);
        // END PATH-FOLLOWING SEQUENCE SECTION
    }

    //TODO TODO TODO
    private Command autonomousSequence() {
        return new SequentialGroup(
                new InstantCommand(() ->
                        ShooterSubsystem2026.INSTANCE.spinUp(SHOOTER_MOTOR_RPM)
                ),
                new Delay(Math.max(0.0, SHOOTER_SPINUP_SECONDS)),
                new InstantCommand(() ->
                            IntakeWithSensorsSubsystem2026.INSTANCE.setFeedingToShooter(true)
                ),
                new Delay(Math.max(0.0, SHOOTER_SPINUP_SECONDS)),
                new InstantCommand(() ->
                        IntakeWithSensorsSubsystem2026.INSTANCE.setFeedingToShooter(false)
                ),
                pathFollowingCommand()
        );
    }

    @Override
    public void onWaitForStart() {
        Pose pose = PedroComponent.follower().getPose();

        telemetry.addLine("Hello Pickle of the robot");
        telemetry.addLine("This is an Mr. Todone Speaking,");
        telemetry.addLine("----------------------------------------------");
        telemetry.addLine("Favorite fruit: Raspberries!!! (Red)");
        telemetry.addData("Path loaded", pastedPath != null ? "YES" : "NO - WILL NOT MOVE");
        telemetry.addData("Shooter motor target", "%.0f RPM", SHOOTER_MOTOR_RPM);
        telemetry.addData("Spin-up delay", "%.2f sec", SHOOTER_SPINUP_SECONDS);
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
        Pose pose = PedroComponent.follower().getPose();

        telemetry.addData("Path loaded", pastedPath != null);
        telemetry.addData("Follower busy", PedroComponent.follower().isBusy());
        telemetry.addData("X", "%.2f in", pose.getX());
        telemetry.addData("Y", "%.2f in", pose.getY());
        telemetry.addData("Heading", "%.1f deg", Math.toDegrees(pose.getHeading()));
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
        ShooterSubsystem2026.INSTANCE.stop();
        IntakeWithSensorsSubsystem2026.INSTANCE.stopAll();

        if (autonStarted) {
            Pose finalPose = PedroComponent.follower().getPose();
            GlobalRobotData.allianceSide = GlobalRobotData.COLOR.RED;
            GlobalRobotData.endAutonPose = finalPose.copy();
            GlobalRobotData.endAutonBallCount = -1;
            GlobalRobotData.endAutonTurretAngleDegrees = Double.NaN;
            GlobalRobotData.endAutonTurretServoCommandAngleDegrees = Double.NaN;
            GlobalRobotData.endAutonTurretServoOffsetDegrees = Double.NaN;
            GlobalRobotData.hasAutonRun = true;
        }

        super.onStop();
    }
}
