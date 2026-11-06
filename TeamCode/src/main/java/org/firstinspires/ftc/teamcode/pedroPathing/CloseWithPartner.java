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
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem2026;
import org.firstinspires.ftc.teamcode.subsystems.ShotSequenceController2026;

import java.util.ArrayList;
import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
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
 * Close-side autonomous for matches with a partner.
 *
 * <p>Init selection on gamepad 1:</p>
 * <ul>
 *     <li>Dpad up: help shoot far. Close preloads, near flower, far shot,
 *     garden, close shot, park.</li>
 *     <li>Dpad down: trust partner. Close preloads, garden, close shot after
 *     the selected delay since the preloads were shot, near flower, close
 *     shot (skipped if it is too late), park.</li>
 *     <li>Dpad left/right: partner delay, 1 second per press.</li>
 * </ul>
 */
@Configurable
@Autonomous(name = "Start Close With Partner", group = "Comp")
public class CloseWithPartner extends NextFTCOpMode {

    public static double SHOOTER_FLYWHEEL_RPM_High = 3200.0;
    public static double SHOOTER_FLYWHEEL_RPM_Low = 3150.0;
    public static double SHOOTER_SPINUP_SECONDS = 2.0;
    public static double FLOWERY_WAIT_SECONDS = 1.0;
    public static double GARDEN_SLOW_START = 0.75;
    public static double GARDEN_SLOW_POWER = 0.40;

    public static double GARDEN_INTAKE_WAIT_SECONDS = 1.0;
    public static double GARDEN_RETRY_WAIT_SECONDS = 0.5;
    public static double GARDEN_BACKUP_POWER = 0.5;
    public static double GARDEN_BACKUP_TIMEOUT_SECONDS = 1.0;
    public static double GARDEN_RETURN_TIMEOUT_SECONDS = 1.5;

    public static double DEFAULT_PARTNER_DELAY_SECONDS = 10.0;
    public static double PARTNER_DELAY_STEP_SECONDS = 1.0;
    public static double MAX_PARTNER_DELAY_SECONDS = 25.0;

    // After the near flower pickup (trust partner), the final close shot is
    // only attempted if fewer than this many seconds have passed since Start.
    public static double LATEST_FINAL_CLOSE_SHOT_START_SECONDS = 21.0;

    private static final int FULL_BALL_COUNT = 4;

    // -------------------------------------------------------------------------
    // PEDRO VISUALIZER POSES
    // -------------------------------------------------------------------------
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(54.178, 7.7849, 90);
    private final Pose closeShoot1 = poseFactory.of(54.178, 11, 86);
    private final Pose flowerNear = poseFactory.of(14, 45.633, 180);
    private final Pose flowerNearControl1 = poseFactory.of(48.1286, 20.3037, 0);
    private final Pose flowerNearControl2 = poseFactory.of(36.8512, 49.0427, 0);
    private final Pose flowerNearSegment1Start = poseFactory.of(14, 45.633, 86);
    private final Pose flowerNearSegment1End = poseFactory.of(14, 45.633, 100);
    private final Pose flowerNearSegment2Start = poseFactory.of(14, 45.633, 100);
    private final Pose flowerNearSegment2End = poseFactory.of(14, 45.633, 180);
    private final Pose flowerNearSegment3Heading = poseFactory.of(14, 45.633, 180);
    private final Pose farShooting = poseFactory.of(35, 120, 305);
    private final Pose farShootingSegment1Start = poseFactory.of(35, 120, 180);
    private final Pose farShootingSegment1End = poseFactory.of(35, 120, 270);
    private final Pose farShootingSegment2Start = poseFactory.of(35, 120, 270);
    private final Pose farShootingSegment2End = poseFactory.of(35, 120, 305);
    private final Pose gardenintakeFromFar = poseFactory.of(8, 6, 180);
    private final Pose gardenintakeFromFarControl1 = poseFactory.of(0, 66.6066, 0);
    private final Pose gardenintakeFromFarControl2 = poseFactory.of(75.8361, 26.8206, 0);
    private final Pose gardenintakeFromFarSegment1Start = poseFactory.of(8, 6, 305);
    private final Pose gardenintakeFromFarSegment1End = poseFactory.of(8, 6, 270);
    private final Pose gardenintakeFromFarSegment2Start = poseFactory.of(8, 6, 270);
    private final Pose gardenintakeFromFarSegment2End = poseFactory.of(8, 6, 220);
    private final Pose gardenintakeFromFarSegment3Start = poseFactory.of(8, 6, 220);
    private final Pose gardenintakeFromFarSegment3End = poseFactory.of(8, 6, 220);
    private final Pose gardenintakeFromFarSegment4Start = poseFactory.of(8, 6, 220);
    private final Pose gardenintakeFromFarSegment4End = poseFactory.of(8, 6, 180);
    private final Pose gardenintakeFromClose = poseFactory.of(8, 6, 180);
    private final Pose gardenintakeFromCloseControl1 = poseFactory.of(40.4476, 37.0864, 0);
    private final Pose gardenintakeFromCloseControl2 = poseFactory.of(36.1545, 13.3795, 0);
    private final Pose gardenintakeFromCloseSegment1Start = poseFactory.of(8, 6, 86);
    private final Pose gardenintakeFromCloseSegment1End = poseFactory.of(8, 6, 220);
    private final Pose gardenintakeFromCloseSegment2Start = poseFactory.of(8, 6, 220);
    private final Pose gardenintakeFromCloseSegment2End = poseFactory.of(8, 6, 220);
    private final Pose gardenintakeFromCloseSegment3Start = poseFactory.of(8, 6, 220);
    private final Pose gardenintakeFromCloseSegment3End = poseFactory.of(8, 6, 180);
    private final Pose gardenBackup = poseFactory.of(14, 6, 180);
    private final Pose gardenReturn = poseFactory.of(8, 6, 180);
    private final Pose closeShoot2Start = poseFactory.of(8, 6, 180);
    private final Pose closeShoot2 = poseFactory.of(42.924, 14.3419, 67);
    private final Pose flowerNearFromClose = poseFactory.of(14, 45.633, 180);
    private final Pose flowerNearFromCloseControl1 = poseFactory.of(48.1286, 20.3037, 0);
    private final Pose flowerNearFromCloseControl2 = poseFactory.of(36.8512, 49.0427, 0);
    private final Pose flowerNearFromCloseSegment1Start = poseFactory.of(14, 45.633, 67);
    private final Pose flowerNearFromCloseSegment1End = poseFactory.of(14, 45.633, 100);
    private final Pose flowerNearFromCloseSegment2Start = poseFactory.of(14, 45.633, 100);
    private final Pose flowerNearFromCloseSegment2End = poseFactory.of(14, 45.633, 180);
    private final Pose flowerNearFromCloseSegment3Heading = poseFactory.of(14, 45.633, 180);
    private final Pose closeShoot3FromFlowerNearStart = poseFactory.of(14, 45.633, 180);
    private final Pose closeShoot3FromFlowerNear = poseFactory.of(42.924, 14.3419, 67);
    private final Pose closeShoot3FromFlowerNearControl1 = poseFactory.of(36.8512, 30, 0);
    private final Pose parkingStart = poseFactory.of(42.924, 14.3419, 67);
    private final Pose parking = poseFactory.of(13, 95.3931, 0);
    private final Pose parkingControl1 = poseFactory.of(8.1788, 74.5683, 0);
    private final Pose parkingFromFlowerNearStart = poseFactory.of(14, 45.633, 180);
    private final Pose parkingFromFlowerNear = poseFactory.of(13, 95.3931, 0);

    public Path closeShoot1() {
        return Paths.line(start, closeShoot1).linear(start, closeShoot1);
    }

    public Path flowerNear() {
        return Paths.curve(closeShoot1, flowerNearControl1, flowerNearControl2, flowerNear).heading(Interpolator.piecewise().until(0.2095, Interpolator.linear(flowerNearSegment1Start, flowerNearSegment1End)).until(0.7896, Interpolator.linear(flowerNearSegment2Start, flowerNearSegment2End)).until(1, Interpolator.constant(flowerNearSegment3Heading)));
    }

    public Path farShooting() {
        return Paths.line(flowerNear, farShooting).heading(Interpolator.piecewise().until(0.184, Interpolator.linear(farShootingSegment1Start, farShootingSegment1End)).until(1, Interpolator.linear(farShootingSegment2Start, farShootingSegment2End)));
    }

    public Path gardenintakeFromFar() {
        return Paths.curve(farShooting, gardenintakeFromFarControl1, gardenintakeFromFarControl2, gardenintakeFromFar).heading(Interpolator.piecewise().until(0.5, Interpolator.linear(gardenintakeFromFarSegment1Start, gardenintakeFromFarSegment1End)).until(0.75, Interpolator.linear(gardenintakeFromFarSegment2Start, gardenintakeFromFarSegment2End)).until(0.875, Interpolator.linear(gardenintakeFromFarSegment3Start, gardenintakeFromFarSegment3End)).until(1, Interpolator.linear(gardenintakeFromFarSegment4Start, gardenintakeFromFarSegment4End)));
    }

    public Path gardenintakeFromClose() {
        return Paths.curve(closeShoot1, gardenintakeFromCloseControl1, gardenintakeFromCloseControl2, gardenintakeFromClose).heading(Interpolator.piecewise().until(0.5, Interpolator.linear(gardenintakeFromCloseSegment1Start, gardenintakeFromCloseSegment1End)).until(0.875, Interpolator.linear(gardenintakeFromCloseSegment2Start, gardenintakeFromCloseSegment2End)).until(1, Interpolator.linear(gardenintakeFromCloseSegment3Start, gardenintakeFromCloseSegment3End)));
    }

    public Path gardenBackup() {
        return Paths.line(gardenintakeFromClose, gardenBackup).constant(gardenBackup);
    }

    public Path gardenReturn() {
        return Paths.line(gardenBackup, gardenReturn).constant(gardenReturn);
    }

    public Path closeShoot2() {
        return Paths.line(closeShoot2Start, closeShoot2).linear(closeShoot2Start, closeShoot2);
    }

    public Path flowerNearFromClose() {
        return Paths.curve(closeShoot2, flowerNearFromCloseControl1, flowerNearFromCloseControl2, flowerNearFromClose).heading(Interpolator.piecewise().until(0.2095, Interpolator.linear(flowerNearFromCloseSegment1Start, flowerNearFromCloseSegment1End)).until(0.7896, Interpolator.linear(flowerNearFromCloseSegment2Start, flowerNearFromCloseSegment2End)).until(1, Interpolator.constant(flowerNearFromCloseSegment3Heading)));
    }

    public Path closeShoot3FromFlowerNear() {
        return Paths.curve(closeShoot3FromFlowerNearStart, closeShoot3FromFlowerNearControl1, closeShoot3FromFlowerNear).linear(closeShoot3FromFlowerNearStart, closeShoot3FromFlowerNear);
    }

    public Path parking() {
        return Paths.curve(parkingStart, parkingControl1, parking).linear(parkingStart, parking);
    }

    public Path parkingFromFlowerNear() {
        return Paths.line(parkingFromFlowerNearStart, parkingFromFlowerNear).linear(parkingFromFlowerNearStart, parkingFromFlowerNear);
    }

    // -------------------------------------------------------------------------
    // PEDRO VISUALIZER PATH FIELDS
    // -------------------------------------------------------------------------
    private PathChain closeshootPath;
    private PathChain flowerNearPath;
    private PathChain farShootingPath;
    private PathChain gardenintakeFromFarPath;
    private PathChain gardenintakeFromClosePath;
    private PathChain gardenBackupPath;
    private PathChain gardenReturnPath;
    private PathChain closeshoot2Path;
    private PathChain flowerNearFromClosePath;
    private PathChain closeShoot3FromFlowerNearPath;
    private PathChain parkingPath;
    private PathChain parkingFromFlowerNearPath;

    private boolean autonStarted;
    private boolean helpShootFar;
    private double partnerDelaySeconds;
    private boolean previousDpadLeft;
    private boolean previousDpadRight;
    private long firstVolleyDoneNanos;
    private long startPressedNanos;

    private enum AutonStage {
        READY,
        SPIN_UP,
        SHOOT_PRELOADS,
        DRIVE_TO_FLOWER,
        COLLECT,
        DRIVE_TO_GARDEN,
        GARDEN_RETRY,
        DRIVE_TO_SHOOT,
        WAIT_FOR_PARTNER,
        SHOOT,
        PARK,
        DONE
    }

    private AutonStage autonStage = AutonStage.READY;
    private long stageStartNanos;
    private final ShotSequenceController2026 shotSequence =
            new ShotSequenceController2026();

    public CloseWithPartner() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(
                        ShooterSubsystem2026.INSTANCE,
                        IntakeWithSensorsSubsystem2026.INSTANCE,
                        LEDControlSubsystem2026.INSTANCE,
                        FlowerIntakeSubsystem2026.INSTANCE,
                        TurretSubsystem2026.INSTANCE
                ),
                BulkReadComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        TurretSubsystem2026.INSTANCE.prepareAutonomous();
        autonStarted = false;
        helpShootFar = true;
        partnerDelaySeconds = DEFAULT_PARTNER_DELAY_SECONDS;
        previousDpadLeft = false;
        previousDpadRight = false;
        firstVolleyDoneNanos = 0L;
        startPressedNanos = 0L;
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

    private void buildPaths() {
        closeshootPath = PedroComponent.follower().pathBuilder()
                .addPath(closeShoot1())
                .build();

        flowerNearPath = PedroComponent.follower().pathBuilder()
                .addPath(flowerNear())
                .build();

        farShootingPath = PedroComponent.follower().pathBuilder()
                .addPath(farShooting())
                .build();

        gardenintakeFromFarPath = PedroComponent.follower().pathBuilder()
                .addPath(gardenintakeFromFar())
                .addParametricCallback(GARDEN_SLOW_START, () ->
                        PedroComponent.follower().setMaxPowerScaling(GARDEN_SLOW_POWER))
                .build();

        gardenintakeFromClosePath = PedroComponent.follower().pathBuilder()
                .addPath(gardenintakeFromClose())
                .addParametricCallback(GARDEN_SLOW_START, () ->
                        PedroComponent.follower().setMaxPowerScaling(GARDEN_SLOW_POWER))
                .build();

        gardenBackupPath = PedroComponent.follower().pathBuilder()
                .addPath(gardenBackup())
                .build();

        gardenReturnPath = PedroComponent.follower().pathBuilder()
                .addPath(gardenReturn())
                .build();

        closeshoot2Path = PedroComponent.follower().pathBuilder()
                .addPath(closeShoot2())
                .build();

        flowerNearFromClosePath = PedroComponent.follower().pathBuilder()
                .addPath(flowerNearFromClose())
                .build();

        closeShoot3FromFlowerNearPath = PedroComponent.follower().pathBuilder()
                .addPath(closeShoot3FromFlowerNear())
                .build();

        parkingPath = PedroComponent.follower().pathBuilder()
                .addPath(parking())
                .build();

        parkingFromFlowerNearPath = PedroComponent.follower().pathBuilder()
                .addPath(parkingFromFlowerNear())
                .build();
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

        private CompatiblePath tangent() {
            setTangentHeadingInterpolation();
            return this;
        }

        private CompatiblePath constant(Pose headingPose) {
            setConstantHeadingInterpolation(headingPose.getHeading());
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
        // The visualizer writes both Interpolator.tangent and Interpolator.tangent().
        private static final HeadingInterpolator tangent = HeadingInterpolator.tangent;

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

    /** Lets visualizer exports that call Paths.curve()/Paths.line() paste in unchanged. */
    private static final class Paths {
        private static CompatiblePath curve(Pose... controlPoints) {
            return new CompatiblePath(new BezierCurve(controlPoints));
        }

        private static CompatiblePath line(Pose startPose, Pose endPose) {
            return new CompatiblePath(new BezierLine(startPose, endPose));
        }
    }

    private static CompatiblePath curve(Pose... controlPoints) {
        return Paths.curve(controlPoints);
    }

    private static CompatiblePath line(Pose startPose, Pose endPose) {
        return Paths.line(startPose, endPose);
    }

    private Command autonomousSequence() {
        return new SequentialGroup(
                shootPreloads(),
                helpShootFar ? helpShootFarSequence() : trustPartnerSequence()
        );
    }

    private Command parkFrom(PathChain parkPath) {
        return new SequentialGroup(
                enterStage(AutonStage.PARK),
                new InstantCommand(() ->
                        ShooterSubsystem2026.INSTANCE.stop()),
                new InstantCommand(() ->
                        IntakeWithSensorsSubsystem2026.INSTANCE.stopIntake()),
                new InstantCommand(() ->
                        FlowerIntakeSubsystem2026.INSTANCE.moveUp()),
                new FollowPath(parkPath),
                enterStage(AutonStage.DONE)
        );
    }

    private Command shootPreloads() {
        return new SequentialGroup(
                enterStage(AutonStage.SPIN_UP),
                new InstantCommand(() ->
                        ShooterSubsystem2026.INSTANCE.spinUp(
                                SHOOTER_FLYWHEEL_RPM_Low
                        )
                ),
                new ParallelGroup(
                        new Delay(SHOOTER_SPINUP_SECONDS),
                        new FollowPath(closeshootPath).endAfter(1.0)
                ),
                enterStage(AutonStage.SHOOT_PRELOADS),
                shoot(),
                new InstantCommand(() -> firstVolleyDoneNanos = System.nanoTime()),
                new InstantCommand(() ->
                        ShooterSubsystem2026.INSTANCE.spinUp(
                                SHOOTER_FLYWHEEL_RPM_High
                        )
                )
        );
    }

    private Command helpShootFarSequence() {
        return new SequentialGroup(
                enterStage(AutonStage.DRIVE_TO_FLOWER),
                new ParallelGroup(
                        new FollowPath(flowerNearPath),
                        new InstantCommand(() ->
                                IntakeWithSensorsSubsystem2026.INSTANCE.intakeForward()),
                        new InstantCommand(() ->
                                FlowerIntakeSubsystem2026.INSTANCE.moveDown())
                ),
                enterStage(AutonStage.COLLECT),
                new Delay(FLOWERY_WAIT_SECONDS),
                enterStage(AutonStage.DRIVE_TO_SHOOT),
                new ParallelGroup(
                        new FollowPath(farShootingPath),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() ->
                                        FlowerIntakeSubsystem2026.INSTANCE.moveUp())
                        )
                ),
                enterStage(AutonStage.SHOOT),
                shoot(),

                collectFromGarden(gardenintakeFromFarPath),
                enterStage(AutonStage.DRIVE_TO_SHOOT),
                new FollowPath(closeshoot2Path),
                new Delay(0.75),
                enterStage(AutonStage.SHOOT),
                shoot(),
                parkFrom(parkingPath)
        );
    }

    private Command trustPartnerSequence() {
        return new SequentialGroup(
                collectFromGarden(gardenintakeFromClosePath),
                enterStage(AutonStage.DRIVE_TO_SHOOT),
                new FollowPath(closeshoot2Path),
                new Delay(0.75),
                enterStage(AutonStage.WAIT_FOR_PARTNER),
                new WaitUntil(() ->
                        secondsSinceFirstVolley() >= partnerDelaySeconds),
                enterStage(AutonStage.SHOOT),
                shoot(),

                enterStage(AutonStage.DRIVE_TO_FLOWER),
                new ParallelGroup(
                        new FollowPath(flowerNearFromClosePath),
                        new InstantCommand(() ->
                                IntakeWithSensorsSubsystem2026.INSTANCE.intakeForward()),
                        new InstantCommand(() ->
                                FlowerIntakeSubsystem2026.INSTANCE.moveDown())
                ),
                enterStage(AutonStage.COLLECT),
                new Delay(FLOWERY_WAIT_SECONDS),
                new IfElseCommand(
                        this::hasTimeForFinalCloseShot,
                        new SequentialGroup(
                                enterStage(AutonStage.DRIVE_TO_SHOOT),
                                new ParallelGroup(
                                        new FollowPath(closeShoot3FromFlowerNearPath),
                                        new SequentialGroup(
                                                new Delay(0.5),
                                                new InstantCommand(() ->
                                                        FlowerIntakeSubsystem2026.INSTANCE.moveUp())
                                        )
                                ),
                                new Delay(0.75),
                                enterStage(AutonStage.SHOOT),
                                shoot(),
                                parkFrom(parkingPath)
                        ),
                        parkFrom(parkingFromFlowerNearPath)
                )
        );
    }

    private boolean hasTimeForFinalCloseShot() {
        return secondsSinceStart() < LATEST_FINAL_CLOSE_SHOT_START_SECONDS;
    }

    /**
     * Drives into the garden corner while intaking. If the robot is not full
     * after the wait, it backs up and drives back in once more.
     */
    private Command collectFromGarden(PathChain approachPath) {
        return new SequentialGroup(
                enterStage(AutonStage.DRIVE_TO_GARDEN),
                new ParallelGroup(
                        new FollowPath(approachPath),
                        new InstantCommand(() ->
                                IntakeWithSensorsSubsystem2026.INSTANCE.intakeForward())
                ),
                enterStage(AutonStage.COLLECT),
                new Delay(GARDEN_INTAKE_WAIT_SECONDS),
                new IfElseCommand(
                        () -> IntakeWithSensorsSubsystem2026.INSTANCE.getBallCount()
                                < FULL_BALL_COUNT,
                        new SequentialGroup(
                                enterStage(AutonStage.GARDEN_RETRY),
                                // Short moves into the wall may never reach the
                                // path end, so each one is time limited.
                                new FollowPath(gardenBackupPath, true, GARDEN_BACKUP_POWER)
                                        .endAfter(GARDEN_BACKUP_TIMEOUT_SECONDS),
                                new FollowPath(gardenReturnPath, true, GARDEN_SLOW_POWER)
                                        .endAfter(GARDEN_RETURN_TIMEOUT_SECONDS),
                                new Delay(GARDEN_RETRY_WAIT_SECONDS)
                        )
                )
        );
    }

    private Command shoot() {
        return new SequentialGroup(
                new InstantCommand(shotSequence::start),
                new WaitUntil(() -> !shotSequence.isActive())
        );
    }

    private double secondsSinceFirstVolley() {
        return firstVolleyDoneNanos == 0L
                ? 0.0 : (System.nanoTime() - firstVolleyDoneNanos) / 1e9;
    }

    private double secondsSinceStart() {
        return startPressedNanos == 0L
                ? 0.0 : (System.nanoTime() - startPressedNanos) / 1e9;
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

    private void addPlanTelemetry() {
        telemetry.addData(
                "Plan",
                helpShootFar
                        ? "HELP FAR (dpad up): flower -> far shot -> garden -> close shot"
                        : "TRUST PARTNER (dpad down): garden -> close shot -> near flower -> close shot"
        );
        telemetry.addData(
                "Partner delay (dpad L/R)",
                helpShootFar
                        ? "%.0f sec (not used when helping far)"
                        : "%.0f sec after preloads",
                partnerDelaySeconds
        );
        if (!helpShootFar) {
            telemetry.addData(
                    "Final close shot",
                    "only if near flower done before %.1f sec",
                    LATEST_FINAL_CLOSE_SHOT_START_SECONDS
            );
        }
    }

    @Override
    public void onWaitForStart() {
        if (gamepad1.dpad_up) {
            helpShootFar = true;
        } else if (gamepad1.dpad_down) {
            helpShootFar = false;
        }

        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;
        if (dpadLeft && !previousDpadLeft) {
            partnerDelaySeconds = Math.max(
                    0.0, partnerDelaySeconds - PARTNER_DELAY_STEP_SECONDS);
        }
        if (dpadRight && !previousDpadRight) {
            partnerDelaySeconds = Math.min(
                    MAX_PARTNER_DELAY_SECONDS,
                    partnerDelaySeconds + PARTNER_DELAY_STEP_SECONDS);
        }
        previousDpadLeft = dpadLeft;
        previousDpadRight = dpadRight;

        TurretSubsystem2026.INSTANCE.addTelemetry(telemetry);
        Pose pose = PedroComponent.follower().getPose();
        addPlanTelemetry();
        addStageTelemetry();
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
        startPressedNanos = System.nanoTime();
        new SequentialGroup(
                new WaitUntil(TurretSubsystem2026.INSTANCE::canShoot),
                autonomousSequence()
        ).schedule();
    }

    @Override
    public void onUpdate() {
        TurretSubsystem2026.INSTANCE.addTelemetry(telemetry);
        // The shared controller owns feed timing and the timed shooter boost.
        shotSequence.update();

        Pose pose = PedroComponent.follower().getPose();

        addPlanTelemetry();
        addStageTelemetry();
        telemetry.addData("Time since Start", "%.1f sec", secondsSinceStart());
        if (!helpShootFar && firstVolleyDoneNanos != 0L) {
            telemetry.addData(
                    "Partner wait remaining",
                    "%.1f sec",
                    Math.max(0.0, partnerDelaySeconds - secondsSinceFirstVolley())
            );
        }
        telemetry.addData("Ball count",
                IntakeWithSensorsSubsystem2026.INSTANCE.getBallCount());
        telemetry.addData("Follower busy", PedroComponent.follower().isBusy());
        telemetry.addData("X", "%.2f in", pose.getX());
        telemetry.addData("Y", "%.2f in", pose.getY());
        telemetry.addData("Heading", "%.1f deg", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Shot sequence active", shotSequence.isActive());
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
            TurretSubsystem2026.INSTANCE.saveAutonomousHandoff();
            GlobalRobotData.hasAutonRun = true;
        }

        TurretSubsystem2026.INSTANCE.stop();
        super.onStop();
    }
}
