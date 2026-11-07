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
import org.firstinspires.ftc.teamcode.subsystems.AutonomousShooterRpm2026;
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
 * Far-side autonomous for matches with a partner.
 *
 * <p>Init selection on gamepad 1:</p>
 * <ul>
 *     <li>Dpad down (default): far shot after the delay, far flower, far shot,
 *     park.</li>
 *     <li>Dpad up: partner only shoots preloads and parks. After the two far
 *     shots, drive through the middle to the garden, close shot, near flower,
 *     close shot (skipped if it is too late), park.</li>
 *     <li>Dpad left/right: first-shot delay after Start, 1 second per press.
 *     The far goal only opens after the hive tips, so the far shots wait.</li>
 * </ul>
 */
@Configurable
@Autonomous(name = "Start Far With Partner", group = "Comp")
public class FarWithPartner extends NextFTCOpMode {

    // False preserves the tested fixed targets. True follows TeleOp's live regression/targets.
    public static boolean USE_DISTANCE_BASED_RPM = false;

    public static double SHOOTER_FLYWHEEL_RPM = AutonomousShooterRpm2026.DEFAULT_HIGH_RPM;
    public static double SHOOTER_SPINUP_SECONDS = 2.0;
    public static double FLOWERY_WAIT_SECONDS = 1.0;
    public static double GARDEN_SLOW_START = 0.75;
    public static double GARDEN_SLOW_POWER = 0.40;

    public static double GARDEN_INTAKE_WAIT_SECONDS = 1.0;
    public static double GARDEN_RETRY_WAIT_SECONDS = 0.5;
    public static double GARDEN_BACKUP_POWER = 0.5;
    public static double GARDEN_BACKUP_TIMEOUT_SECONDS = 1.0;
    public static double GARDEN_RETURN_TIMEOUT_SECONDS = 1.5;

    public static double DEFAULT_FIRST_SHOT_DELAY_SECONDS = 10.0;
    public static double FIRST_SHOT_DELAY_STEP_SECONDS = 1.0;
    public static double MAX_FIRST_SHOT_DELAY_SECONDS = 25.0;

    // After the near flower pickup, the final close shot is only attempted if
    // fewer than this many seconds have passed since Start. Otherwise park.
    public static double LATEST_FINAL_CLOSE_SHOT_START_SECONDS = 21.0;

    private static final int FULL_BALL_COUNT = 4;

    // -------------------------------------------------------------------------
    // PEDRO VISUALIZER POSES
    // -------------------------------------------------------------------------
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(61.42, 136, 270);
    private final Pose farShootingFromStart = poseFactory.of(35, 120, 305);
    private final Pose farShootingFromStartControl1 = poseFactory.of(61.42, 112, 0);
    private final Pose farShootingFromStartControl2 = poseFactory.of(45, 112, 0);
    private final Pose flowerfar = poseFactory.of(45.716, 128.8, 90);
    private final Pose flowerfarControl1 = poseFactory.of(46.3382, 101.403, 0);
    private final Pose flowerfarSegment1Start = poseFactory.of(45.716, 128.8, 305);
    private final Pose flowerfarSegment1End = poseFactory.of(45.716, 128.8, 90);
    private final Pose flowerfarSegment2Start = poseFactory.of(45.716, 128.8, 90);
    private final Pose flowerfarSegment2End = poseFactory.of(45.716, 128.8, 90);
    private final Pose farshooting2 = poseFactory.of(35, 120, -55);
    private final Pose farshooting2Control1 = poseFactory.of(43.4511, 114.4166, 0);
    private final Pose farshooting2Segment1Heading = poseFactory.of(35, 120, 90);
    private final Pose farshooting2Segment2Start = poseFactory.of(35, 120, 90);
    private final Pose farshooting2Segment2End = poseFactory.of(35, 120, 305);
    private final Pose throughHive = poseFactory.of(59.3063, 35.3987, -104.6766);
    private final Pose throughHiveControl1 = poseFactory.of(68.3998, 120.9034, 0);
    private final Pose throughHiveControl2 = poseFactory.of(62.9121, 49.1664, 0);
    private final Pose gardenFromHive = poseFactory.of(8, 6, 180);
    private final Pose gardenFromHiveControl1 = poseFactory.of(56.5104, 23.6309, 0);
    private final Pose gardenFromHiveControl2 = poseFactory.of(25.2146, 10.6622, 0);
    // Starts at throughHive's ending tangent heading so there is no snap.
    private final Pose gardenFromHiveSegment1Start = poseFactory.of(8, 6, -104.6766);
    private final Pose gardenFromHiveSegment1End = poseFactory.of(8, 6, 220);
    private final Pose gardenFromHiveSegment2Start = poseFactory.of(8, 6, 220);
    private final Pose gardenFromHiveSegment2End = poseFactory.of(8, 6, 220);
    private final Pose gardenFromHiveSegment3Start = poseFactory.of(8, 6, 220);
    private final Pose gardenFromHiveSegment3End = poseFactory.of(8, 6, 180);
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
    // The partner parks in the rear of the box (around 13, 95), so parking from
    // the close side loops out around it and comes in from above.
    private final Pose farParkingFromClose = poseFactory.of(13, 119, 0);
    private final Pose farParkingFromCloseControl1 = poseFactory.of(16.5461, 62.3115, 0);
    private final Pose farParkingFromCloseControl2 = poseFactory.of(55.3972, 129.1871, 0);
    private final Pose farParkingFromCloseSegment2Start = poseFactory.of(13, 119, 90);
    private final Pose farParkingFromCloseSegment2End = poseFactory.of(13, 119, 0);
    private final Pose farParkingFromFlowerNear = poseFactory.of(13, 119, 0);
    private final Pose farParkingFromFlowerNearControl1 = poseFactory.of(32, 55, 0);
    private final Pose farParkingFromFlowerNearControl2 = poseFactory.of(55.3972, 129.1871, 0);
    private final Pose farParkingFromFlowerNearSegment2Start = poseFactory.of(13, 119, 90);
    private final Pose farParkingFromFlowerNearSegment2End = poseFactory.of(13, 119, 0);
    private final Pose farParkingStart = poseFactory.of(35, 120, 305);
    private final Pose farParking = poseFactory.of(13, 119, 0);

    public Path farShootingFromStart() {
        return Paths.curve(start, farShootingFromStartControl1, farShootingFromStartControl2, farShootingFromStart).linear(start, farShootingFromStart);
    }

    public Path flowerfar() {
        return Paths.curve(farShootingFromStart, flowerfarControl1, flowerfar).heading(Interpolator.piecewise().until(0.5, Interpolator.linear(flowerfarSegment1Start, flowerfarSegment1End)).until(1, Interpolator.linear(flowerfarSegment2Start, flowerfarSegment2End)));
    }

    public Path farshooting2() {
        return Paths.curve(flowerfar, farshooting2Control1, farshooting2).heading(Interpolator.piecewise().until(0.1646, Interpolator.constant(farshooting2Segment1Heading)).until(1, Interpolator.linear(farshooting2Segment2Start, farshooting2Segment2End)));
    }

    public Path throughHive() {
        return Paths.curve(farshooting2, throughHiveControl1, throughHiveControl2, throughHive).tangent();
    }

    public Path gardenFromHive() {
        return Paths.curve(throughHive, gardenFromHiveControl1, gardenFromHiveControl2, gardenFromHive).heading(Interpolator.piecewise().until(0.4, Interpolator.linear(gardenFromHiveSegment1Start, gardenFromHiveSegment1End)).until(0.8, Interpolator.linear(gardenFromHiveSegment2Start, gardenFromHiveSegment2End)).until(1, Interpolator.linear(gardenFromHiveSegment3Start, gardenFromHiveSegment3End)));
    }

    public Path gardenBackup() {
        return Paths.line(gardenFromHive, gardenBackup).constant(gardenBackup);
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

    public Path farParkingFromClose() {
        return Paths.curve(closeShoot3FromFlowerNear, farParkingFromCloseControl1, farParkingFromCloseControl2, farParkingFromClose).heading(Interpolator.piecewise().until(0.5, Interpolator.tangent).until(1, Interpolator.linear(farParkingFromCloseSegment2Start, farParkingFromCloseSegment2End)));
    }

    public Path farParkingFromFlowerNear() {
        return Paths.curve(flowerNearFromClose, farParkingFromFlowerNearControl1, farParkingFromFlowerNearControl2, farParkingFromFlowerNear).heading(Interpolator.piecewise().until(0.5, Interpolator.tangent).until(1, Interpolator.linear(farParkingFromFlowerNearSegment2Start, farParkingFromFlowerNearSegment2End)));
    }

    public Path farParking() {
        return Paths.line(farParkingStart, farParking).linear(farParkingStart, farParking);
    }

    // -------------------------------------------------------------------------
    // PEDRO VISUALIZER PATH FIELDS
    // -------------------------------------------------------------------------
    private PathChain farShootingFromStartPath;
    private PathChain flowerfarPath;
    private PathChain farshooting2Path;
    private PathChain gardenThroughHivePath;
    private PathChain gardenBackupPath;
    private PathChain gardenReturnPath;
    private PathChain closeshoot2Path;
    private PathChain flowerNearFromClosePath;
    private PathChain closeShoot3FromFlowerNearPath;
    private PathChain farParkingPath;
    private PathChain farParkingFromClosePath;
    private PathChain farParkingFromFlowerNearPath;

    private boolean autonStarted;
    private boolean partnerOnlyShootsPreloads;
    private double firstShotDelaySeconds;
    private boolean previousDpadLeft;
    private boolean previousDpadRight;
    private long startPressedNanos;

    private enum AutonStage {
        READY,
        SPIN_UP,
        WAIT_FOR_PARTNER,
        SHOOT_PRELOADS,
        DRIVE_TO_FLOWER,
        COLLECT,
        DRIVE_TO_GARDEN,
        GARDEN_RETRY,
        DRIVE_TO_SHOOT,
        SHOOT,
        PARK,
        DONE
    }

    private AutonStage autonStage = AutonStage.READY;
    private long stageStartNanos;
    private final ShotSequenceController2026 shotSequence =
            new ShotSequenceController2026();
    private final AutonomousShooterRpm2026 shooterRpm = new AutonomousShooterRpm2026();

    public FarWithPartner() {
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
        shooterRpm.reset(SHOOTER_FLYWHEEL_RPM);
        TurretSubsystem2026.INSTANCE.prepareAutonomous();
        autonStarted = false;
        partnerOnlyShootsPreloads = false;
        firstShotDelaySeconds = DEFAULT_FIRST_SHOT_DELAY_SECONDS;
        previousDpadLeft = false;
        previousDpadRight = false;
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
        farShootingFromStartPath = PedroComponent.follower().pathBuilder()
                .addPath(farShootingFromStart())
                .build();

        flowerfarPath = PedroComponent.follower().pathBuilder()
                .addPath(flowerfar())
                .build();

        farshooting2Path = PedroComponent.follower().pathBuilder()
                .addPath(farshooting2())
                .build();

        gardenThroughHivePath = PedroComponent.follower().pathBuilder()
                .addPath(throughHive())
                .addPath(gardenFromHive())
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

        farParkingPath = PedroComponent.follower().pathBuilder()
                .addPath(farParking())
                .build();

        farParkingFromClosePath = PedroComponent.follower().pathBuilder()
                .addPath(farParkingFromClose())
                .build();

        farParkingFromFlowerNearPath = PedroComponent.follower().pathBuilder()
                .addPath(farParkingFromFlowerNear())
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
                shootPreloadsAfterDelay(),
                farFlowerAndShoot(),
                partnerOnlyShootsPreloads
                        ? gardenCloseShotAndNearFlower()
                        : parkFrom(farParkingPath)
        );
    }

    private Command parkFrom(PathChain parkingPath) {
        return new SequentialGroup(
                enterStage(AutonStage.PARK),
                new InstantCommand(() ->
                        ShooterSubsystem2026.INSTANCE.stop()),
                new InstantCommand(() ->
                        IntakeWithSensorsSubsystem2026.INSTANCE.stopIntake()),
                new InstantCommand(() ->
                        FlowerIntakeSubsystem2026.INSTANCE.moveUp()),
                new FollowPath(parkingPath),
                enterStage(AutonStage.DONE)
        );
    }

    private Command shootPreloadsAfterDelay() {
        return new SequentialGroup(
                enterStage(AutonStage.SPIN_UP),
                new InstantCommand(() ->
                        startShooter(SHOOTER_FLYWHEEL_RPM)),
                new ParallelGroup(
                        new Delay(SHOOTER_SPINUP_SECONDS),
                        new FollowPath(farShootingFromStartPath)
                ),
                enterStage(AutonStage.WAIT_FOR_PARTNER),
                new WaitUntil(() -> secondsSinceStart() >= firstShotDelaySeconds),
                enterStage(AutonStage.SHOOT_PRELOADS),
                shoot()
        );
    }

    private Command farFlowerAndShoot() {
        return new SequentialGroup(
                enterStage(AutonStage.DRIVE_TO_FLOWER),
                new ParallelGroup(
                        new FollowPath(flowerfarPath).endAfter(2.5),
                        new InstantCommand(() ->
                                IntakeWithSensorsSubsystem2026.INSTANCE.intakeForward()),
                        new InstantCommand(() ->
                                FlowerIntakeSubsystem2026.INSTANCE.moveDown())
                ),
                enterStage(AutonStage.COLLECT),
                new Delay(FLOWERY_WAIT_SECONDS),
                enterStage(AutonStage.DRIVE_TO_SHOOT),
                new FollowPath(farshooting2Path),
                new InstantCommand(() ->
                        FlowerIntakeSubsystem2026.INSTANCE.moveUp()),
                new Delay(0.75),
                enterStage(AutonStage.SHOOT),
                shoot()
        );
    }

    private Command gardenCloseShotAndNearFlower() {
        return new SequentialGroup(
                collectFromGarden(gardenThroughHivePath),
                enterStage(AutonStage.DRIVE_TO_SHOOT),
                new FollowPath(closeshoot2Path),
                new Delay(0.75),
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
                                parkFrom(farParkingFromClosePath)
                        ),
                        parkFrom(farParkingFromFlowerNearPath)
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

    private double secondsSinceStart() {
        return startPressedNanos == 0L
                ? 0.0 : (System.nanoTime() - startPressedNanos) / 1e9;
    }

    private void startShooter(double fixedRpm) {
        shooterRpm.start(fixedRpm, USE_DISTANCE_BASED_RPM, PedroComponent.follower().getPose());
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
                partnerOnlyShootsPreloads
                        ? "PARTNER PRELOADS ONLY (dpad up): 2 far shots -> garden -> close shot -> near flower -> close shot -> park"
                        : "DEFAULT (dpad down): 2 far shots -> park"
        );
        telemetry.addData(
                "First shot delay (dpad L/R)",
                "%.0f sec after Start",
                firstShotDelaySeconds
        );
        if (partnerOnlyShootsPreloads) {
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
            partnerOnlyShootsPreloads = true;
        } else if (gamepad1.dpad_down) {
            partnerOnlyShootsPreloads = false;
        }

        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;
        if (dpadLeft && !previousDpadLeft) {
            firstShotDelaySeconds = Math.max(
                    0.0, firstShotDelaySeconds - FIRST_SHOT_DELAY_STEP_SECONDS);
        }
        if (dpadRight && !previousDpadRight) {
            firstShotDelaySeconds = Math.min(
                    MAX_FIRST_SHOT_DELAY_SECONDS,
                    firstShotDelaySeconds + FIRST_SHOT_DELAY_STEP_SECONDS);
        }
        previousDpadLeft = dpadLeft;
        previousDpadRight = dpadRight;

        TurretSubsystem2026.INSTANCE.addTelemetry(telemetry);
        Pose pose = PedroComponent.follower().getPose();
        shooterRpm.addTelemetry(telemetry, USE_DISTANCE_BASED_RPM, pose);
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
        Pose pose = PedroComponent.follower().getPose();
        shooterRpm.update(USE_DISTANCE_BASED_RPM, pose);
        shotSequence.update();
        shooterRpm.addTelemetry(telemetry, USE_DISTANCE_BASED_RPM, pose);

        addPlanTelemetry();
        addStageTelemetry();
        telemetry.addData("Time since Start", "%.1f sec", secondsSinceStart());
        telemetry.addData(
                "First shot wait remaining",
                "%.1f sec",
                Math.max(0.0, firstShotDelaySeconds - secondsSinceStart())
        );
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
