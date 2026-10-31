package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.GlobalRobotData;
import org.firstinspires.ftc.teamcode.subsystems.IntakeWithSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.CsvLogger;

import java.io.File;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

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
//@Autonomous(name = "closeBlueSide", group = "Comp")
public class closeAutonPaths_WorldsTopTriangle extends NextFTCOpMode{
    public static double autonShooterRPM = 3370.0; //changed from 3350
    public static double nearAutonShooterRPM = 3200.0;
    public static double autonShooterHoodServoPos = 0.525;
    public static double nearAutonShooterHoodServoPos = 0.44;
    public static double nearShootingMoving_x = 25.0;
    public static double nearShootingMoving_y = 144;
    public static double gateIntakeWaitDelay = 2.5;
    public static double gateIntakeShortWaitDelay = 1.5;

    public static double autonShootingTurretAngle = -97;
    public static double pickupBrakingStrength = 1.0;
    public static double pickupCornerBrakingStrength = 0.5;
    public static double firstGatePushDelay = 0.0;
    public static double secondGatePushDelay = 0.4;
    public static double thirdGatePushDelay = 2.00;
    public static double threeGatePushDelayBeforeLastShot = 1.5;
    public static boolean ENABLE_AUTON_LOGGING = true;
    public static long AUTON_LOG_PERIOD_MS = 25L;
    public static double AUTON_SHOOTER_AT_SPEED_TOLERANCE_RPM = 75.0;
    public static boolean AUTON_USE_HYBRID_DUMBSHOOT_BOOST = true;

    private CsvLogger autonLogger;
    private long autonLogStartMs = 0L;
    private long lastAutonLogMs = 0L;
    private long lastAutonLoopMs = 0L;

    public static double activeGateIntakeWaitDelay = gateIntakeWaitDelay;
    public static double redTargetPoseXOffset = -5.0;
    public static double redTargetPoseYOffset = 0.0;

    public final Pose startPoseBlue = new Pose(31.5, 134.0, Math.toRadians(270)); // Start Pose of our robot
    private final Pose scorePoseCloseBlue = new Pose(59, 74, Math.toRadians(195)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePoseFarBlue = new Pose(33, 107, Math.toRadians(225)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePoseNearBlue = new Pose(50, 87, Math.toRadians(235)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePoseFinalBlue = new Pose(61, 101, Math.toRadians(235)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private final Pose targetPoseCloseBlue = new Pose(-4, 144.0, Math.toRadians(180));
    private final Pose pickup1PoseBlue = new Pose(40, 88.0, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark. //move +3 Y to the right
    private final Pose pickup1CP1Blue = new Pose(45.0, 85.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1CP2Blue = new Pose( 38.0, 85.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup1EndPoseBlue = new Pose(27, 86.0, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark. //move +3 Y to the right
    private final Pose pickup1GateLeverEndPoseBlue = new Pose(24, 86.0, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark. //move +3 Y to the right

    private final Pose pickup1GateLeverPushPoseBlue = new Pose(23, 75, Math.toRadians(180));
    private final Pose pickup1GateLeverPushCP1Blue = new Pose(30.5, 82, Math.toRadians(180));
    private final Pose pickup1GateLeverPushCP2Blue = new Pose( 27, 76, Math.toRadians(180));

    private final Pose pickup1GateLeverPushHoldPoseBlue = new Pose(15.5, 72.0, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark. //move +3 Y to the right

    private final Pose pickup2PoseBlue = new Pose(41, 63, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2CP1Blue = new Pose(51, 65, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2CP2Blue = new Pose( 41, 62, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup2EndPoseBlue = new Pose(22.0, 60.0, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark. //move +3 Y to the right
    private final Pose getPickup2CPPathBlue = new Pose(48, 67, Math.toRadians(180));

    private final Pose pickup2GateLeverEndPoseBlue = new Pose(17, 60.0, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup2GateLeverPushPoseBlue = new Pose(23, 69.0, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2GateLeverPushCP1Blue = new Pose(26.1, 61, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2GateLeverPushCP2Blue = new Pose(28.5, 69.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup2GateLeverPushHoldPoseBlue = new Pose(18, 68.0, Math.toRadians(180));

    private final Pose pickup3GateLeverEndPoseBlue = new Pose(20, 35.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup3GateLeverPushPoseBlue = new Pose(23, 70.0, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup3GateLeverPushCP1Blue = new Pose(22.5, 57, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup3GateLeverPushCP2Blue = new Pose(28.5, 69.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup3GateLeverPushHoldPoseBlue = new Pose(15.5, 72.0, Math.toRadians(180));

    private final Pose pickup3PoseBlue = new Pose(40, 41.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup3CP1Blue = new Pose(54.5,53  , Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup3CP2Blue = new Pose( 53.5, 38, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup3AfterGateCP1Blue = new Pose(52,73  , Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup3AfterGateCP2Blue = new Pose( 61.5, 36, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose toGateBefore3PoseBlue = new Pose( 24, 72, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose toGateBefore3CP1Blue = new Pose(47, 91.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose toGateBefore3CP2Blue = new Pose(42,78  , Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose toGateBefore3HoldPoseBlue = new Pose(16, 70.0, Math.toRadians(180));

    private final Pose pickup3EndPoseBlue = new Pose(18, 35.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark. //move +3 Y to the right
    private final Pose getPickup3CPPathBlue = new Pose(40, 45, Math.toRadians(180));

    private final Pose pickupZonePoseBlue = new Pose(9, 37, Math.toRadians(235)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickupZoneCP1Blue = new Pose(32,48, Math.toRadians(235)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickupZoneCP2Blue = new Pose( 12, 55.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickupZoneEndPoseBlue = new Pose(8, 13.0, Math.toRadians(255)); // Highest (First Set) of Artifacts from the Spike Mark. //move +3 Y to the right
    private final Pose getPickupZoneCPPathBlue = new Pose(44, 59, Math.toRadians(235));

    private final Pose offLineLeverBlue = new Pose(30, 90, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose offLineLeverExtraTimeBlue = new Pose(30, 80, Math.toRadians(270)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose offLineCloseBlue = new Pose(50, 117, Math.toRadians(135));
    private final Pose offLineAfterPickupBlue = new Pose(50, 60, Math.toRadians(90));

    // Gate poses and control points
    private final Pose gateCP1Blue = new Pose(53.576, 72.048, Math.toRadians(180));
    private final Pose gateCP2Blue = new Pose(35.985, 63.624, Math.toRadians(180));

    private final Pose gatePoseBlue = new Pose(17.010, 66.268, Math.toRadians(167));
    private final Pose directGateCP1Blue = new Pose(53.576, 72.048, Math.toRadians(180));
    private final Pose directGateCP2Blue = new Pose(35.985, 63.624, Math.toRadians(180));
    private final Pose directGatePoseBlue = new Pose(12, 61.0,     Math.toRadians(145));

    // Intake from gate poses
    private final Pose intakeCP1Blue = new Pose(13.708, 51.629, Math.toRadians(180));
    private final Pose intakePoseBlue = new Pose(9.481, 54.984, Math.toRadians(123));
    private final Pose directGateIntakeCP1Blue = new Pose(13.708, 51.629, Math.toRadians(180));
    private final Pose directGateIntakePoseBlue = new Pose(9.481, 54.984, Math.toRadians(123));

    // Intake from gate return to shoot
    private final Pose intakeFromGateShootCP1Blue = new Pose(44.217, 64.852, Math.toRadians(180));
    private final Pose directGateShootCP1Blue = new Pose(44.217, 64.852, Math.toRadians(180));

    public final Pose startPoseRed = startPoseBlue.mirror(144);
    private final Pose scorePoseCloseRed = scorePoseCloseBlue.mirror(144);
    private final Pose scorePoseNearRed = scorePoseNearBlue.mirror(144);
    private final Pose scorePoseFinalRed = scorePoseFinalBlue.mirror(144);

    private final Pose targetPoseCloseRed = targetPoseCloseBlue.mirror(144);
    private final Pose pickup1PoseRed = pickup1PoseBlue.mirror(144);
    private final Pose pickup1CP1Red = pickup1CP1Blue.mirror(144);
    private final Pose pickup1CP2Red = pickup1CP2Blue.mirror(144);
    private final Pose pickup1EndPoseRed = pickup1EndPoseBlue.mirror(144);

    private final Pose pickup1GateLeverEndPoseRed = pickup1GateLeverEndPoseBlue.mirror(144);
    private final Pose pickup1GateLeverPushPoseRed = pickup1GateLeverPushPoseBlue.mirror(144);
    private final Pose pickup1GateLeverPushCP1Red = pickup1GateLeverPushCP1Blue.mirror(144);
    private final Pose pickup1GateLeverPushCP2Red = pickup1GateLeverPushCP2Blue.mirror(144);
    private final Pose pickup1GateLeverPushHoldPoseRed = pickup1GateLeverPushHoldPoseBlue.mirror(144);

    private final Pose pickup2PoseRed = pickup2PoseBlue.mirror(144);
    private final Pose pickup2CP1Red = pickup2CP1Blue.mirror(144);
    private final Pose pickup2CP2Red = pickup2CP2Blue.mirror(144);
    private final Pose pickup2EndPoseRed = pickup2EndPoseBlue.mirror(144);
    private final Pose getPickup2CPPathRed = getPickup2CPPathBlue.mirror(144);
    private final Pose pickup2GateLeverEndPoseRed = pickup2GateLeverEndPoseBlue.mirror(144);
    private final Pose pickup2GateLeverPushPoseRed = pickup2GateLeverPushPoseBlue.mirror(144);
    private final Pose pickup2GateLeverPushHoldPoseRed = pickup2GateLeverPushHoldPoseBlue.mirror(144);
    private final Pose pickup2GateLeverPushCP1Red = pickup2GateLeverPushCP1Blue.mirror(144);
    private final Pose pickup2GateLeverPushCP2Red = pickup2GateLeverPushCP2Blue.mirror(144);
    private final Pose pickup3GateLeverEndPoseRed = pickup3GateLeverEndPoseBlue.mirror(144);
    private final Pose pickup3GateLeverPushPoseRed = pickup3GateLeverPushPoseBlue.mirror(144);
    private final Pose pickup3GateLeverPushHoldPoseRed = pickup3GateLeverPushHoldPoseBlue.mirror(144);
    private final Pose pickup3GateLeverPushCP1Red = pickup3GateLeverPushCP1Blue.mirror(144);
    private final Pose pickup3GateLeverPushCP2Red = pickup3GateLeverPushCP2Blue.mirror(144);
    private final Pose pickup3PoseRed = pickup3PoseBlue.mirror(144);
    private final Pose pickup3CP1Red = pickup3CP1Blue.mirror(144);
    private final Pose pickup3CP2Red = pickup3CP2Blue.mirror(144);
    private final Pose pickup3AfterGateCP1Red = pickup3AfterGateCP1Blue.mirror(144);
    private final Pose pickup3AfterGateCP2Red = pickup3AfterGateCP2Blue.mirror(144);
    private final Pose toGateBefore3PoseRed = toGateBefore3PoseBlue.mirror(144);
    private final Pose toGateBefore3CP1Red = toGateBefore3CP1Blue.mirror(144);
    private final Pose toGateBefore3CP2Red = toGateBefore3CP2Blue.mirror(144);
    private final Pose toGateBefore3HoldPoseRed = toGateBefore3HoldPoseBlue.mirror(144);
    private final Pose pickup3EndPoseRed = pickup3EndPoseBlue.mirror(144);
    private final Pose getPickup3CPPathRed = getPickup3CPPathBlue.mirror(144);
    private final Pose pickupZonePoseRed = pickupZonePoseBlue.mirror(144);
    private final Pose pickupZoneCP1Red = pickupZoneCP1Blue.mirror(144);
    private final Pose pickupZoneCP2Red = pickupZoneCP2Blue.mirror(144);
    private final Pose pickupZoneEndPoseRed = pickupZoneEndPoseBlue.mirror(144);
    private final Pose getPickupZoneCPPathRed = getPickupZoneCPPathBlue.mirror(144);

    private final Pose offLineLeverRed = offLineLeverBlue.mirror(144);
    private final Pose offLineLeverExtraTimeRed = offLineLeverExtraTimeBlue.mirror(144);
    private final Pose offLineCloseRed = offLineCloseBlue.mirror(144);
    private final Pose offLineAfterPickupRed = offLineAfterPickupBlue.mirror(144);

    private final Pose gateCP1Red = gateCP1Blue.mirror(144);
    private final Pose gateCP2Red = gateCP2Blue.mirror(144);
    private final Pose gatePoseRed = gatePoseBlue.mirror(144);
    private final Pose intakeCP1Red = intakeCP1Blue.mirror(144);
    private final Pose intakePoseRed = intakePoseBlue.mirror(144);
    private final Pose intakeFromGateShootCP1Red = intakeFromGateShootCP1Blue.mirror(144);

    private final Pose directGateCP1Red = directGateCP1Blue.mirror(144);
    private final Pose directGateCP2Red = directGateCP2Blue.mirror(144);
    private final Pose directGatePoseRed = directGatePoseBlue.mirror(144);
    private final Pose directGateIntakeCP1Red = directGateIntakeCP1Blue.mirror(144);
    private final Pose directGateIntakePoseRed = directGateIntakePoseBlue.mirror(144);
    private final Pose directGateShootCP1Red = directGateShootCP1Blue.mirror(144);

    public Pose currentGoodPose = new Pose(0,0,0);

    public double turretOffset = 0.0;

    public Pose targetPoseClose;

    public Pose startPose;
    public Pose scorePoseClose;
    public Pose scorePoseNear;
    public Pose scorePoseFinal;

    public Pose pickup1Pose;
    public Pose pickup1CP1;
    public Pose pickup1CP2;

    public Pose pickup1EndPose;

    public Pose pickup1GateLeverEndPose;
    public Pose pickup1GateLeverPushPose;
    public Pose pickup1GatePushCP1;
    public Pose pickup1GatePushCP2;
    public Pose pickup1GateLeverPushHoldPose;

    public Pose pickup2Pose;
    public Pose pickup2CP1;
    public Pose pickup2CP2;

    public Pose pickup2EndPose;
    public Pose getPickup2CPPath;

    public Pose pickup2GateLeverEndPose;

    public Pose pickup2GateLeverPushPose;
    public Pose pickup2GateLeverPushHoldPose;
    public Pose pickup2GateLeverPushCP1;
    public Pose pickup2GateLeverPushCP2;

    public Pose pickup3GateLeverEndPose;

    public Pose pickup3GateLeverPushPose;
    public Pose pickup3GateLeverPushHoldPose;
    public Pose pickup3GateLeverPushCP1;
    public Pose pickup3GateLeverPushCP2;

    public Pose pickup3Pose;
    public Pose pickup3CP1;
    public Pose pickup3CP2;
    public Pose pickup3AfterGateCP1;
    public Pose pickup3AfterGateCP2;

    public Pose toGateBefore3Pose;
    public Pose toGateBefore3CP1;
    public Pose toGateBefore3CP2;
    public Pose toGateBefore3HoldPose;

    public Pose pickup3EndPose;
    public Pose getPickup3CPPath;

    public Pose pickupZonePose;
    public Pose pickupZoneCP1;
    public Pose pickupZoneCP2;

    public Pose pickupZoneEndPose;
    public Pose getPickupZoneCPPath;

    public Pose offLineLever;
    public Pose offLineLeverExtraTime;
    public Pose offLineClose;
    public Pose offLineAfterPickup;

    public Pose gateCP1;
    public Pose gateCP2;
    public Pose gatePose;
    public Pose intakeCP1;
    public Pose intakePose;
    public Pose intakeFromGateShootCP1;

    public Pose directGateCP1;
    public Pose directGateCP2;
    public Pose directGatePose;
    public Pose directGateIntakeCP1;
    public Pose directGateIntakePose;
    public Pose directGateShootCP1;

    private PathChain firstshootpath,firstshootpathStraight, firstPickupEndToBalls, firstPickupEndToScore, secondPickupStraight, secondPickupEndToBalls, secondPickupEndToScore, thirdPickupEndToBalls,thirdPickupEndToScore1stRow, thirdPickupEndToScore, thirdPickupGateLeverEnd, moveOffLineLeverExtraTime, thirdPickupAfterGate, goToGateLeverBefore3rdPickup, thirdPickupGateLeverShootEnd, firstPickup,firstPickupEnd, firstPickupGateLeverEnd, firstPickupGateLeverShootEnd, secondPickup, secondPickupEnd, secondPickupGateLeverEnd, secondPickupGateLeverShootEnd, thirdPickup, thirdPickupEnd, zonePickup, zonePickupEnd, zonePickupShoot, moveOffLineLever, moveOffLineClose, moveOffLineAfterPickup;
    private PathChain hitGateBeforePickup;
    private PathChain intakeFromGate;
    private PathChain intakeFromGateShoot;
    private PathChain hitDirectGateBeforePickup;
    private PathChain intakeFromDirectGate;
    private PathChain intakeFromDirectGateShoot, intakeFromDirectGateShoot1stRow;

    protected void startAutonLogger() {
        autonLogStartMs = System.currentTimeMillis();
        lastAutonLogMs = 0L;
        lastAutonLoopMs = 0L;

        if (!ENABLE_AUTON_LOGGING) {
            autonLogger = null;
            return;
        }

        autonLogger = new CsvLogger(getClass().getSimpleName() + "_auton");
        autonLogger.start(
                "t_ms," +
                        "match_t_ms," +
                        "bot_x," +
                        "bot_y," +
                        "bot_heading_deg," +
                        "turret_target_deg," +
                        "turret_measured_deg," +
                        "shooter_target_rpm," +
                        "shooter_rpm1," +
                        "shooter_rpm2," +
                        "shooter_at_speed_75," +
                        "shooter_battery_v," +
                        "shooter_battery_v_filtered," +
                        "intake_active," +
                        "intake_direction," +
                        "intake_m1_rpm," +
                        "intake_m3_rpm," +
                        "ball_count," +
                        "loop_time_ms"
        );
    }

    protected void logAutonLoop() {
        long nowMs = System.currentTimeMillis();
        long loopTimeMs = (lastAutonLoopMs == 0L) ? 0L : (nowMs - lastAutonLoopMs);
        lastAutonLoopMs = nowMs;

        Pose pose = PedroComponent.follower().getPose();
        if ((pose.getX() == 0) && (pose.getY() == 0) && (pose.getHeading() == 0)) {
        } else {
            currentGoodPose = pose;
        }
        turretOffset = TurretSubsystem.INSTANCE.getMeasuredAngleDegrees();

        if (!ENABLE_AUTON_LOGGING || autonLogger == null) {
            return;
        }
        if (lastAutonLogMs != 0L && nowMs - lastAutonLogMs < AUTON_LOG_PERIOD_MS) {
            return;
        }
        lastAutonLogMs = nowMs;

        autonLogger.addRow(
                nowMs,
                (autonLogStartMs == 0L) ? 0L : (nowMs - autonLogStartMs),
                pose.getX(),
                pose.getY(),
                Math.toDegrees(pose.getHeading()),
                TurretSubsystem.INSTANCE.getTargetAngleDegrees(),
                TurretSubsystem.INSTANCE.getMeasuredAngleDegrees(),
                ShooterSubsystem.INSTANCE.getTargetRpm(),
                ShooterSubsystem.INSTANCE.getShooter1RPM(),
                ShooterSubsystem.INSTANCE.getShooter2RPM(),
                ShooterSubsystem.INSTANCE.isAtSpeed(AUTON_SHOOTER_AT_SPEED_TOLERANCE_RPM),
                ShooterSubsystem.INSTANCE.getBatteryVoltageRaw(),
                ShooterSubsystem.INSTANCE.getBatteryVoltageFiltered(),
                IntakeWithSensorsSubsystem.INSTANCE.isIntakingActive(),
                IntakeWithSensorsSubsystem.INSTANCE.getCurrentDirection(),
                IntakeWithSensorsSubsystem.INSTANCE.getMotor1RPM(),
                IntakeWithSensorsSubsystem.INSTANCE.getMotor3RPM(),
                IntakeWithSensorsSubsystem.INSTANCE.getBallCount(),
                loopTimeMs
        );
    }

    protected void saveAutonLogger() {
        if (!ENABLE_AUTON_LOGGING || autonLogger == null) {
            return;
        }

        File savedFile = autonLogger.save();
        if (savedFile != null) {
            RobotLog.ii(getClass().getSimpleName(), "Auton CSV saved: " + savedFile.getAbsolutePath());
        } else {
            RobotLog.ww(getClass().getSimpleName(), "Auton CSV was not saved.");
        }
    }

    private Pose withRedTargetPoseOffset(Pose targetPose) {
        return new Pose(
                targetPose.getX() + redTargetPoseXOffset,
                targetPose.getY() + redTargetPoseYOffset,
                targetPose.getHeading()
        );
    }

        public void buildPaths() {

            if (GlobalRobotData.allianceSide == GlobalRobotData.COLOR.BLUE) {
                targetPoseClose = targetPoseCloseBlue;

                startPose = startPoseBlue;
                scorePoseClose = scorePoseCloseBlue;
                scorePoseNear = scorePoseNearBlue;
                scorePoseFinal = scorePoseFinalBlue;
                pickup1Pose = pickup1PoseBlue;
                pickup1CP1 = pickup1CP1Blue;
                pickup1CP2 = pickup1CP2Blue;
                pickup1EndPose = pickup1EndPoseBlue;
                pickup1GateLeverEndPose = pickup1GateLeverEndPoseBlue;
                pickup1GateLeverPushPose = pickup1GateLeverPushPoseBlue;
                pickup1GatePushCP1 = pickup1GateLeverPushCP1Blue;
                pickup1GatePushCP2 = pickup1GateLeverPushCP2Blue;
                pickup1GateLeverPushHoldPose = pickup1GateLeverPushHoldPoseBlue;
                pickup2Pose = pickup2PoseBlue;
                pickup2CP1 = pickup2CP1Blue;
                pickup2CP2 = pickup2CP2Blue;
                pickup2EndPose = pickup2EndPoseBlue;
                pickup2GateLeverEndPose = pickup2GateLeverEndPoseBlue;
                pickup2GateLeverPushPose = pickup2GateLeverPushPoseBlue;
                pickup2GateLeverPushHoldPose = pickup2GateLeverPushHoldPoseBlue;
                pickup2GateLeverPushCP1 = pickup2GateLeverPushCP1Blue;
                pickup2GateLeverPushCP2 = pickup2GateLeverPushCP2Blue;
                pickup3GateLeverEndPose = pickup3GateLeverEndPoseBlue;
                pickup3GateLeverPushPose = pickup3GateLeverPushPoseBlue;
                pickup3GateLeverPushHoldPose = pickup3GateLeverPushHoldPoseBlue;
                pickup3GateLeverPushCP1 = pickup3GateLeverPushCP1Blue;
                pickup3GateLeverPushCP2 = pickup3GateLeverPushCP2Blue;
                getPickup2CPPath = getPickup2CPPathBlue;
                pickup3Pose = pickup3PoseBlue;
                pickup3CP1 = pickup3CP1Blue;
                pickup3CP2 = pickup3CP2Blue;
                pickup3AfterGateCP1 = pickup3AfterGateCP1Blue;
                pickup3AfterGateCP2 = pickup3AfterGateCP2Blue;
                toGateBefore3Pose = toGateBefore3PoseBlue;
                toGateBefore3CP1 = toGateBefore3CP1Blue;
                toGateBefore3CP2 = toGateBefore3CP2Blue;
                toGateBefore3HoldPose = toGateBefore3HoldPoseBlue;
                pickup3EndPose = pickup3EndPoseBlue;
                getPickup3CPPath = getPickup3CPPathBlue;
                pickupZonePose = pickupZonePoseBlue;
                pickupZoneCP1 = pickupZoneCP1Blue;
                pickupZoneCP2 = pickupZoneCP2Blue;
                pickupZoneEndPose = pickupZoneEndPoseBlue;
                getPickupZoneCPPath = getPickupZoneCPPathBlue;
                offLineLever = offLineLeverBlue;
                offLineLeverExtraTime = offLineLeverExtraTimeBlue;
                offLineClose = offLineCloseBlue;
                offLineAfterPickup = offLineAfterPickupBlue;
                gateCP1 = gateCP1Blue;
                gateCP2 = gateCP2Blue;
                gatePose = gatePoseBlue;
                intakeCP1 = intakeCP1Blue;
                intakePose = intakePoseBlue;
                intakeFromGateShootCP1 = intakeFromGateShootCP1Blue;
                directGateCP1 = directGateCP1Blue;
                directGateCP2 = directGateCP2Blue;
                directGatePose = directGatePoseBlue;
                directGateIntakeCP1 = directGateIntakeCP1Blue;
                directGateIntakePose = directGateIntakePoseBlue;
                directGateShootCP1 = directGateShootCP1Blue;
            } else {
                targetPoseClose = withRedTargetPoseOffset(targetPoseCloseRed);
                startPose = startPoseRed;
                scorePoseClose = scorePoseCloseRed;
                scorePoseNear = scorePoseNearRed;
                scorePoseFinal = scorePoseFinalRed;
                pickup1Pose = pickup1PoseRed;
                pickup1CP1 = pickup1CP1Red;
                pickup1CP2 = pickup1CP2Red;
                pickup1EndPose = pickup1EndPoseRed;
                pickup1GateLeverEndPose = pickup1GateLeverEndPoseRed;
                pickup1GateLeverPushPose = pickup1GateLeverPushPoseRed;
                pickup1GatePushCP1 = pickup1GateLeverPushCP1Red;
                pickup1GatePushCP2 = pickup1GateLeverPushCP2Red;
                pickup1GateLeverPushHoldPose = pickup1GateLeverPushHoldPoseRed;
                pickup2Pose = pickup2PoseRed;
                pickup2CP1 = pickup2CP1Red;
                pickup2CP2 = pickup2CP2Red;
                pickup2EndPose = pickup2EndPoseRed;
                pickup2GateLeverEndPose = pickup2GateLeverEndPoseRed;
                pickup2GateLeverPushPose = pickup2GateLeverPushPoseRed;
                pickup2GateLeverPushHoldPose = pickup2GateLeverPushHoldPoseRed;
                pickup2GateLeverPushCP1 = pickup2GateLeverPushCP1Red;
                pickup2GateLeverPushCP2 = pickup2GateLeverPushCP2Red;
                pickup3GateLeverEndPose = pickup3GateLeverEndPoseRed;
                pickup3GateLeverPushPose = pickup3GateLeverPushPoseRed;
                pickup3GateLeverPushHoldPose = pickup3GateLeverPushHoldPoseRed;
                pickup3GateLeverPushCP1 = pickup3GateLeverPushCP1Red;
                pickup3GateLeverPushCP2 = pickup3GateLeverPushCP2Red;
                getPickup2CPPath = getPickup2CPPathRed;
                pickup3Pose = pickup3PoseRed;
                pickup3CP1 = pickup3CP1Red;
                pickup3CP2 = pickup3CP2Red;
                pickup3AfterGateCP1 = pickup3AfterGateCP1Red;
                pickup3AfterGateCP2 = pickup3AfterGateCP2Red;
                toGateBefore3Pose = toGateBefore3PoseRed;
                toGateBefore3CP1 = toGateBefore3CP1Red;
                toGateBefore3CP2 = toGateBefore3CP2Red;
                toGateBefore3HoldPose = toGateBefore3HoldPoseRed;
                pickup3EndPose = pickup3EndPoseRed;
                getPickup3CPPath = getPickup3CPPathRed;
                pickupZonePose = pickupZonePoseRed;
                pickupZoneCP1 = pickupZoneCP1Red;
                pickupZoneCP2 = pickupZoneCP2Red;
                pickupZoneEndPose = pickupZoneEndPoseRed;
                getPickupZoneCPPath = getPickupZoneCPPathRed;
                offLineLever = offLineLeverRed;
                offLineLeverExtraTime = offLineLeverExtraTimeRed;
                offLineClose = offLineCloseRed;
                offLineAfterPickup = offLineAfterPickupRed;
                gateCP1 = gateCP1Red;
                gateCP2 = gateCP2Red;
                gatePose = gatePoseRed;
                intakeCP1 = intakeCP1Red;
                intakePose = intakePoseRed;
                intakeFromGateShootCP1 = intakeFromGateShootCP1Red;
                directGateCP1 = directGateCP1Red;
                directGateCP2 = directGateCP2Red;
                directGatePose = directGatePoseRed;
                directGateIntakeCP1 = directGateIntakeCP1Red;
                directGateIntakePose = directGateIntakePoseRed;
                directGateShootCP1 = directGateShootCP1Red;

            }

            // if we want to change braking for a pathchain, then use .setNoDeceleration() or .setBrakingStrength(double set) after a path is added
            firstshootpath=PedroComponent.follower().pathBuilder()
                    .addPath(
                    new BezierCurve(
                            new Pose(31.500, 134.000),
                            new Pose(50.031, 133.018),
                            new Pose(55.727, 121.363),
                            new Pose(52.308, 114.664) )
                   )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270),.75)

                    .addPath(
                            new BezierLine(
                                    new Pose(52.308, 114.664),
                                    new Pose(47.297, 107.442))

                    )
                     .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                    .setNoDeceleration()
                    .build();

            firstshootpathStraight=PedroComponent.follower().pathBuilder()

                    .addPath(
                            new BezierLine(
                                    startPose,
                                    scorePoseClose)

                    )
                    .setLinearHeadingInterpolation(startPose.getHeading(), scorePoseClose.getHeading())
                    .build();

            firstPickup= PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePoseClose,
                                    pickup1CP1,
                                    pickup1CP2,
                                    pickup1Pose
                            )
                    )
                    .setLinearHeadingInterpolation(scorePoseClose.getHeading(), pickup1Pose.getHeading(),.75)
                    .setNoDeceleration()
                    .build();

            firstPickupEnd = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup1Pose, pickup1EndPose))
                    .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1EndPose.getHeading())

                    .addPath(new BezierLine(pickup1EndPose, scorePoseClose))
                    .setLinearHeadingInterpolation(pickup1EndPose.getHeading(), scorePoseClose.getHeading())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            firstPickupEndToBalls = PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePoseNear,
                                    pickup1CP1,
                                    pickup1CP2,
                                    pickup1EndPose
                            )
                    )
                    .setConstantHeadingInterpolation(pickup1EndPose.getHeading())
                    .setNoDeceleration()
                    .build();

            firstPickupEndToScore = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup1EndPose, scorePoseFinal))
                    .setLinearHeadingInterpolation(pickup1EndPose.getHeading(), scorePoseFinal.getHeading())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            firstPickupGateLeverEnd = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup1Pose, pickup1GateLeverEndPose))
                    .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1GateLeverEndPose.getHeading())

                    .addPath(
                            new BezierCurve(
                                    pickup1GateLeverEndPose,
                                    pickup1GatePushCP1,
                                    pickup1GatePushCP2,
                                    pickup1GateLeverPushPose
                            )
                    )
                    .setLinearHeadingInterpolation(pickup1GateLeverEndPose.getHeading(), pickup1GateLeverPushPose.getHeading(),.75)

                    .addPath(new BezierLine(pickup1GateLeverPushPose, pickup1GateLeverPushHoldPose))
                    .setLinearHeadingInterpolation(pickup1GateLeverPushPose.getHeading(), pickup1GateLeverPushHoldPose.getHeading())

                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            firstPickupGateLeverShootEnd = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup1GateLeverPushHoldPose, scorePoseClose))
                    .setLinearHeadingInterpolation(pickup1GateLeverPushHoldPose.getHeading(), scorePoseClose.getHeading())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            secondPickup= PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(47.297, 107.442),
                                    new Pose(46.817, 98.437),
                                    new Pose(47.317, 90.819),
                                    new Pose(51.719, 82.533)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                    .addPath(
                            new BezierCurve(
                                    new Pose(51.719, 82.533),
                                    pickup2CP1,
                                    pickup2CP2,
                                    pickup2Pose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), pickup2Pose.getHeading(), 0.75)
                    .setNoDeceleration()
                    .build();

            secondPickupStraight= PedroComponent.follower().pathBuilder()
                   .addPath(
                            new BezierCurve(
                                    scorePoseClose,
                                    pickup2CP1,
                                    pickup2CP2,
                                    pickup2Pose
                            )
                    ).setLinearHeadingInterpolation(scorePoseClose.getHeading(), pickup2Pose.getHeading(), 0.70)
                    .setNoDeceleration()
                    .setTValueConstraint(.8)
                    .build();

            secondPickupEnd = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(    pickup2Pose,
                                                pickup2EndPose))
                    .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2EndPose.getHeading())

                    .addPath(new BezierCurve( pickup2EndPose,
                            getPickup2CPPath,
                            scorePoseClose))
                    .setLinearHeadingInterpolation(pickup2EndPose.getHeading(), scorePoseClose.getHeading())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            secondPickupEndToBalls = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierCurve(scorePoseClose, pickup2CP1, pickup2CP2, pickup2EndPose))
                    .setLinearHeadingInterpolation(scorePoseClose.getHeading(), pickup2EndPose.getHeading(), 0.2)
                    .setNoDeceleration()
                    .build();

            secondPickupEndToScore = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierCurve(pickup2EndPose, getPickup2CPPath, scorePoseClose))
                    .setLinearHeadingInterpolation(pickup2EndPose.getHeading(), scorePoseClose.getHeading())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            secondPickupGateLeverEnd = PedroComponent.follower().pathBuilder()


                    .addPath(new BezierCurve(
                            pickup2EndPose,
                            pickup2GateLeverPushCP1,
                            pickup2GateLeverPushCP2,
                            pickup2GateLeverPushPose))
                    .setLinearHeadingInterpolation(pickup2GateLeverEndPose.getHeading(), pickup2GateLeverPushPose.getHeading())

                    .addPath(new BezierLine(pickup2GateLeverPushPose, pickup2GateLeverPushHoldPose))
                    .setLinearHeadingInterpolation(pickup2GateLeverPushPose.getHeading(), pickup2GateLeverPushHoldPose.getHeading())

                    .setBrakingStrength(pickupBrakingStrength)
                    .build();
//
//            secondPickupGateLeverEnd = PedroComponent.follower().pathBuilder()
//                    .addPath(new BezierLine(pickup2Pose, pickup2GateLeverEndPose))
//                    .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2GateLeverEndPose.getHeading())
//
//                    .addPath(new BezierCurve(
//                            pickup2GateLeverEndPose,
//                            pickup2GateLeverPushCP1,
//                            pickup2GateLeverPushCP2,
//                            pickup2GateLeverPushPose))
//                    .setLinearHeadingInterpolation(pickup2GateLeverEndPose.getHeading(), pickup2GateLeverPushPose.getHeading())
//
//                    .addPath(new BezierLine(pickup2GateLeverPushPose, pickup2GateLeverPushHoldPose))
//                    .setLinearHeadingInterpolation(pickup2GateLeverPushPose.getHeading(), pickup2GateLeverPushHoldPose.getHeading())
//
//                    .setBrakingStrength(pickupBrakingStrength)
//                    .build();

            goToGateLeverBefore3rdPickup = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierCurve(
                            scorePoseClose,
                            toGateBefore3CP1,
                            toGateBefore3CP2,
                            toGateBefore3Pose))
                    .setLinearHeadingInterpolation(scorePoseClose.getHeading(), toGateBefore3Pose.getHeading())

                    .addPath(new BezierLine(toGateBefore3Pose, toGateBefore3HoldPose))
                    .setLinearHeadingInterpolation(toGateBefore3Pose.getHeading(), toGateBefore3HoldPose.getHeading())

                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            thirdPickupGateLeverEnd = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup3Pose, pickup3GateLeverEndPose))
                    .setLinearHeadingInterpolation(pickup3Pose.getHeading(), pickup3GateLeverEndPose.getHeading())

                    .addPath(new BezierCurve(
                            pickup3GateLeverEndPose,
                            pickup3GateLeverPushCP1,
                            pickup3GateLeverPushCP2,
                            pickup3GateLeverPushPose))
                    .setLinearHeadingInterpolation(pickup3GateLeverEndPose.getHeading(), pickup3GateLeverPushPose.getHeading())

                    .addPath(new BezierLine(pickup3GateLeverPushPose, pickup3GateLeverPushHoldPose))
                    .setLinearHeadingInterpolation(pickup3GateLeverPushPose.getHeading(), pickup3GateLeverPushHoldPose.getHeading())

                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            secondPickupGateLeverShootEnd = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup2GateLeverPushPose, scorePoseClose))
                    .setLinearHeadingInterpolation(pickup2GateLeverPushPose.getHeading(), scorePoseClose.getHeading())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            thirdPickupGateLeverShootEnd = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup3GateLeverPushPose, scorePoseClose))
                    .setLinearHeadingInterpolation(pickup3GateLeverPushPose.getHeading(), scorePoseClose.getHeading())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            thirdPickupAfterGate= PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    toGateBefore3HoldPose,
                                    pickup3AfterGateCP1,
                                    pickup3AfterGateCP2,
                                    pickup3Pose
                            )
                    )
                    .setLinearHeadingInterpolation(toGateBefore3HoldPose.getHeading(), pickup3Pose.getHeading(),.75)
                    .setNoDeceleration()
                    .build();

            thirdPickup= PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePoseClose,
                                    pickup3CP1,
                                    pickup3CP2,
                                    pickup3Pose
                            )
                    )
                    .setLinearHeadingInterpolation(scorePoseClose.getHeading(), pickup3Pose.getHeading(),.75)
                    .setNoDeceleration()
                    .build();

            thirdPickupEnd = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup3Pose, pickup3EndPose))
                    .setLinearHeadingInterpolation(pickup3Pose.getHeading(), pickup3EndPose.getHeading())

                    .addPath(new BezierCurve(pickup3EndPose, getPickup3CPPath, scorePoseClose))
                    .setLinearHeadingInterpolation(pickup3EndPose.getHeading(), scorePoseClose.getHeading())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            thirdPickupEndToBalls = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierCurve(scorePoseClose, pickup3CP1, pickup3CP2, pickup3EndPose))
                    .setLinearHeadingInterpolation(scorePoseClose.getHeading(), pickup3EndPose.getHeading(), 0.2)
                    .setNoDeceleration()
                    .build();

            thirdPickupEndToScore1stRow = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup3EndPose, scorePoseNear))
                    .setTangentHeadingInterpolation()
                    .setHeadingInterpolation(HeadingInterpolator.tangent.reverse())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            zonePickup= PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePoseClose,
                                    pickupZoneCP1,
                                    pickupZoneCP2,
                                    pickupZonePose
                            )
                    )
                    .setLinearHeadingInterpolation(scorePoseClose.getHeading(), pickupZonePose.getHeading(),.75)
                    .setNoDeceleration()
                    .build();

            zonePickupEnd = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickupZonePose, pickupZoneEndPose))
                    .setLinearHeadingInterpolation(pickupZonePose.getHeading(), pickupZoneEndPose.getHeading())
                    .setBrakingStrength(pickupCornerBrakingStrength)
                    .build();

            zonePickupShoot = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierCurve(pickupZoneEndPose, getPickupZoneCPPath, scorePoseClose))
                    .setLinearHeadingInterpolation(pickupZoneEndPose.getHeading(), scorePoseClose.getHeading())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            moveOffLineLever=PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierLine(scorePoseClose,offLineLever)

                    )
                    .setLinearHeadingInterpolation(scorePoseClose.getHeading(), offLineLever.getHeading())
                    .build();

            moveOffLineLeverExtraTime=PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierLine(scorePoseClose,offLineLeverExtraTime)

                    )
                    .setLinearHeadingInterpolation(scorePoseClose.getHeading(), offLineLever.getHeading())
                    .build();

            moveOffLineAfterPickup=PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierLine(pickupZoneEndPose,offLineAfterPickup)

                    )
                    .setLinearHeadingInterpolation(scorePoseClose.getHeading(), offLineLever.getHeading())
                    .build();

            moveOffLineClose=PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierLine(scorePoseClose,offLineClose)

                    )
                    .setLinearHeadingInterpolation(scorePoseClose.getHeading(), offLineClose.getHeading())
                    .build();

            // Path 5: From score area, hit gate before pickup
            hitGateBeforePickup = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierCurve(scorePoseClose, gateCP1, gateCP2, gatePose))
                    .setLinearHeadingInterpolation(scorePoseClose.getHeading(), gatePose.getHeading())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            // Path 6: Intake from gate position
            intakeFromGate = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierCurve(gatePose, intakeCP1, intakePose))
                    .setLinearHeadingInterpolation(gatePose.getHeading(), intakePose.getHeading())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            // Path 7: Return from intake to far scoring position
            intakeFromGateShoot = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierCurve(intakePose, intakeFromGateShootCP1, scorePoseClose))
                    .setLinearHeadingInterpolation(intakePose.getHeading(), scorePoseClose.getHeading())
                    .build();

            // DirectGate copies of the gate paths. Start with the same geometry as the
            // original gate sequence, but keep the poses/pathchains independent for tuning.
            hitDirectGateBeforePickup = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierCurve(scorePoseClose, directGateCP1, directGateCP2, directGatePose))
                    .setLinearHeadingInterpolation(scorePoseClose.getHeading(), directGatePose.getHeading())
                    // maybe use .setGlobalDeceleration()
                    .setBrakingStrength(pickupBrakingStrength)
                    .setTValueConstraint(0.98) // require 98% of the path before considering it at the parametric end
                    .build();

            intakeFromDirectGate = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierCurve(directGatePose, directGateIntakeCP1, directGateIntakePose))
                    .setLinearHeadingInterpolation(directGatePose.getHeading(), directGateIntakePose.getHeading())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            intakeFromDirectGateShoot = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierCurve(directGatePose, directGateShootCP1, scorePoseClose))
                    .setLinearHeadingInterpolation(directGatePose.getHeading(), scorePoseClose.getHeading())
                    .build();

            intakeFromDirectGateShoot1stRow = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierCurve(directGatePose, directGateShootCP1, scorePoseNear))
                    .setLinearHeadingInterpolation(directGatePose.getHeading(), scorePoseNear.getHeading())
                    .build();

        }
    // if you want to reduce power for a path, use FollowPath(pathchain, holdend, maxpower)

    private Command AutonDumbShootWithHybridBoost() {
        return new InstantCommand(() -> {
            IntakeWithSensorsSubsystem.INSTANCE.setDumbShootDistanceForDelayInches(0.0);
            IntakeWithSensorsSubsystem.INSTANCE.dumbShoot();

            if (AUTON_USE_HYBRID_DUMBSHOOT_BOOST && ShooterSubsystem.ENABLE_HYBRID_SHOT_FEED_BOOST) {
                ShooterSubsystem.INSTANCE.startHybridShotFeedBoostController(false);
            }
        });
    }

    public Command CloseShootPreload() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(firstshootpathStraight),
                        new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                        new InstantCommand(() -> ShooterSubsystem.INSTANCE.shooterHoodDrive(autonShooterHoodServoPos)),
                        new InstantCommand(() -> TurretSubsystem.INSTANCE.aimAtFieldPoint(targetPoseClose)),
                        //new InstantCommand(() -> TurretSubsystem.INSTANCE.setTargetAngleFromRobotFrontRelativeDegrees(autonShootingTurretAngle)),
                        new Delay(1.5)  //wait until flywheel is up to speed
                ),
                AutonDumbShootWithHybridBoost(),
                new Delay(0.20)
        );
    }

    public Command CloseShootMovingPreload() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(firstshootpath, false, 0.70),
                        new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(nearAutonShooterRPM)),
                        new InstantCommand(() -> TurretSubsystem.INSTANCE.aimAtFieldPoint(nearShootingMoving_x, nearShootingMoving_y)),
                        new SequentialGroup(
                                new WaitUntil(() -> PedroComponent.follower().getPathCompletion() >= 0.80),
                                //new Delay(1.50),  //Could replace this with shooting a ball
                                AutonDumbShootWithHybridBoost(),
                                new Delay(0.50)
                                )
                ),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                //new InstantCommand(() -> TurretSubsystem.INSTANCE.setTargetAngleFromRobotFrontRelativeDegrees(autonShootingTurretAngle)),
                new InstantCommand(() -> TurretSubsystem.INSTANCE.aimAtFieldPoint(targetPoseClose)),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.shooterHoodDrive(autonShooterHoodServoPos))
                );
    }

    public Command CloseGoToFirstPickupLine() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(firstPickup),
                        new SequentialGroup(
                                new Delay(0.3),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(0.7),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                )
        );
    }
    public Command ClosePickupAndShootFirstRow() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(firstPickupEnd),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),

                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(0.0),
                AutonDumbShootWithHybridBoost(),
                new Delay(0.50),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
        );
    }

    public Command ClosePickupAndShootFirstRowRace() {
        return new SequentialGroup(
                new Delay(0.2),
                new ParallelGroup(
                        new ParallelRaceGroup(
                                new FollowPath(firstPickupEndToBalls),
                                new WaitUntil(() -> IntakeWithSensorsSubsystem.INSTANCE.getBallCount() >= 3)
                        ),
                        new SequentialGroup(
                                new Delay(0.3),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(0.1),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward()),
                                new Delay(0.1),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(nearAutonShooterRPM))
                        )
                ),
                new ParallelGroup(
                        new FollowPath(firstPickupEndToScore),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(0.0),
                AutonDumbShootWithHybridBoost(),
                new Delay(0.50)
        );
    }

    public Command ClosePickupAndShootGateRace() {
        return new SequentialGroup(
                new Delay(0.2),
                new ParallelGroup(
                        new SequentialGroup(
                                new FollowPath(hitDirectGateBeforePickup,true),
                                new ParallelRaceGroup(
                                        new Delay(activeGateIntakeWaitDelay),
                                        new WaitUntil(() -> IntakeWithSensorsSubsystem.INSTANCE.getBallCount() >= 3)
                                )
                        ),
                        new SequentialGroup(
                                new Delay(0.3),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(0.1),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward()),
                                new Delay(0.1),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
                        )
                ),
                new ParallelGroup(
                        new FollowPath(intakeFromDirectGateShoot),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(0.0),
                AutonDumbShootWithHybridBoost(),
                new Delay(0.20)
        );
    }

    public Command ClosePickupAndShootGateBefore1stRowRace() {
        return new SequentialGroup(
                new Delay(0.2),
                new ParallelGroup(
                        new SequentialGroup(
                                new FollowPath(hitDirectGateBeforePickup,true),
                                new ParallelRaceGroup(
                                        new Delay(activeGateIntakeWaitDelay),
                                        new WaitUntil(() -> IntakeWithSensorsSubsystem.INSTANCE.getBallCount() >= 3)
                                )
                        ),
                        new SequentialGroup(
                                new Delay(0.3),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(0.1),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward()),
                                new Delay(0.1),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.shooterHoodDrive(nearAutonShooterHoodServoPos)),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(nearAutonShooterRPM))
                        )
                ),
                new ParallelGroup(
                        new FollowPath(intakeFromDirectGateShoot1stRow),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(0.0),
                AutonDumbShootWithHybridBoost(),
                new Delay(0.20)
        );
    }

    public Command ClosePickupAndGateLeverFirstRow() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(firstPickupGateLeverEnd),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                )
        );
    }

    public Command ClosePickupShootAfterGateLever1stRow() {
        return new SequentialGroup(
                new FollowPath(firstPickupGateLeverShootEnd),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(0.0),
                AutonDumbShootWithHybridBoost(),
                new Delay(0.50),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
        );
    }

    public Command CloseGoTo2ndPickupLine() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(secondPickup),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(1.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                )
            );
    }

    public Command CloseGoTo2ndPickupLineStraight() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(secondPickupStraight),
                        new SequentialGroup(
                                new Delay(0.3),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(0.7),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                )
        );
    }

    public Command ClosePickupAndShoot2ndRow() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(secondPickupEnd),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(0.0),
                AutonDumbShootWithHybridBoost(),
                new Delay(0.50),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
        );
    }

    public Command ClosePickupAndShoot2ndRowRace() {
        return new SequentialGroup(
                new ParallelGroup(
                        new ParallelRaceGroup(
                                new FollowPath(secondPickupEndToBalls),
                                new WaitUntil(() -> IntakeWithSensorsSubsystem.INSTANCE.getBallCount() >= 3)
                        ),
                        new SequentialGroup(
                                new Delay(0.3),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(0.1),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward()),
                                new Delay(0.1),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
                        )       
                ),
                new ParallelGroup(
                        new FollowPath(secondPickupEndToScore),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(0.0),
                AutonDumbShootWithHybridBoost(),
                new Delay(0.20)
        );
    }

    public Command ClosePickupAndShoot2ndRowRaceAndGate() {
        return new SequentialGroup(
                new ParallelGroup(
                        new ParallelRaceGroup(
                                new FollowPath(secondPickupEndToBalls),
                                new WaitUntil(() -> IntakeWithSensorsSubsystem.INSTANCE.getBallCount() >= 3)
                        ),
                        new SequentialGroup(
                                new Delay(0.3),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(0.1),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward()),
                                new Delay(0.1),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
                        )
                ),
                new FollowPath(secondPickupGateLeverEnd),
                new Delay(0.1),

                new ParallelGroup(
                        new FollowPath(secondPickupEndToScore),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(0.0),
                AutonDumbShootWithHybridBoost(),
                new Delay(0.20)
        );
    }

    public Command ClosePickupAndGateLever2ndRow() {
        return new SequentialGroup(
                    new ParallelGroup(
                        new FollowPath(secondPickupGateLeverEnd),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                )
       );
    }

    public Command ClosePickupShootAfterGateLever2ndRow() {
        return new SequentialGroup(
                new FollowPath(secondPickupGateLeverShootEnd),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(0.0),
                AutonDumbShootWithHybridBoost(),
                new Delay(0.50),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
        );
    }

    public Command CloseGoToGateLeverBefore3rdPickup() {
        return new SequentialGroup(
                new FollowPath(goToGateLeverBefore3rdPickup)
        );
    }

    public Command CloseGoTo3rdPickupAfterGateLever() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(thirdPickupAfterGate),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(1.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                )
        );
    }

    public Command ClosePickupAndGateLever3rdRow() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(thirdPickupGateLeverEnd),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                )
        );
    }

    public Command ClosePickupShootAfterGateLever3rdRow() {
        return new SequentialGroup(
                new FollowPath(thirdPickupGateLeverShootEnd),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(0.0),
                AutonDumbShootWithHybridBoost(),
                new Delay(0.50),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
        );
    }

    public Command CloseGoTo3rdPickupLine() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(thirdPickup),
                        new SequentialGroup(
                                new Delay(0.3),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(0.7),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                )
        );
    }


    public Command ClosePickupAndShoot3rdRow() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(thirdPickupEnd),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(0.0),
                AutonDumbShootWithHybridBoost(),
                new Delay(0.2)
        );
    }

    public Command ClosePickupAndShoot3rdRowBefore1stRowRace() {
        return new SequentialGroup(
                new Delay(0.2),
                new ParallelGroup(
                        new ParallelRaceGroup(
                                new FollowPath(thirdPickupEndToBalls),
                                new WaitUntil(() -> IntakeWithSensorsSubsystem.INSTANCE.getBallCount() >= 3)
                        ),
                        new SequentialGroup(
                                new Delay(0.3),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(0.1),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward()),
                                new Delay(0.1),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.shooterHoodDrive(nearAutonShooterHoodServoPos)),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(nearAutonShooterRPM))
                        )
                ),
                new ParallelGroup(
                        new FollowPath(thirdPickupEndToScore1stRow),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(0.0),
                AutonDumbShootWithHybridBoost(),
                new Delay(0.2)
        );
    }

    public Command ClosePickupAndShoot3rdRowRace() {
        return new SequentialGroup(
                new ParallelGroup(
                    new ParallelRaceGroup(
                            new FollowPath(thirdPickupEndToBalls),
                            new WaitUntil(() -> IntakeWithSensorsSubsystem.INSTANCE.getBallCount() >= 3)
                        ),
                                new SequentialGroup(
                                new Delay(0.3),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(0.1),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward()),
                                new Delay(0.5),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
                        )
                ),
                new ParallelGroup(
                        new FollowPath(thirdPickupEndToScore),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(0.0),
                AutonDumbShootWithHybridBoost(),
                new Delay(0.2)
        );
    }

    public Command ClosePickupAndShootAfter3rdRowWithDelay() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(thirdPickupEnd),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(threeGatePushDelayBeforeLastShot),
                AutonDumbShootWithHybridBoost(),
                new Delay(0.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0))
        );
    }

    public Command CloseGoToZonePickupLine() {
        return new SequentialGroup(
                new ParallelGroup(
                        FollowZonePickupUntilFull(),
                        //new FollowPath(zonePickup),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(0.1),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                )
        );
    }

    /**
     * Follows the zonePickupEnd path and terminates when EITHER:
     * - The path completes, OR
     * - The intake has collected 3 balls
     *
     * Uses ParallelRaceGroup which stops when the first command finishes.
     */
    /*public Command ClosePickupZoneRow() {
        return new SequentialGroup(
             new ParallelRaceGroup(
                     //new FollowPath(zonePickupEnd).raceWith(new Delay(2.0))'
                     new Delay(2.0),
                    IntakeUntilFull()
            )
        );
    } */ // Parallel Race Group was not working as expected, so we are using the IntakeUntilFull command instead

    /**
     * Follows zonePickupEnd path and finishes when EITHER:
     * - The path completes, OR
     * - The intake has collected 3 balls
     */
    public Command FollowZonePickupEndUntilFull() {
        return new LambdaCommand()
                .setStart(() -> PedroComponent.follower().followPath(zonePickupEnd))
                .setIsDone(() -> (IntakeWithSensorsSubsystem.INSTANCE.getBallCount() >= 3) || (!PedroComponent.follower().isBusy()))
                .setInterruptible(true)
                .named("FollowZonePickupEndUntilFull");
    }

    /**
     * Follows zonePickup path and finishes when EITHER:
     * - The path completes, OR
     * - The intake has collected 3 balls
     */
    public Command FollowZonePickupUntilFull() {
        return new LambdaCommand()
                .setStart(() -> PedroComponent.follower().followPath(zonePickup))
                .setIsDone(() -> (IntakeWithSensorsSubsystem.INSTANCE.getBallCount() >= 3) || (!PedroComponent.follower().isBusy()))
                .setInterruptible(true)
                .named("FollowZonePickupUntilFull");
    }

    public Command CloseShootZoneRow() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(zonePickupShoot),
                        new SequentialGroup(
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(0.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(0.0),
                AutonDumbShootWithHybridBoost(),
                new Delay(0.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0))
        );
    }

    /** Hit the gate while enabling intake - intake starts when we begin following this path */
    public Command HitGateWithIntake() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(hitGateBeforePickup),
                        new SequentialGroup(
                                new Delay(0.3),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                )
        );
    }

    /** Follow the intake-from-gate path (intake stays running to keep collecting balls) */
    public Command IntakeFromGate() {
        return new SequentialGroup(
                new FollowPath(intakeFromGate)
        );
    }

    /** Race version: finish intake-from-gate as soon as path completes or ball count reaches 3. */
    public Command IntakeFromGateRace() {
        return new SequentialGroup(
                new ParallelRaceGroup(
                        new FollowPath(intakeFromGate),
                        new WaitUntil(() -> IntakeWithSensorsSubsystem.INSTANCE.getBallCount() >= 3)
                )
        );
    }

    /** Return from gate intake to far scoring position, spin up shooter on the way, then shoot */
    public Command IntakeFromGateShoot() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(intakeFromGateShoot),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(0.0),
                AutonDumbShootWithHybridBoost(),
                new Delay(0.20)       );
    }

    /** Hit the duplicated DirectGate path while enabling intake. */
    public Command HitDirectGateWithIntake() {
        return new SequentialGroup(
                new ParallelGroup(
                                new FollowPath(intakeFromDirectGate),
                                new SequentialGroup(
                                        new Delay(0.3),
                                        new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                                        new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                        new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0))
                                )
                )

        );
    }

    /** Follow the duplicated DirectGate intake path. */
    public Command IntakeFromDirectGate() {
        return new SequentialGroup(
                new FollowPath(intakeFromDirectGate)
        );
    }

    /** Race version: finish direct-gate intake as soon as path completes or ball count reaches 3. */
    public Command IntakeFromDirectGateRace() {
        return new SequentialGroup(
                new ParallelRaceGroup(
                        new FollowPath(intakeFromDirectGate),
                        new WaitUntil(() -> IntakeWithSensorsSubsystem.INSTANCE.getBallCount() >= 3)
                )
        );
    }

    /** Return from duplicated DirectGate intake path, spin up shooter on the way, then shoot. */
    public Command IntakeFromDirectGateShoot() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(intakeFromDirectGateShoot),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(0.0),
                AutonDumbShootWithHybridBoost(),
                new Delay(0.20)
        );
    }


    public Command CloseMoveOffLine() {
        return new SequentialGroup(
                new FollowPath(moveOffLineClose)
        );
    }

    public Command CloseMoveOffLineToLever() {
        return new SequentialGroup(
               new FollowPath(moveOffLineLever)


        );
    }

    public Command CloseMoveOffLineToLeverExtraTime() {
        return new SequentialGroup(
                new FollowPath(moveOffLineLeverExtraTime)
        );
    }

    public Command CloseMoveOffLineAfterPickup() {
        return new SequentialGroup(
                new FollowPath(moveOffLineAfterPickup)
        );
    }

}

