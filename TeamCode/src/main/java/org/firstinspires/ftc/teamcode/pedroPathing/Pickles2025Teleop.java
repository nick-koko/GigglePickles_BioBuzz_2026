package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.util.CsvLogger;
import java.io.File;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.utils.LoopTimer;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.GlobalRobotData;
import org.firstinspires.ftc.teamcode.subsystems.LEDControlSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeWithSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shot.ShotCalibrationTable;
import org.firstinspires.ftc.teamcode.subsystems.shot.ShotSample;
import org.firstinspires.ftc.teamcode.subsystems.shot.ShotSolution;
import org.firstinspires.ftc.teamcode.subsystems.shot.ShootingZones;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@TeleOp(name = "Pickles 2025 Teleop", group = "Comp")
public class    Pickles2025Teleop extends NextFTCOpMode {
    public Pickles2025Teleop() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(
                        ShooterSubsystem.INSTANCE,
                        IntakeWithSensorsSubsystem.INSTANCE,
                        LEDControlSubsystem.INSTANCE,
                        TurretSubsystem.INSTANCE
                ),
                BulkReadComponent.INSTANCE
        );
    }

    public static Pose startingPoseBlue = new Pose(31.5, 134.0, Math.toRadians(270)); //See ExampleAuto to understand how to use this
    public static Pose startingPoseRed = startingPoseBlue.mirror(144);
    public static Pose startingPose = startingPoseBlue;

    // logging variables
    private CsvLogger turretLogger;
    private CsvLogger shotInfoLogger;
    private CsvLogger sotmLogger;
    private CsvLogger dumbShootRpmLogger;
    private CsvLogger shotTuningLogger;
    private long lastTurretLogMs = 0L;
    private long lastSotmLogMs = 0L;
    private long lastDumbShootRpmLogMs = 0L;
    public static boolean ENABLE_TURRET_LOGGING = false;
    public static long TURRET_LOG_PERIOD_MS = 25;
    // Turret diagnostics for steady-state tracking/stiction investigation.
    public static double TURRET_DIAG_TARGET_STABLE_DELTA_DEG = 0.15;
    public static double TURRET_DIAG_STICTION_ERROR_MIN_DEG = 0.8;
    public static double TURRET_DIAG_STICTION_VELOCITY_MAX_DEG_PER_SEC = 2.0;
    public static double TURRET_DIAG_SERVO_DEADBAND_POS_GUESS = 0.0010;
    public static long TURRET_DIAG_STICTION_MIN_TIME_MS = 120;
    public static boolean ENABLE_SOTM_LOGGING = true;
    public static long SOTM_LOG_PERIOD_MS = 25;
    public static boolean ENABLE_DUMBSHOOT_RPM_LOGGING = false;
    public static long DUMBSHOOT_RPM_LOG_PERIOD_MS = 5;
    public static boolean DUMBSHOOT_RPM_LOG_EVERY_LOOP = true;
    public static long DUMBSHOOT_RPM_LOG_DURATION_MS = 1500;
    public static int BURST_PROFILE_ID = 0;
    public static long SHOOT_BLOCK_LED_STROBE_MS = 180;
    public static double SHOOT_GATE_AT_SPEED_TOLERANCE_RPM = 150.0;
    public static double SHOOT_GATE_MIN_TARGET_RPM = 500.0;
    public static double FAR_SHOT_RPM_THRESHOLD = 4000.0;
    public static double FAR_SHOT_AT_SPEED_TOLERANCE_RPM = 75.0;
    // Duration of the "too close to shoot" red LED warning flash, shown only when the
    // driver actually presses a fire button while inside minDisatanceForShooting. Keeps
    // the ball-count color visible the rest of the time so the driver still knows how
    // many balls are loaded when intaking near the goal.
    public static long TOO_CLOSE_FIRE_ATTEMPT_FLASH_MS = 1500L;
    // Period for the red strobe used during the "too close" warning flash. Shorter =
    // more urgent visual; longer = easier to distinguish from other strobes.
    public static long TOO_CLOSE_FIRE_ATTEMPT_FLASH_PERIOD_MS = 180L;
    // Shot tuning mode: same driving/aiming flow, but manual shooter controls and KEEP/IGNORE labels.
    public static boolean SHOT_TUNING_MODE = false;
    public static boolean ENABLE_SHOT_TUNING_LOGGING = false;
    public static double SHOT_TUNING_RPM_STEP = 25.0;
    public static double SHOT_TUNING_HOOD_STEP = 0.005;
    public static double SHOT_TUNING_TARGET_STEP_IN = 1.0;
    public static double SHOT_TUNING_TARGET_X_IN = 144.0;
    public static double SHOT_TUNING_TARGET_Y_IN = 137.0;

    public static double RED_AIM_X_OFFSET_IN = 4.;  //was 5
    public static double RED_AIM_Y_OFFSET_IN = 0.0;

    public static double BLUE_AIM_X_OFFSET_IN = 4.; //was 4

    public static double SOTM_DRIVER_STICK_DEADBAND = 0.08;
    public static double SOTM_STOP_LEAD_SPEED_IN_PER_SEC = 8.0;
    // Seed RPM applied on match start when SHOT_TUNING_MODE is enabled. Normal
    // mode leaves targetRPM at 0 until the table lookup computes one, but
    // tuning mode disables that auto path, so without a seed here the first
    // right-bumper spin-up would call spinUp(0) and silently disable the
    // flywheel. Operator can still trim up/down from this baseline with
    // gamepad 2 D-pad up/down.
    public static double SHOT_TUNING_DEFAULT_RPM = 3500.0;
    // Calibration-session controls. Purely informational telemetry overlay: the
    // robot never auto-drives. Driver steers to each pinned waypoint by eye,
    // uses CAL_drift_in / CAL_locked feedback to park, then tunes and fires.
    public static int CAL_POINT_INDEX = 0;
    public static boolean CAL_SESSION_ACTIVE = false;
    public static double CAL_WAYPOINT_LOCKED_RADIUS_IN = 2.0;
    public static boolean ENABLE_SHOT_INFO_LOGGING = false;
    public static long SHOT_INFO_LOG_TIMEOUT_MS = 3000;
    // Dumbshoot-specific logging window based on observed burst timing:
    // shot1 < 200 ms, shot2 ~150 ms later, shot3 usually ~200 ms later.
    // Keep a small margin for loop/sensor jitter.
    public static long SHOT_INFO_DUMBSHOOT_TIMEOUT_MS = 900;
    // Hold logging at least this long for dumbshoot to avoid early-finalize truncation.
    public static long SHOT_INFO_DUMBSHOOT_MIN_CAPTURE_MS = 750;
    // If shot2 was seen but shot3 is missing, wait this long after shot2 before finalizing.
    public static long SHOT_INFO_DUMBSHOOT_AFTER_SECOND_TIMEOUT_MS = 325;
    // After seeing the final expected shot edge, wait a short grace period before finalize.
    public static long SHOT_INFO_EXPECTED_SHOTS_FINALIZE_GRACE_MS = 90;
    private long logStartMs = 0L;
    private int shotSequenceIdCounter = 0;
    private boolean shotLogSequenceActive = false;

    public static double SHOOTER_IDLE_RPM = 3200.0;
    public static double CLOSE_IDLE_RPM = 3200.0;
    public static double FAR_IDLE_RPM = 3800.0;

    public static boolean START_IN_FAR_MODE = false;

    public static double BLUE_LEVER_ANGLE_DEG = 144.0;
    public static double RED_LEVER_ANGLE_DEG = 180 - BLUE_LEVER_ANGLE_DEG;
    public double leverAngleDeg = BLUE_LEVER_ANGLE_DEG;

    public static long LIMELIGHT_MISSING_LED_STROBE_MS = 250L;

    private boolean farModeEnabled = START_IN_FAR_MODE;
    private boolean defenseModeEnabled = false;
    private boolean prevGamepad2DpadLeft = false;
    private boolean prevGamepad2DpadRight = false;
    private boolean prevGamepad2A = false;
    private boolean prevGamepad2Y = false;

    private Pose lastTurretAimPose = null;
    private Pose lastRawPoseForSlipCheck = null;
    private long lastOdoSlipMs = 0L;

    public static double TURRET_ODOMETRY_SLIP_JUMP_IN = 5.0;
    public static long TURRET_ODOMETRY_SLIP_HOLD_MS = 350L;

    private boolean shooterIdleMode = false;
    private long shotSequenceStartMs = 0L;
    private int shotSequenceId = 0;
    private int shotSequenceStartBallCount = 0;
    private int shotSequenceExpectedShots = 0;
    private String shotSequenceReason = "";
    private int shotSequenceBurstProfileId = 0;
    private double shotSequenceStartTargetRpm = 0.0;
    private double shotSequenceStartHoodPos = 0.0;
    private long shotSequenceStartBoostDelayMs = 0L;
    private double shotSequenceStartPreBoostAmount = 0.0;
    private double shotSequenceStartBoostMult1 = 0.0;
    private double shotSequenceStartBoostMult2 = 0.0;
    private int shotSequenceLinkedDumbShootRpmSequenceId = -1;
    private boolean shotSequenceStartBb0 = false;
    private boolean shotSequenceStartBb1 = false;
    private boolean shotSequenceStartBb2 = false;
    private int shotBb0FallCount = 0;
    private int shotBb0RiseCount = 0;
    private int shotBb1FallCount = 0;
    private int shotBb1RiseCount = 0;
    private int shotBb2FallCount = 0;
    private int shotBb2RiseCount = 0;
    private final long[] shotBb0FallMs = new long[] {-1L, -1L, -1L};
    private final long[] shotBb0RiseMs = new long[] {-1L, -1L, -1L};
    private final long[] shotBb1FallMs = new long[] {-1L, -1L, -1L};
    private final long[] shotBb1RiseMs = new long[] {-1L, -1L, -1L};
    private final long[] shotBb2FallMs = new long[] {-1L, -1L, -1L};
    private final long[] shotBb2RiseMs = new long[] {-1L, -1L, -1L};
    private final long[] shotIntervalMs = new long[] {-1L, -1L, -1L};
    private int shotBb2ClearLoopCountTotal = 0;
    private int shotBb2ClearStreakCurrent = 0;
    private int shotBb2ClearStreakMax = 0;
    private boolean prevBb0ForShotLog = false;
    private boolean prevBb1ForShotLog = false;
    private boolean prevBb2ForShotLog = false;
    private boolean bbPrevInitializedForShotLog = false;
    private boolean dumbShootRpmLogActive = false;
    private long dumbShootRpmLogStartMs = 0L;
    private int dumbShootRpmLogSequenceId = 0;
    private int dumbShootRpmSequenceCounter = 0;
    private int dumbShootRpmLogExpectedShots = 0;
    private int dumbShootRpmLinkedShotInfoSequenceId = -1;
    private int dumbShootRpmBurstProfileId = 0;
    private int dumbShootRpmBb0RiseCount = 0;
    private int dumbShootRpmBb0FallCount = 0;
    private int dumbShootRpmBb1RiseCount = 0;
    private int dumbShootRpmBb1FallCount = 0;
    private int dumbShootRpmBb2RiseCount = 0;
    private int dumbShootRpmBb2FallCount = 0;
    private boolean dumbShootRpmPrevBbInitialized = false;
    private boolean dumbShootRpmPrevBb0 = false;
    private boolean dumbShootRpmPrevBb1 = false;
    private boolean dumbShootRpmPrevBb2 = false;
    private int shotTuningShotIdCounter = 0;
    private boolean shotTuningPendingLabel = false;
    private int shotTuningPendingShotId = 0;
    private long shotTuningPendingFireMs = 0L;
    private int shotTuningPendingStartBallCount = 0;
    private double shotTuningPendingTargetRpm = 0.0;
    private double shotTuningPendingHoodPos = 0.0;
    private double shotTuningPendingRpm1 = 0.0;
    private double shotTuningPendingRpm2 = 0.0;
    private double shotTuningPendingBotX = 0.0;
    private double shotTuningPendingBotY = 0.0;
    private double shotTuningPendingBotHeadingDeg = 0.0;
    private double shotTuningPendingTargetX = 0.0;
    private double shotTuningPendingTargetY = 0.0;
    private double shotTuningPendingOdoDistance = 0.0;
    private double shotTuningPendingTurretTargetDeg = 0.0;
    private double shotTuningPendingTurretMeasuredDeg = 0.0;
    private int shotTuningPendingCalPointIndex = -1;
    private double shotTuningPendingTargetPointX = Double.NaN;
    private double shotTuningPendingTargetPointY = Double.NaN;
    private String shotTuningPendingZone = "OUT";
    private double shotTuningPendingTableRpm = 0.0;
    private double shotTuningPendingTableHood = 0.0;
    private double shotTuningPendingTableAimX = 0.0;
    private double shotTuningPendingTableAimY = 0.0;
    private boolean prevCalPointDec = false;
    private boolean prevCalPointInc = false;
    private boolean prevTuningTargetDpadUp = false;
    private boolean prevTuningTargetDpadDown = false;
    private boolean prevTuningTargetDpadLeft = false;
    private boolean prevTuningTargetDpadRight = false;
    private boolean prevTuningRpmDpadUp = false;
    private boolean prevTuningRpmDpadDown = false;
    private boolean prevTuningHoodDpadLeft = false;
    private boolean prevTuningHoodDpadRight = false;
    private String shotGateLedState = "NONE";
    private double prevTurretLogTargetDeg = Double.NaN;
    private double prevTurretLogMeasuredDeg = Double.NaN;
    private double prevTurretLogServoCommandDeg = Double.NaN;
    private double prevTurretLogServoPos = Double.NaN;
    private long prevTurretLogMs = 0L;
    private long turretStictionCandidateStartMs = 0L;

    //public static Pose redShootingTarget = new Pose(127.63, 130.35, Math.toRadians(36));
    public static Pose redShootingTarget = new Pose(144, 136, Math.toRadians(36));
    public static Pose blueShootingTarget = redShootingTarget.mirror();
    // Field width fed to Pose.mirror(width) when flipping blue-authored
    // calibration lookups onto the red side. Matches the mirror width used
    // everywhere else in the project (startingPoseRed, autopath mirroring,
    // etc.); exposed as public static so it can be adjusted per-event if a
    // field is built slightly over- or under-sized.
    public static double FIELD_WIDTH_IN = 144.0;
    // Per-zone global trim on top of the calibration table. This lets the team
    // nudge all A-zone ("far") or B-zone ("near") shots together at an event
    // without editing every row in ShotCalibrationTable. RPM is additive and
    // aim X/Y are field-inch deltas from the table's returned target point.
    public static double SHOT_ZONE_A_RPM_OFFSET = 0.0;
    public static double SHOT_ZONE_A_TARGET_X_OFFSET_IN = 0.0;
    public static double SHOT_ZONE_A_TARGET_Y_OFFSET_IN = 0.0;
    public static double SHOT_ZONE_B_RPM_OFFSET = 0.0;
    public static double SHOT_ZONE_B_TARGET_X_OFFSET_IN = 0.0;
    public static double SHOT_ZONE_B_TARGET_Y_OFFSET_IN = 0.0;

    private boolean rpmLimitEnabled = true;
    private boolean prevX = false;
    private boolean prevX2 = false;
    Pose MT1PedroPose = new Pose();
    private static final int LIMELIGHT_VISION_HISTORY_CAPACITY = 8;
    private final double[] limelightVisionDxHistory = new double[LIMELIGHT_VISION_HISTORY_CAPACITY];
    private final double[] limelightVisionDyHistory = new double[LIMELIGHT_VISION_HISTORY_CAPACITY];
    private final double[] limelightVisionDhHistoryDeg = new double[LIMELIGHT_VISION_HISTORY_CAPACITY];
    private int limelightVisionHistoryNextIdx = 0;
    private int limelightVisionHistoryFill = 0;
    private double limelightVisionBiasXIn = 0.0;
    private double limelightVisionBiasYIn = 0.0;
    private double limelightVisionBiasHeadingDeg = 0.0;
    private double limelightVisionLastNudgeXIn = 0.0;
    private double limelightVisionLastNudgeYIn = 0.0;
    private double limelightVisionLastNudgeHeadingDeg = 0.0;
    private int limelightVisionLoopsSinceApply = Integer.MAX_VALUE / 2;
    private int limelightVisionConsecutiveAccepts = 0;

    // Adjust these from Panels at runtime
    public static boolean hold = false;
    public static boolean prevHold = false;
    public static boolean SHOW_SMOOTHED = false;
    public static int SMOOTH_WINDOW = 8;           // samples for moving average
    private final LoopTimer timer = new LoopTimer();
    private double rpmShooter1Smoothed = 0.0;
    private double rpmShooter2Smoothed = 0.0;
    private double rpmOuttakeSmoothed = 0.0;
    private double[] windowShooter1;
    private int wIdxShooter1 = 0;
    private int wFillShooter1 = 0;
    private double[] windowShooter2;
    private int wIdxShooter2 = 0;
    private int wFillShooter2 = 0;
    private double[] windowOuttake;
    private int wIdxOuttake = 0;
    private int wFillOuttake = 0;

    private Pose shootingTargetLocation;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    boolean goToTargetAngle;
    double targetAngleDeg = -135.0;
    double targetAngleRad;
    double propAngleGain = -0.5;
    public static double shooterTargetkP = 0.0165;

    double minAnglePower = 0.075;
    double maxRotate = 0.8;
    double angleAllianceOffset = 0.0;
    double drivePower = 1.0;
    public static double normDrivePower = 1;
    public static double slowedDrivePower = 0.5;
    private Limelight3A limelight;
    private double cameraHeightFromTags = 18.25;

    private double xOffset;
    private double yOffset;
    private double distanceLL;
    private double ODODistance;
    private double yDesired = 17;

    private double areaOffset;
    public static boolean testShooter = false;
    public static double targetRPM = 0;
    public static double targetInitialRPM = 0;
    public static double minDisatanceForShooting = 0.0; //was 42", but set to 0 for testing

    // SOTM Stage 3 (turret + distance-based time of flight)
    public static boolean SOTM_ENABLED = true;
    public static double SOTM_MIN_SPEED_IN_PER_SEC = 2.0;
    public static double SOTM_LEAD_GAIN = 1.25;
    public static double SOTM_MAX_LEAD_IN = 36.0;
    public static double SOTM_ANGULAR_LEAD_GAIN = 0.6;
    public static double SOTM_OMEGA_FILTER_ALPHA = 0.2;
    // Separate turret-lag feedforward layer (independent from SOTM lead-point math).
    // Equivalent concept to: turretTarget += angularVelocity * kVF.
    public static boolean SOTM_TURRET_LAG_COMP_ENABLED = true;
    public static double SOTM_TURRET_LAG_COMP_SEC = 0.15;
    public static double SOTM_TURRET_LAG_COMP_MAX_DEG = 25.0;
    // When true, keep SOTM solution (turret + ballistic distance) live all the time.
    public static boolean SOTM_ALWAYS_TRACK_TARGETS = true;
    // Legacy toggle (intentionally unused now): flywheel enable is controlled only by
    // shooterFollowEnabled (left-bumper latch or 3-ball auto-enable).
    public static boolean SOTM_ALWAYS_PRIME_SHOOTER = false;
    public static double SOTM_FIRE_AIM_TOLERANCE_DEG = 4.0;
    public static double SOTM_FIRE_MAX_TURRET_SPEED_DEG_PER_SEC = 220.0;
    public static boolean SOTM_REQUIRE_TURRET_READY_FOR_FIRE = false;
    public static double SOTM_REACHABILITY_TOLERANCE_DEG = 1.0;
    public static boolean SOTM_USE_TOF_LOOKUP = true;
    public static double SOTM_MECHANICAL_DELAY_SEC = 0.08;
    public static double SOTM_BALL_TRANSFER_TIME_SEC = 0.0;
    public static double SOTM_ESTIMATED_BALL_SPEED_IN_PER_SEC = 300.0;
    private static final double[] SOTM_TOF_DISTANCE_IN = {60.0, 80.0, 100.0, 120.0};
    private static final double[] SOTM_TOF_FLIGHT_TIME_SEC = {0.15, 0.20, 0.27, 0.35};

    public static boolean SOTM_RPM_DIRECTION_COMP_ENABLED = true;
    public static double SOTM_RPM_TOWARD_GOAL_MULT = 0.80;
    public static double SOTM_RPM_AWAY_FROM_GOAL_MULT = 1.20;
    public static double SOTM_RPM_DIRECTION_MIN_RADIAL_SPEED_IN_PER_SEC = 15.0;

    // Hold gamepad1.x to cap RPM to the “top of triangle” limit.
    public static boolean SOTM_TOP_TRIANGLE_RPM_LIMIT_ENABLED = true;
    public static double SOTM_TOP_TRIANGLE_RPM_LIMIT = 3600.0;

    // Limelight translation-only fusion. MT2 is the default source because the
    // current validation log shows it clustering tighter than MT1. Heading
    // stays on the robot yaw sensor by default; enable heading only after
    // logging proves it is helpful.
    public static boolean LIMELIGHT_VISION_BLEND_ENABLED = true;
    public static int LIMELIGHT_VISION_BLEND_SOURCE = 2; // 1 = MT1, 2 = MT2
    public static boolean LIMELIGHT_VISION_BLEND_USE_HEADING = false;
    public static int LIMELIGHT_VISION_BIAS_WINDOW = 5;
    // Plausibility caps. Previously 10" translation / 6" history delta were
    // too tight: log analysis showed ~45% of valid-pose frames being rejected
    // on the translation cap whenever the robot had already drifted past 10"
    // (the exact case we want to correct) and the history-delta cap blocked
    // the big-innovation recovery right after a bump. Widening these lets
    // real-world drift and collision offsets flow through, while the
    // consecutive-accepts + history median still protect us against single
    // noisy frames.
    public static double LIMELIGHT_VISION_MAX_TRANSLATION_ERROR_IN = 24.0;
    public static double LIMELIGHT_VISION_MAX_HEADING_ERROR_DEG = 10.0;
    public static double LIMELIGHT_VISION_MAX_DELTA_FROM_HISTORY_IN = 10.0;
    public static double LIMELIGHT_VISION_BLEND_ALPHA_MOVING = 0.05;
    public static double LIMELIGHT_VISION_BLEND_ALPHA_SLOW = 0.15;
    public static double LIMELIGHT_VISION_BLEND_ALPHA_STATIONARY = 0.35;
    // Escalated alpha when the raw vision delta is large AND speed/omega are
    // under the stationary-or-slow caps. This is the "recover from a bump"
    // path: when the robot has been physically displaced, we want to pull
    // odometry back toward vision faster than the normal gentle trim. The
    // consecutive-accepts gate still guarantees the large error is repeatable
    // before we act on it.
    public static double LIMELIGHT_VISION_BLEND_LARGE_ERROR_IN = 5.0;
    public static double LIMELIGHT_VISION_BLEND_ALPHA_LARGE_ERROR = 0.6;
    public static double LIMELIGHT_VISION_SPEED_SLOW_IN_PER_SEC = 8.0;
    public static double LIMELIGHT_VISION_SPEED_STATIONARY_IN_PER_SEC = 2.5;
    public static double LIMELIGHT_VISION_OMEGA_SLOW_DEG_PER_SEC = 35.0;
    public static double LIMELIGHT_VISION_OMEGA_STATIONARY_DEG_PER_SEC = 10.0;
    // Throttle + streak gates on the actual setPose() call. Even after the raw
    // sample is "accepted" we only re-localize Pedro when:
    //   - at least MIN_LOOPS_BETWEEN_APPLIES have passed since the last apply
    //     (avoid fighting Pinpoint's internal integration every loop), AND
    //   - at least MIN_CONSECUTIVE_ACCEPTS recent samples in a row passed the
    //     plausibility + history gates (avoid reacting to a single noisy tag),
    //     AND
    //   - current speed/omega are under the hard apply caps (never slam the
    //     pose while the robot is actively driving fast).
    // MT2 only produces a valid sample on frames where the goal tag is in
    // view (~5% of loops in recent logs), so a streak of 3 rarely completes
    // before the tag drops out again. A streak of 2 is the minimum that still
    // guarantees the sample wasn't a single-frame glitch.
    public static int LIMELIGHT_VISION_BLEND_MIN_LOOPS_BETWEEN_APPLIES = 12;
    public static int LIMELIGHT_VISION_BLEND_MIN_CONSECUTIVE_ACCEPTS = 2;
    public static double LIMELIGHT_VISION_BLEND_MAX_APPLY_SPEED_IN_PER_SEC = 16.0;
    public static double LIMELIGHT_VISION_BLEND_MAX_APPLY_OMEGA_DEG_PER_SEC = 60.0;

    // Auto-stop shooter timeout after starting a dumbShoot burst (ms)
    public static long DUMBSHOOT_SHOOTER_TIMEOUT_MS = 1200;
    // After the shooter timeout fires we stop every motor, but we do NOT immediately
    // re-validate ball count + re-enter intakeForward. Doing so in the same loop tick
    // means a ball that was still physically mid-transit through the transfer (or
    // just bounced off the decelerating flywheel) is instantly detected at sensor2,
    // ballCount jumps to 1, m3Enabled goes false, and intakeForward commands m3 to
    // M3_HOLD_RPM_OCCUPIED (-20 RPM reverse) - which actively pulls a not-yet-shot
    // ball back down the transfer and strands it. We instead hold all motors at zero
    // for this settle window so the last ball has time to fully exit (or fully drop
    // to a sensor) before validation runs.
    public static long DUMBSHOOT_POST_SHOT_SETTLE_MS = 250;

    public static double shooterHoodPos = 0;
    private boolean hasResults = false;
    private boolean selectAllianceSide = false;

    private boolean shoot = false;

    private double turretStartupServoCommandAngleDeg = Double.NaN;

    private boolean adjustLimelight = false;
    private boolean adjustOdo = true;
    // Edge-detect left trigger so we only start one single-ball feed per press
    private boolean prevLeftTriggerActive = false;
    // Latches a single-shot request until shooter RPM is in range
    private boolean singleShotPending = false;
    // State for auto-stopping shooter after dumbShoot
    private boolean dumbShootTimerActive = false;
    private long dumbShootStartTimeMs = 0L;
    // Settle phase: true while we have already stopped shooter+intake after a
    // dumbShoot burst but are waiting DUMBSHOOT_POST_SHOT_SETTLE_MS before
    // validating ball count and re-entering intakeForward.
    private boolean dumbShootSettleActive = false;
    private long dumbShootSettleStartMs = 0L;
    // True while the robot is inside minDisatanceForShooting. Used as a fire-gate by
    // shot-trigger code further down the loop; LED presentation is independent.
    private boolean tooCloseWarningActive = false;
    // Wall-clock timestamp (ms) at which the "too close" red LED flash should end.
    // 0L means not flashing. The flash is only armed by a fire-button rising edge while
    // tooCloseWarningActive; outside the flash window, the ball-count color is shown.
    private long tooCloseWarningFlashEndMs = 0L;
    // Rising-edge detector for the SOTM fire request so the flash only re-arms on a
    // fresh button press, not every loop the button is held.
    private boolean prevSotmFireRequestActive = false;
    // Latches flywheel tracking mode so RPM keeps following location without holding a button.
    private boolean shooterFollowEnabled = false;
    private boolean prevGamepad1LeftBumper = false;
    private double sotmFilteredOmegaRadPerSec = 0.0;
    private boolean sotmOmegaFilterInitialized = false;
    // True only after Start is pressed; prevents flywheel spin-up during Init.
    private boolean matchHasStarted = false;
    private boolean turretStartupFromAuton = false;

    private double turretStartupExpectedAngleDeg = TurretSubsystem.STARTUP_EXPECTED_TURRET_ANGLE_DEGREES;
    // Teleop init time on the field is often under a second, so use a short
    // startup settle here and fall back to a short sampled absolute read at Start
    // if init did not finish calibration in time.
    public static long TELEOP_TURRET_STARTUP_SETTLE_MS = 250L;
    public static int TELEOP_TURRET_START_SAMPLE_COUNT = 5;
    public static long TELEOP_TURRET_START_SAMPLE_INTERVAL_MS = 10L;
    public static double FAR_NO_BOOST_RPM_THRESHOLD = 4000.0;
    public static long FAR_NO_BOOST_BETWEEN_SHOTS_MS = 150L;

    @Override
    public void onInit() {
        matchHasStarted = false;
        targetRPM = 0.0;
        ShooterSubsystem.INSTANCE.stop();
        shooterFollowEnabled = false;
        dumbShootTimerActive = false;
        dumbShootSettleActive = false;
        singleShotPending = false;
        shotTuningPendingLabel = false;
        shotTuningShotIdCounter = 0;
        shooterHoodPos = 0.0;
        prevTuningTargetDpadUp = false;
        prevTuningTargetDpadDown = false;
        prevTuningTargetDpadLeft = false;
        prevTuningTargetDpadRight = false;
        prevTuningRpmDpadUp = false;
        prevTuningRpmDpadDown = false;
        prevTuningHoodDpadLeft = false;
        prevTuningHoodDpadRight = false;
        prevCalPointDec = false;
        prevCalPointInc = false;
        shotGateLedState = "NONE";
        hold = false;
        prevHold = false;
        shooterIdleMode = true;
        ShooterSubsystem.INSTANCE.resetHybridShotFeedBoostController();
        resetLimelightVisionBlendState();
        farModeEnabled = START_IN_FAR_MODE;
        defenseModeEnabled = false;
        SHOOTER_IDLE_RPM = CLOSE_IDLE_RPM;
        targetRPM = SHOOTER_IDLE_RPM;
        shooterIdleMode = true;
        shooterFollowEnabled = false;

        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        TurretSubsystem.INSTANCE.inTeleop = true;
        TurretSubsystem.INSTANCE.turretCenterDelayComplete = false;
//        TurretSubsystem.INSTANCE.resetEncoderCounts();
//
//        turretStartupFromAuton = (GlobalRobotData.endAutonPose != null) && (GlobalRobotData.hasAutonRun);
//        if (turretStartupFromAuton) {
//            startingPose = GlobalRobotData.endAutonPose;
//            GlobalRobotData.hasAutonRun = false;
//            TurretSubsystem.INSTANCE.publicTurretEncoderOffset(GlobalRobotData.endAutonTurretAngleDegrees);
//        } else {
//            TurretSubsystem.INSTANCE.resetEncoderCounts();
//            selectAllianceSide = true;
//        }
//
//        turretStartupExpectedAngleDeg =
//                (turretStartupFromAuton && Double.isFinite(GlobalRobotData.endAutonTurretAngleDegrees))
//                        ? GlobalRobotData.endAutonTurretAngleDegrees
//                        : TurretSubsystem.STARTUP_EXPECTED_TURRET_ANGLE_DEGREES;
//
//        turretStartupServoCommandAngleDeg =
//                (turretStartupFromAuton && Double.isFinite(GlobalRobotData.endAutonTurretServoCommandAngleDegrees))
//                    ? GlobalRobotData.endAutonTurretServoCommandAngleDegrees : turretStartupExpectedAngleDeg;
//        if (turretStartupFromAuton) {
//            TurretSubsystem.INSTANCE.beginStartupCalibrationWithoutCentering();
//        } else {
//            TurretSubsystem.INSTANCE.beginStartupCentering();
//        }
        turretStartupFromAuton = (GlobalRobotData.endAutonPose != null) && GlobalRobotData.hasAutonRun;

        if (turretStartupFromAuton) {
            startingPose = GlobalRobotData.endAutonPose.copy();

            // Consume auton handoff so it cannot be reused by accident later.
            GlobalRobotData.hasAutonRun = false;
            GlobalRobotData.endAutonPose = null;
            GlobalRobotData.endAutonTurretAngleDegrees = Double.NaN;
            GlobalRobotData.endAutonTurretServoCommandAngleDegrees = Double.NaN;
        } else {
            TurretSubsystem.INSTANCE.resetEncoderCounts();
            selectAllianceSide = true;
        }

// TeleOp should NOT use the auton measured turret angle as an encoder offset.
// Always let the turret absolute encoder/startup calibration define turret zero.
        turretStartupExpectedAngleDeg = TurretSubsystem.STARTUP_EXPECTED_TURRET_ANGLE_DEGREES;
        turretStartupServoCommandAngleDeg = turretStartupExpectedAngleDeg;

        //TurretSubsystem.INSTANCE.beginStartupCentering();
        if (turretStartupFromAuton) {
            TurretSubsystem.INSTANCE.beginStartupCalibrationWithoutCentering();
        } else {
            TurretSubsystem.INSTANCE.beginStartupCentering();
        }
        TurretSubsystem.INSTANCE.beginStartupCentering();

        //PedroComponent.follower().setStartingPose(startingPose);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        windowShooter1 = new double[Math.max(1, SMOOTH_WINDOW)];
        windowShooter2 = new double[Math.max(1, SMOOTH_WINDOW)];
        windowOuttake = new double[Math.max(1, SMOOTH_WINDOW)];

        LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.GREEN);
        TurretSubsystem.INSTANCE.setPeriodicAbsoluteEncoderReadEnabled(ENABLE_TURRET_LOGGING);

        // Carry over ball count from auton if available, then consume the handoff so a
        // subsequent teleop init (without running auton first) doesn't re-apply a stale
        // value. GlobalRobotData is a process-wide static singleton; without clearing it
        // here, running "auton -> teleop -> teleop" would wrongly set ballCount to the
        // auton-ending count on the second teleop init.
        // IntakeWithSensorsSubsystem.initialize() already forces ballCount=0, so the
        // no-auton path is now explicitly safe (no else branch needed).
        if (GlobalRobotData.endAutonBallCount >= 0) {
            IntakeWithSensorsSubsystem.INSTANCE.setBallCount(GlobalRobotData.endAutonBallCount);
            GlobalRobotData.endAutonBallCount = -1;
        }

        /* pathChain = () -> PedroComponent.follower().pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(PedroComponent.follower()::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(PedroComponent.follower()::getHeading, Math.toRadians(45), 0.8))
                .build(); */


        turretLogger = new CsvLogger("pickles2025_turret");
        turretLogger.start(
                "t_ms," +
                        "match_t_ms," +
                        "bot_x," +
                        "bot_y," +
                        "bot_heading_deg," +
                        "shoot_target_x," +
                        "shoot_target_y," +
                        "field_angle_deg," +
                        "angle_error_deg," +
                        "turret_target_deg," +
                        "turret_target_corrected_deg," +
                        "turret_commanded_deg," +
                        "turret_servo_command_deg," +
                        "turret_measured_deg," +
                        "turret_error_deg," +
                        "turret_outer_trim_deg," +
                        "turret_command_diff_deg," +
                        "turret_rate_step_deg," +
                        "turret_left_cmd_deg," +
                        "turret_right_cmd_deg," +
                        "turret_servo_pos," +
                        "turret_left_servo_pos," +
                        "turret_right_servo_pos," +
                        "turret_quad_raw_deg," +
                        "turret_quad_offset_deg," +
                        "turret_encoder_vel_deg_s," +
                        "turret_abs_raw_deg," +
                        "turret_abs_deg," +
                        "turret_abs_delta_deg," +
                        "turret_learned_servo_offset_deg," +
                        "turret_abs_startup_error_deg," +
                        "turret_encoder_ticks," +
                        "turret_log_dt_ms," +
                        "turret_target_delta_deg," +
                        "turret_measured_delta_deg," +
                        "turret_servo_command_delta_deg," +
                        "turret_servo_pos_delta," +
                        "turret_trim_as_servo_delta_pos," +
                        "turret_cmd_above_deadband_guess," +
                        "turret_target_stable," +
                        "turret_limit_clipped," +
                        "turret_stiction_suspect," +
                        "turret_ready," +
                        "driving_cmd," +
                        "strafe_cmd," +
                        "rotate_cmd," +
                        "ododistance," +
                        "x_offset," +
                        "y_offset," +
                        "has_results," +
                        "loop_time_ms"
        );

        if (ENABLE_SOTM_LOGGING) {
            sotmLogger = new CsvLogger("pickles2025_sotm");
            sotmLogger.start(
                    "t_ms," +
                            "match_t_ms," +
                            "sotm_fire_request_active," +
                            "sotm_control_active," +
                            "right_bumper_active," +
                            "right_trigger_active," +
                            "bot_x," +
                            "bot_y," +
                            "bot_heading_deg," +
                            "mt1_valid," +
                            "mt1_x," +
                            "mt1_y," +
                            "mt1_heading_deg," +
                            "mt2_valid," +
                            "mt2_x," +
                            "mt2_y," +
                            "mt2_heading_deg," +
                            "ll_blend_enabled," +
                            "ll_blend_source," +
                            "ll_pose_valid," +
                            "ll_pose_accepted," +
                            "ll_pose_applied," +
                            "ll_gate_reason," +
                            "ll_consecutive_accepts," +
                            "ll_loops_since_apply," +
                            "ll_raw_dx_in," +
                            "ll_raw_dy_in," +
                            "ll_raw_dist_in," +
                            "ll_raw_heading_err_deg," +
                            "ll_bias_x_in," +
                            "ll_bias_y_in," +
                            "ll_bias_heading_deg," +
                            "ll_nudge_x_in," +
                            "ll_nudge_y_in," +
                            "ll_nudge_heading_deg," +
                            "ll_blend_alpha," +
                            "target_x," +
                            "target_y," +
                            "field_angle_deg," +
                            "angle_error_deg," +
                            "real_distance_in," +
                            "sotm_valid," +
                            "sotm_lead_applied," +
                            "sotm_speed_in_s," +
                            "sotm_vx_in_s," +
                            "sotm_vy_in_s," +
                            "sotm_omega_raw_deg_s," +
                            "sotm_omega_deg_s," +
                            "sotm_total_tof_s," +
                            "sotm_lead_x_in," +
                            "sotm_lead_y_in," +
                            "sotm_radial_vel_in_s," +
                            "sotm_effective_dist_in," +
                            "sotm_ballistic_dist_in," +
                            "sotm_turret_aim_deg," +
                            "sotm_turret_lag_comp_deg," +
                            "turret_target_deg," +
                            "turret_measured_deg," +
                            "turret_ready," +
                            "sotm_turret_gate," +
                            "sotm_turret_goal_error_deg," +
                            "sotm_turret_constraint_error_deg," +
                            "sotm_turret_reachable," +
                            "sotm_turret_speed_deg_s," +
                            "sotm_turret_speed_gate," +
                            "sotm_fire_gate," +
                            "sotm_can_shoot_gate," +
                            "shooter_target_rpm," +
                            "shooter_rpm1," +
                            "shooter_rpm2," +
                            "shooter_at_speed_75," +
                            "shooter_battery_v," +
                            "shooter_battery_v_filtered," +
                            "shooter_voltage_comp_gain," +
                            "shooter_cmd_pre_vcomp," +
                            "shooter_cmd_post_vcomp," +
                            "shooter_cmd_saturated," +
                            "dumbshoot_timer_active," +
                            "ball_count," +
                            "intake_m1_ticks," +
                            "intake_m3_ticks," +
                            "hold_state," +
                            "drive_cmd," +
                            "strafe_cmd," +
                            "rotate_cmd," +
                            "loop_time_ms"
            );
        } else {
            sotmLogger = null;
        }

        logStartMs = 0L;
        lastTurretLogMs = 0L;
        lastSotmLogMs = 0L;
        lastDumbShootRpmLogMs = 0L;
        prevTurretLogTargetDeg = Double.NaN;
        prevTurretLogMeasuredDeg = Double.NaN;
        prevTurretLogServoCommandDeg = Double.NaN;
        prevTurretLogServoPos = Double.NaN;
        prevTurretLogMs = 0L;
        turretStictionCandidateStartMs = 0L;
        dumbShootRpmLogActive = false;
        dumbShootRpmLogStartMs = 0L;
        dumbShootRpmLogSequenceId = 0;
        dumbShootRpmSequenceCounter = 0;
        dumbShootRpmLogExpectedShots = 0;
        dumbShootRpmLinkedShotInfoSequenceId = -1;
        dumbShootRpmBurstProfileId = 0;
        dumbShootRpmBb0RiseCount = 0;
        dumbShootRpmBb0FallCount = 0;
        dumbShootRpmBb1RiseCount = 0;
        dumbShootRpmBb1FallCount = 0;
        dumbShootRpmBb2RiseCount = 0;
        dumbShootRpmBb2FallCount = 0;
        dumbShootRpmPrevBbInitialized = false;
        shotTuningPendingLabel = false;
        shotTuningShotIdCounter = 0;

        if (ENABLE_SHOT_INFO_LOGGING) {
            shotInfoLogger = new CsvLogger("pickles2025_shot_breakbeam");
            shotInfoLogger.start(
                    "t_ms," +
                            "match_t_ms," +
                            "sequence_id," +
                            "trigger_reason," +
                            "start_ball_count," +
                            "expected_shots," +
                            "start_bb0," +
                            "start_bb1," +
                            "start_bb2," +
                            "bb0_fall_1_ms," +
                            "bb0_rise_1_ms," +
                            "bb0_fall_2_ms," +
                            "bb0_rise_2_ms," +
                            "bb0_fall_3_ms," +
                            "bb0_rise_3_ms," +
                            "bb1_fall_1_ms," +
                            "bb1_rise_1_ms," +
                            "bb1_fall_2_ms," +
                            "bb1_rise_2_ms," +
                            "bb1_fall_3_ms," +
                            "bb1_rise_3_ms," +
                            "bb2_fall_1_ms," +
                            "bb2_rise_1_ms," +
                            "bb2_fall_2_ms," +
                            "bb2_rise_2_ms," +
                            "bb2_fall_3_ms," +
                            "bb2_rise_3_ms," +
                            "shot1_interval_ms," +
                            "shot2_interval_ms," +
                            "shot3_interval_ms," +
                            "bb2_clear_gap_1to2_ms," +
                            "bb2_clear_gap_2to3_ms," +
                            "bb2_clear_loops_total," +
                            "bb2_clear_streak_max_loops," +
                            "bb0_fall_count," +
                            "bb0_rise_count," +
                            "bb1_fall_count," +
                            "bb1_rise_count," +
                            "bb2_fall_count," +
                            "bb2_rise_count," +
                            "end_reason," +
                            "duration_ms," +
                            "burst_profile_id," +
                            "start_target_rpm," +
                            "start_hood_pos," +
                            "start_boost_delay_ms," +
                            "start_pre_boost_amount," +
                            "start_boost_mult1," +
                            "start_boost_mult2," +
                            "linked_dumbshoot_rpm_sequence_id"
            );
        } else {
            shotInfoLogger = null;
        }

        if (ENABLE_DUMBSHOOT_RPM_LOGGING) {
            dumbShootRpmLogger = new CsvLogger("pickles2025_dumbshoot_rpm");
            dumbShootRpmLogger.start(
                    "t_ms," +
                            "match_t_ms," +
                            "sequence_id," +
                            "shot_info_sequence_id," +
                            "burst_profile_id," +
                            "t_since_start_ms," +
                            "expected_shots," +
                            "dumbshoot_timer_active," +
                            "shooter_target_rpm," +
                            "shooter_rpm1," +
                            "shooter_rpm2," +
                            "shooter_avg_rpm," +
                            "shooter_rpm_delta_avg," +
                            "shooter_power1," +
                            "shooter_power2," +
                            "shooter_at_speed_75," +
                            "shooter_battery_v," +
                            "shooter_battery_v_filtered," +
                            "shooter_voltage_comp_gain," +
                            "shooter_cmd_pre_vcomp," +
                            "shooter_cmd_post_vcomp," +
                            "shooter_cmd_saturated," +
                            "boost_active," +
                            "second_boost_active," +
                            "boost_override," +
                            "shooter_boost_mult1," +
                            "shooter_boost_mult2," +
                            "hybrid_feed_boost_active," +
                            "hybrid_phase," +
                            "hybrid_t_since_shot_feed_start_ms," +
                            "hybrid_expected_contact_ms," +
                            "hybrid_preboost_amount_active," +
                            "hybrid_last_advance_reason," +
                            "hybrid_last_advance_rel_ms," +
                            "shooter_rpm_filtered_baseline," +
                            "shooter_rpm_derivative_rpm_per_sec," +
                            "rpm_at_shot_sequence_start," +
                            "rpm_delta_from_target," +
                            "rpm_delta_from_baseline," +
                            "rpm_drop_candidate_target_delta," +
                            "rpm_drop_candidate_baseline_delta," +
                            "rpm_drop_candidate_derivative," +
                            "shooter_current_a1," +
                            "shooter_current_a2," +
                            "shooter_current_avg_a," +
                            "shooter_current_filtered_baseline_a," +
                            "shooter_current_derivative_a_per_sec," +
                            "shooter_current_at_shot_sequence_start_a," +
                            "shooter_current_delta_from_baseline_a," +
                            "current_spike_candidate_baseline_delta," +
                            "current_spike_candidate_derivative," +
                            "bb0," +
                            "bb1," +
                            "bb2," +
                            "bb0_rise_edge," +
                            "bb0_fall_edge," +
                            "bb1_rise_edge," +
                            "bb1_fall_edge," +
                            "bb2_rise_edge," +
                            "bb2_fall_edge," +
                            "bb0_rise_count_total," +
                            "bb0_fall_count_total," +
                            "bb1_rise_count_total," +
                            "bb1_fall_count_total," +
                            "bb2_rise_count_total," +
                            "bb2_fall_count_total," +
                            "ball_count," +
                            "loop_time_ms"
            );
        } else {
            dumbShootRpmLogger = null;
        }

        if (ENABLE_SHOT_TUNING_LOGGING) {
            shotTuningLogger = new CsvLogger("pickles2025_shot_tuning");
            shotTuningLogger.start(
                    "t_ms," +
                            "match_t_ms," +
                            "shot_id," +
                            "label," +
                            "label_reason," +
                            "fire_t_ms," +
                            "t_since_fire_ms," +
                            "start_ball_count," +
                            "target_x," +
                            "target_y," +
                            "bot_x_fire," +
                            "bot_y_fire," +
                            "bot_heading_deg_fire," +
                            "ododistance_fire," +
                            "target_rpm_fire," +
                            "hood_pos_fire," +
                            "rpm1_fire," +
                            "rpm2_fire," +
                            "target_rpm_label," +
                            "hood_pos_label," +
                            "rpm1_label," +
                            "rpm2_label," +
                            "turret_target_deg_fire," +
                            "turret_measured_deg_fire," +
                            "turret_target_deg_label," +
                            "turret_measured_deg_label," +
                            "boost_active_label," +
                            "second_boost_active_label," +
                            "boost_override_label," +
                            "cal_point_index," +
                            "target_point_x," +
                            "target_point_y," +
                            "zone," +
                            "table_rpm_fire," +
                            "table_hood_fire," +
                            "table_aim_x_fire," +
                            "table_aim_y_fire"
            );
        } else {
            shotTuningLogger = null;
        }

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void onWaitForStart() {
//        boolean turretStartupCalibrated = !turretStartupFromAuton && TurretSubsystem.INSTANCE.updateStartupCalibrationFromExpected(
//                turretStartupExpectedAngleDeg,
//                TELEOP_TURRET_STARTUP_SETTLE_MS
//        );

        if (gamepad1.dpad_left) {
            farModeEnabled = false;
            SHOOTER_IDLE_RPM = CLOSE_IDLE_RPM;
            SOTM_TOP_TRIANGLE_RPM_LIMIT_ENABLED = true;
        }

        if (gamepad1.dpad_right) {
            farModeEnabled = true;
            SHOOTER_IDLE_RPM = FAR_IDLE_RPM;
            SOTM_TOP_TRIANGLE_RPM_LIMIT_ENABLED = false;
        }

        telemetry.addData("Shot Strategy", farModeEnabled ? "FAR / 3800 idle" : "CLOSE / 3200 idle");

        if (selectAllianceSide) {
            if (gamepad1.xWasPressed()) {
                GlobalRobotData.allianceSide = GlobalRobotData.COLOR.BLUE;
                startingPose = startingPoseBlue;
                leverAngleDeg = BLUE_LEVER_ANGLE_DEG;
            } else if (gamepad1.bWasPressed()) {
                GlobalRobotData.allianceSide = GlobalRobotData.COLOR.RED;
                startingPose = startingPoseRed;
                leverAngleDeg = RED_LEVER_ANGLE_DEG;

            }

            telemetry.addLine("Hello Pickle of the robot");
            telemetry.addLine("This is an Mr. Todone Speaking,");
            telemetry.addLine("----------------------------------------------");
            if (GlobalRobotData.allianceSide == GlobalRobotData.COLOR.BLUE) {
                telemetry.addLine("Favorite fruit: Blueberries!!! (Blue)");
            } else {
                telemetry.addLine("Favorite fruit: Raspberries!!! (Red)");
            }
            //telemetry.addData("turretStartupCal", turretStartupCalibrated);
            telemetry.addData("turretStartupState", TurretSubsystem.INSTANCE.getStartupCalibrationStateName());

            telemetry.update();

        }
    }

    @Override
    public void onStartButtonPressed() {
        matchHasStarted = true;
        //TurretSubsystem.INSTANCE.beginStartupCentering();
        TurretSubsystem.INSTANCE.beginStartupCalibrationWithoutCentering();
        TurretSubsystem.INSTANCE.turretResetDelayTotalTime = 0.0;

        if (TurretSubsystem.INSTANCE.turretCenterDelayComplete) {
            if (!TurretSubsystem.INSTANCE.isStartupCalibrationComplete()) {
                TurretSubsystem.INSTANCE.forceStartupCalibrationFromExpectedSampled(
                        turretStartupExpectedAngleDeg,
                        TELEOP_TURRET_START_SAMPLE_COUNT,
                        TELEOP_TURRET_START_SAMPLE_INTERVAL_MS
                );
            }
        }
//        if (!turretStartupFromAuton) {
//            TurretSubsystem.INSTANCE.publicTurretEncoderOffset(GlobalRobotData.endAutonTurretAngleDegrees);
//        }

//       if (!TurretSubsystem.INSTANCE.isStartupCalibrationComplete()) {
//            if (turretStartupFromAuton) {
//                TurretSubsystem.INSTANCE.forceStartupCalibrationFromEstimatedAngleAndServoCommandSampled(
//                        turretStartupExpectedAngleDeg,
//                        turretStartupServoCommandAngleDeg,
//                        TELEOP_TURRET_START_SAMPLE_COUNT,
//                        TELEOP_TURRET_START_SAMPLE_INTERVAL_MS
//                );
//            }
//            else {
//                TurretSubsystem.INSTANCE.forceStartupCalibrationFromExpectedSampled(
//                        turretStartupExpectedAngleDeg,
//                        TELEOP_TURRET_START_SAMPLE_COUNT,
//                        TELEOP_TURRET_START_SAMPLE_INTERVAL_MS
//                );
//            }
//        }
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        PedroComponent.follower().setStartingPose(startingPose);

        PedroComponent.follower().startTeleopDrive();

        if (GlobalRobotData.allianceSide == GlobalRobotData.COLOR.RED) {
            shootingTargetLocation = redShootingTarget;
        } else {
            shootingTargetLocation = blueShootingTarget;
        }
        SHOT_TUNING_TARGET_X_IN = shootingTargetLocation.getX();
        SHOT_TUNING_TARGET_Y_IN = shootingTargetLocation.getY();
        sotmOmegaFilterInitialized = false;
        sotmFilteredOmegaRadPerSec = 0.0;
        prevTurretLogTargetDeg = Double.NaN;
        prevTurretLogMeasuredDeg = Double.NaN;
        prevTurretLogServoCommandDeg = Double.NaN;
        prevTurretLogServoPos = Double.NaN;
        prevTurretLogMs = 0L;
        turretStictionCandidateStartMs = 0L;
        dumbShootRpmLogActive = false;
        dumbShootRpmLogStartMs = 0L;
        dumbShootRpmLogSequenceId = 0;
        dumbShootRpmSequenceCounter = 0;
        dumbShootRpmLogExpectedShots = 0;
        dumbShootRpmLinkedShotInfoSequenceId = -1;
        dumbShootRpmBurstProfileId = 0;
        dumbShootRpmBb0RiseCount = 0;
        dumbShootRpmBb0FallCount = 0;
        dumbShootRpmBb1RiseCount = 0;
        dumbShootRpmBb1FallCount = 0;
        dumbShootRpmBb2RiseCount = 0;
        dumbShootRpmBb2FallCount = 0;
        dumbShootRpmPrevBbInitialized = false;
        lastDumbShootRpmLogMs = 0L;
        shotTuningPendingLabel = false;
        prevTuningTargetDpadUp = false;
        prevTuningTargetDpadDown = false;
        prevTuningTargetDpadLeft = false;
        prevTuningTargetDpadRight = false;
        prevTuningRpmDpadUp = false;
        prevTuningRpmDpadDown = false;
        prevTuningHoodDpadLeft = false;
        prevTuningHoodDpadRight = false;
        prevCalPointDec = false;
        prevCalPointInc = false;
        shotGateLedState = "NONE";
        shooterHoodPos = ShooterSubsystem.INSTANCE.getShooterHoodPosition();
        SHOOTER_IDLE_RPM = CLOSE_IDLE_RPM;
        targetRPM = SHOOTER_IDLE_RPM;
        shooterIdleMode = true;
        shooterFollowEnabled = false;
        ShooterSubsystem.INSTANCE.spinUp(targetRPM);

        // In shot-tuning mode the auto-RPM path (ShotCalibrationTable lookup)
        // is gated off so the operator can trim RPM with the D-pad. Without
        // seeding a baseline here the first right-bumper press would call
        // spinUp(0) and disable the flywheel, which has been observed as
        // "flywheel won't start in tuning mode". Seed a sensible baseline so
        // the very first spin-up is usable, then let the operator trim from
        // there.
        if (SHOT_TUNING_MODE) {
            targetRPM = SHOT_TUNING_DEFAULT_RPM;
        }

        logStartMs = System.currentTimeMillis();
    }

    @Override
    public void onUpdate() {
        if (gamepad2.rightBumperWasPressed()) {
            BLUE_AIM_X_OFFSET_IN -= 3;
            RED_AIM_X_OFFSET_IN -= 3;

        }
        else if(gamepad2.leftBumperWasPressed()) {
            BLUE_AIM_X_OFFSET_IN += 3;
            RED_AIM_X_OFFSET_IN += 3;
        }
//        if (gamepad1.yWasPressed()) {
//            shooterIdleMode = !shooterIdleMode;
//        }
        //Call this once per loop
        timer.start();
        long nowMs = System.currentTimeMillis();
        TurretSubsystem.INSTANCE.setPeriodicAbsoluteEncoderReadEnabled(ENABLE_TURRET_LOGGING);

        // Phase 1: dumbShoot burst has reached its timeout. Stop shooter + intake and
        // start a settle window. Do NOT validate ball count or call intakeForward yet -
        // a ball may still be mid-transit and an immediate intakeForward would command
        // m3 to reverse at M3_HOLD_RPM_OCCUPIED, pulling it back.
        if (dumbShootTimerActive &&
                nowMs - dumbShootStartTimeMs >= DUMBSHOOT_SHOOTER_TIMEOUT_MS) {
            if (shooterIdleMode) {
                targetRPM = SHOOTER_IDLE_RPM;
                shooterFollowEnabled = true;   // keep it spinning
            } else {
                targetRPM = 0.0;
                shooterFollowEnabled = false;
                ShooterSubsystem.INSTANCE.stop();
            }
            IntakeWithSensorsSubsystem.INSTANCE.stop();
            dumbShootTimerActive = false;
            shooterFollowEnabled = false;
            ShooterSubsystem.INSTANCE.resetHybridShotFeedBoostController();
            dumbShootSettleActive = true;
            dumbShootSettleStartMs = nowMs;
        }

        // Phase 2: settle window has elapsed. Now read sensors (they're stable) and
        // re-arm the intake. If the last ball is genuinely stuck at sensor2 after the
        // settle, validation will correctly set ballCount=1 and intakeForward will hold
        // it; if the ball fully exited, sensors are clear and ballCount stays 0 so the
        // next intake cycle starts clean.
        if (dumbShootSettleActive &&
                nowMs - dumbShootSettleStartMs >= DUMBSHOOT_POST_SHOT_SETTLE_MS) {
            IntakeWithSensorsSubsystem.INSTANCE.validateBallCountAfterShoot();
            IntakeWithSensorsSubsystem.INSTANCE.intakeForward();
            dumbShootSettleActive = false;
        }

        Pose rawBotPose = PedroComponent.follower().getPose();
        Pose currentBotPose = getTurretSafePose(rawBotPose);
        double botHeadingRad = currentBotPose.getHeading();
        double botxvalue = currentBotPose.getX(); //gettingxvalue :D
        double botyvalue = currentBotPose.getY(); //gettingyvalue :D

        // MegaTag 2 expects the robot yaw in FTC-standard field coordinates.
        // In this codebase, Pedro heading is offset from FTC-standard by -90 deg,
        // so the inverse transform for the yaw we feed Limelight is +90 deg.
        double limelightRobotYawDeg = normalizeDegrees(Math.toDegrees(botHeadingRad) + 90.0);
        limelight.updateRobotOrientation(limelightRobotYawDeg);

        boolean dpadUpPressed = gamepad2.dpad_up;
        if (dpadUpPressed && !prevX) {
            farModeEnabled = true;
            SHOOTER_IDLE_RPM = FAR_IDLE_RPM;
            targetRPM = SHOOTER_IDLE_RPM;
            shooterIdleMode = true;
            shooterFollowEnabled = false;
            ShooterSubsystem.INSTANCE.spinUp(targetRPM);
            SOTM_TOP_TRIANGLE_RPM_LIMIT_ENABLED = false;
        }
        prevX = dpadUpPressed;

        boolean dpadDownPressed = gamepad2.dpad_down;
        if (dpadDownPressed && !prevX2) {
            farModeEnabled = false;
            SHOOTER_IDLE_RPM = CLOSE_IDLE_RPM;
            targetRPM = SHOOTER_IDLE_RPM;
            shooterIdleMode = true;
            shooterFollowEnabled = false;
            ShooterSubsystem.INSTANCE.spinUp(targetRPM);
            SOTM_TOP_TRIANGLE_RPM_LIMIT_ENABLED = true;
        }
        prevX2 = dpadDownPressed;

        boolean dpadLeftPressed = gamepad2.dpad_left;
        if (dpadLeftPressed && !prevGamepad2DpadLeft) {
            farModeEnabled = false;
            SHOOTER_IDLE_RPM = CLOSE_IDLE_RPM;
            SOTM_TOP_TRIANGLE_RPM_LIMIT_ENABLED = false;
        }
        prevGamepad2DpadLeft = dpadLeftPressed;

        boolean dpadRightPressed = gamepad2.dpad_right;
        if (dpadRightPressed && !prevGamepad2DpadRight) {
            farModeEnabled = true;
            SHOOTER_IDLE_RPM = FAR_IDLE_RPM;
            SOTM_TOP_TRIANGLE_RPM_LIMIT_ENABLED = false;
        }
        prevGamepad2DpadRight = dpadRightPressed;

        LLResult result = limelight.getLatestResult();
        boolean limelightMissing = (result == null);

        if (limelightMissing) {
            LEDControlSubsystem.INSTANCE.startStrobe(
                    LEDControlSubsystem.LedColor.OFF,
                    LEDControlSubsystem.LedColor.WHITE,
                    Math.max(50L, LIMELIGHT_MISSING_LED_STROBE_MS)
            );
        }
        boolean mt1Valid = false;
        double mt1PedroX = Double.NaN;
        double mt1PedroY = Double.NaN;
        double mt1PedroHeadingDeg = Double.NaN;
        boolean mt2Valid = false;
        double mt2PedroX = Double.NaN;
        double mt2PedroY = Double.NaN;
        double mt2PedroHeadingDeg = Double.NaN;
        String limelightBlendSource = "MT2";
        boolean limelightBlendPoseValid = false;
        boolean limelightBlendPoseAccepted = false;
        boolean limelightBlendPoseApplied = false;
        double limelightBlendRawDxIn = Double.NaN;
        double limelightBlendRawDyIn = Double.NaN;
        double limelightBlendRawDistIn = Double.NaN;
        double limelightBlendRawHeadingErrDeg = Double.NaN;
        double limelightBlendAlpha = 0.0;
        // Per-loop reason string surfaced to log + telemetry so the team can see
        // at a glance why a frame was (or was not) accepted/applied. Default is
        // "disabled" until we flow through the gate logic below.
        String limelightBlendGateReason = LIMELIGHT_VISION_BLEND_ENABLED ? "no_tag" : "disabled";
        if (result != null && result.isValid()) {
            xOffset = result.getTx();
            yOffset = result.getTy();
            areaOffset = result.getTa();
            distanceLL = cameraHeightFromTags / (Math.tan(Math.toRadians(yOffset)));

            Pose3D mt1Pose = result.getBotpose();
            if (mt1Pose != null) {
                Pose mt1PedroPoseCandidate = convertLimelightBotposeToPedro(mt1Pose);
                double candidateX = mt1PedroPoseCandidate.getPose().getX();
                double candidateY = mt1PedroPoseCandidate.getPose().getY();
                double candidateHeadingDeg = normalizeDegrees(Math.toDegrees(mt1PedroPoseCandidate.getPose().getHeading()));
                if (Double.isFinite(candidateX) && Double.isFinite(candidateY) && Double.isFinite(candidateHeadingDeg)) {
                    mt1Valid = true;
                    mt1PedroX = candidateX;
                    mt1PedroY = candidateY;
                    mt1PedroHeadingDeg = candidateHeadingDeg;
                    MT1PedroPose = mt1PedroPoseCandidate;
                }
            }

            Pose3D mt2Pose = result.getBotpose_MT2();
            if (mt2Pose != null) {
                Pose mt2PedroPoseCandidate = convertLimelightBotposeToPedro(mt2Pose);
                double candidateX = mt2PedroPoseCandidate.getPose().getX();
                double candidateY = mt2PedroPoseCandidate.getPose().getY();
                double candidateHeadingDeg = normalizeDegrees(Math.toDegrees(mt2PedroPoseCandidate.getPose().getHeading()));
                if (Double.isFinite(candidateX) && Double.isFinite(candidateY) && Double.isFinite(candidateHeadingDeg)) {
                    mt2Valid = true;
                    mt2PedroX = candidateX;
                    mt2PedroY = candidateY;
                    mt2PedroHeadingDeg = candidateHeadingDeg;
                }
            }
        }
        Vector limelightVisionVelocity = PedroComponent.follower().getVelocity();
        double limelightVisionSpeedInPerSec = limelightVisionVelocity.getMagnitude();
        double limelightVisionOmegaDegPerSec = Math.toDegrees(PedroComponent.follower().poseTracker.getAngularVelocity());

        limelightBlendSource = (LIMELIGHT_VISION_BLEND_SOURCE == 1) ? "MT1" : "MT2";
        double selectedVisionX = Double.NaN;
        double selectedVisionY = Double.NaN;
        double selectedVisionHeadingDeg = Double.NaN;
        if (LIMELIGHT_VISION_BLEND_SOURCE == 1) {
            limelightBlendPoseValid = mt1Valid;
            selectedVisionX = mt1PedroX;
            selectedVisionY = mt1PedroY;
            selectedVisionHeadingDeg = mt1PedroHeadingDeg;
        } else {
            limelightBlendPoseValid = mt2Valid;
            selectedVisionX = mt2PedroX;
            selectedVisionY = mt2PedroY;
            selectedVisionHeadingDeg = mt2PedroHeadingDeg;
        }

        if (limelightBlendPoseValid) {
            limelightBlendRawDxIn = selectedVisionX - botxvalue;
            limelightBlendRawDyIn = selectedVisionY - botyvalue;
            limelightBlendRawDistIn = Math.hypot(limelightBlendRawDxIn, limelightBlendRawDyIn);
            limelightBlendRawHeadingErrDeg =
                    normalizeDegrees(selectedVisionHeadingDeg - Math.toDegrees(botHeadingRad));

            boolean translationOk = limelightBlendRawDistIn <= Math.max(0.0, LIMELIGHT_VISION_MAX_TRANSLATION_ERROR_IN);
            boolean headingOk = Math.abs(limelightBlendRawHeadingErrDeg) <=
                    Math.max(0.0, LIMELIGHT_VISION_MAX_HEADING_ERROR_DEG);
            boolean historyOk = true;
            if (limelightVisionHistoryFill >= 3) {
                double historyDx = getRecentVisionHistoryMedian(limelightVisionDxHistory);
                double historyDy = getRecentVisionHistoryMedian(limelightVisionDyHistory);
                double historyDeltaDist = Math.hypot(
                        limelightBlendRawDxIn - historyDx,
                        limelightBlendRawDyIn - historyDy
                );
                historyOk = historyDeltaDist <= Math.max(0.0, LIMELIGHT_VISION_MAX_DELTA_FROM_HISTORY_IN);
            }

            limelightBlendPoseAccepted =
                    LIMELIGHT_VISION_BLEND_ENABLED &&
                            translationOk &&
                            headingOk &&
                            historyOk;

            if (limelightBlendPoseAccepted) {
                pushLimelightVisionHistory(
                        limelightBlendRawDxIn,
                        limelightBlendRawDyIn,
                        limelightBlendRawHeadingErrDeg
                );
                limelightVisionConsecutiveAccepts++;

                boolean isStationary =
                        limelightVisionSpeedInPerSec <= Math.max(0.0, LIMELIGHT_VISION_SPEED_STATIONARY_IN_PER_SEC) &&
                                Math.abs(limelightVisionOmegaDegPerSec) <= Math.max(0.0, LIMELIGHT_VISION_OMEGA_STATIONARY_DEG_PER_SEC);
                boolean isSlow =
                        limelightVisionSpeedInPerSec <= Math.max(0.0, LIMELIGHT_VISION_SPEED_SLOW_IN_PER_SEC) &&
                                Math.abs(limelightVisionOmegaDegPerSec) <= Math.max(0.0, LIMELIGHT_VISION_OMEGA_SLOW_DEG_PER_SEC);
                boolean isLargeError = limelightBlendRawDistIn >=
                        Math.max(0.0, LIMELIGHT_VISION_BLEND_LARGE_ERROR_IN);

                if (isLargeError && (isStationary || isSlow)) {
                    // Big displacement detected and we're not driving hard -
                    // treat this as a bump/collision recovery and pull faster.
                    limelightBlendAlpha = LIMELIGHT_VISION_BLEND_ALPHA_LARGE_ERROR;
                } else if (isStationary) {
                    limelightBlendAlpha = LIMELIGHT_VISION_BLEND_ALPHA_STATIONARY;
                } else if (isSlow) {
                    limelightBlendAlpha = LIMELIGHT_VISION_BLEND_ALPHA_SLOW;
                } else {
                    limelightBlendAlpha = LIMELIGHT_VISION_BLEND_ALPHA_MOVING;
                }
                limelightBlendGateReason = "accept";
            } else {
                limelightVisionConsecutiveAccepts = 0;
                if (!LIMELIGHT_VISION_BLEND_ENABLED) {
                    limelightBlendGateReason = "disabled";
                } else if (!translationOk) {
                    limelightBlendGateReason = "translation";
                } else if (!headingOk) {
                    limelightBlendGateReason = "heading";
                } else {
                    limelightBlendGateReason = "history";
                }
            }
        } else {
            limelightVisionConsecutiveAccepts = 0;
            limelightBlendGateReason = LIMELIGHT_VISION_BLEND_ENABLED ? "no_tag" : "disabled";
        }

        // Apply gate: even when the raw sample is accepted, throttle actual
        // corrections to avoid fighting Pinpoint's internal integrator. Require
        // a streak of good frames and a hard speed cap so we never re-localize
        // while actively driving fast. Heading is only touched if the user
        // explicitly opts in via LIMELIGHT_VISION_BLEND_USE_HEADING.
        if (LIMELIGHT_VISION_BLEND_ENABLED) {
            limelightVisionLoopsSinceApply++;
        }
        boolean throttleOk = limelightVisionLoopsSinceApply >=
                Math.max(1, LIMELIGHT_VISION_BLEND_MIN_LOOPS_BETWEEN_APPLIES);
        boolean streakOk = limelightVisionConsecutiveAccepts >=
                Math.max(1, LIMELIGHT_VISION_BLEND_MIN_CONSECUTIVE_ACCEPTS);
        boolean speedOk = limelightVisionSpeedInPerSec <=
                Math.max(0.0, LIMELIGHT_VISION_BLEND_MAX_APPLY_SPEED_IN_PER_SEC);
        boolean omegaOk = Math.abs(limelightVisionOmegaDegPerSec) <=
                Math.max(0.0, LIMELIGHT_VISION_BLEND_MAX_APPLY_OMEGA_DEG_PER_SEC);

        // Reset per-loop nudge each iteration. Cumulative correction lives in
        // PoseTracker's x/y offsets, not in a code-side variable, so there is
        // no persistent state here that can runaway.
        limelightVisionLastNudgeXIn = 0.0;
        limelightVisionLastNudgeYIn = 0.0;
        limelightVisionLastNudgeHeadingDeg = 0.0;

        // If accepted but blocked from applying, overwrite "accept" reason
        // with the narrower apply-gate that failed, so logs show the true
        // bottleneck rather than silently saying "accept" every frame.
        if (limelightBlendPoseAccepted) {
            if (!throttleOk) {
                limelightBlendGateReason = "throttle";
            } else if (!streakOk) {
                limelightBlendGateReason = "streak";
            } else if (!speedOk) {
                limelightBlendGateReason = "speed";
            } else if (!omegaOk) {
                limelightBlendGateReason = "omega";
            }
        }

        if (LIMELIGHT_VISION_BLEND_ENABLED &&
                limelightBlendPoseAccepted &&
                throttleOk && streakOk && speedOk && omegaOk) {
            // Use Pedro's PoseTracker x/y offset as the persistent correction
            // instead of calling follower.setPose(). setPose() routes through
            // localizer.setPose() and Pinpoint-style localizers track heading
            // as a delta from an internal reference; rewriting that reference
            // every loop nudges the heading state even when we hand back the
            // same value. PoseTracker offsets are pure post-hoc trims added
            // by applyOffset() in getPose(): the localizer keeps ticking
            // undisturbed, and getPose() returns raw + offset.
            double filteredDx = getRecentVisionHistoryMedian(limelightVisionDxHistory);
            double filteredDy = getRecentVisionHistoryMedian(limelightVisionDyHistory);
            double filteredDh = getRecentVisionHistoryMedian(limelightVisionDhHistoryDeg);

            double nudgeX = limelightBlendAlpha * filteredDx;
            double nudgeY = limelightBlendAlpha * filteredDy;
            double nudgeHeadingDeg = LIMELIGHT_VISION_BLEND_USE_HEADING
                    ? limelightBlendAlpha * filteredDh
                    : 0.0;

            if (Math.abs(nudgeX) > 1e-6 || Math.abs(nudgeY) > 1e-6 ||
                    (LIMELIGHT_VISION_BLEND_USE_HEADING && Math.abs(nudgeHeadingDeg) > 1e-6)) {
                PoseTracker pt = PedroComponent.follower().poseTracker;
                pt.setXOffset(pt.getXOffset() + nudgeX);
                pt.setYOffset(pt.getYOffset() + nudgeY);
                if (LIMELIGHT_VISION_BLEND_USE_HEADING) {
                    pt.setHeadingOffset(pt.getHeadingOffset() + Math.toRadians(nudgeHeadingDeg));
                }

                // Refresh local copies so downstream code in this loop sees
                // the corrected pose.
                rawBotPose = PedroComponent.follower().getPose();
                currentBotPose = getTurretSafePose(rawBotPose);
                botxvalue = currentBotPose.getX();
                botyvalue = currentBotPose.getY();
                botHeadingRad = currentBotPose.getHeading();

                limelightVisionLastNudgeXIn = nudgeX;
                limelightVisionLastNudgeYIn = nudgeY;
                limelightVisionLastNudgeHeadingDeg = nudgeHeadingDeg;
                limelightBlendPoseApplied = true;
                limelightVisionLoopsSinceApply = 0;
            }
        }

        // Surface the cumulative PoseTracker offsets (what is actually being
        // added to every getPose() call) for logging / telemetry. This is the
        // ground truth of "how much Limelight correction is currently in
        // effect" because it is the only persistent correction state.
        {
            PoseTracker pt = PedroComponent.follower().poseTracker;
            limelightVisionBiasXIn = pt.getXOffset();
            limelightVisionBiasYIn = pt.getYOffset();
            limelightVisionBiasHeadingDeg = Math.toDegrees(pt.getHeadingOffset());
        }
        ODODistance = PedroComponent.follower().getPose().distanceFrom(shootingTargetLocation);

        double driving = (-gamepad1.left_stick_y) * drivePower;
        double strafe = (-gamepad1.left_stick_x) * drivePower;
        double rotate = (-gamepad1.right_stick_x) * 0.5;
        boolean rightTriggerActive = gamepad1.right_trigger > 0.1;
        boolean rightBumperActive = gamepad1.right_bumper;
        boolean dpadUpActive = !SHOT_TUNING_MODE && gamepad1.dpad_up;
        // Fire request from gamepad 1 right trigger / bumper. We allow this in shot
        // tuning mode too so that the normal dumbShoot path runs (just with the
        // virtual goal target and the operator's manual RPM / hood values).
        boolean sotmFireRequestActive = rightTriggerActive || rightBumperActive;
        boolean sotmControlActive = SOTM_ENABLED && (SOTM_ALWAYS_TRACK_TARGETS || sotmFireRequestActive);

        if (GlobalRobotData.allianceSide == GlobalRobotData.COLOR.BLUE) {
            driving *= -1;
            strafe *= -1;
        }
        if (SHOT_TUNING_MODE) {
            boolean tuningTargetUp = gamepad1.dpad_up;
            boolean tuningTargetDown = gamepad1.dpad_down;
            boolean tuningTargetLeft = gamepad1.dpad_left;
            boolean tuningTargetRight = gamepad1.dpad_right;
            double targetStep = Math.max(0.0, SHOT_TUNING_TARGET_STEP_IN);
            if (tuningTargetUp && !prevTuningTargetDpadUp) {
                SHOT_TUNING_TARGET_Y_IN += targetStep;
            }
            if (tuningTargetDown && !prevTuningTargetDpadDown) {
                SHOT_TUNING_TARGET_Y_IN -= targetStep;
            }
            if (tuningTargetRight && !prevTuningTargetDpadRight) {
                SHOT_TUNING_TARGET_X_IN += targetStep;
            }
            if (tuningTargetLeft && !prevTuningTargetDpadLeft) {
                SHOT_TUNING_TARGET_X_IN -= targetStep;
            }
            prevTuningTargetDpadUp = tuningTargetUp;
            prevTuningTargetDpadDown = tuningTargetDown;
            prevTuningTargetDpadLeft = tuningTargetLeft;
            prevTuningTargetDpadRight = tuningTargetRight;
        }

        // Single source of truth for every non-tuning shot: the calibration
        // table is looked up once per loop from the current robot pose, with
        // zone-wide trims applied in blue-authored calibration space. The
        // resulting rpm / hood / aim
        // values feed RPM selection (here-and-below), the aim plumbed into the
        // turret (immediately below), and hood position (near the end of
        // onUpdate). shotSol is guaranteed non-null whenever SHOT_TUNING_MODE
        // is false because lookup() always returns something.
        ShotSolution tableShotSol = lookupShotForAlliance(botxvalue, botyvalue);
        ShotSolution shotSol = null;
        if (!SHOT_TUNING_MODE) {
            shotSol = tableShotSol;
        }

        double shootTargetX;
        double shootTargetY;
        if (SHOT_TUNING_MODE) {
            shootTargetX = SHOT_TUNING_TARGET_X_IN;
            shootTargetY = SHOT_TUNING_TARGET_Y_IN;
        } else {
            shootTargetX = shotSol.aimX;
            shootTargetY = shotSol.aimY;
        }

        if (!SHOT_TUNING_MODE && GlobalRobotData.allianceSide == GlobalRobotData.COLOR.RED) {
            shootTargetX += RED_AIM_X_OFFSET_IN;
            shootTargetY += RED_AIM_Y_OFFSET_IN;
        }

        if (!SHOT_TUNING_MODE && GlobalRobotData.allianceSide == GlobalRobotData.COLOR.BLUE) {
            shootTargetX -= BLUE_AIM_X_OFFSET_IN;
            shootTargetY += RED_AIM_Y_OFFSET_IN;
        }

        // Vector from robot -> target
        double dx = shootTargetX - botxvalue;
        double dy = shootTargetY - botyvalue;

        // 1) Field-relative angle to the target (0 rad along +X, CCW positive)
        double fieldAngleRad = Math.atan2(dy, dx);
        // - fieldAngleDeg  -> "absolute field heading I want the robot to face"
        double fieldAngleDeg = Math.toDegrees(fieldAngleRad);

        // 2) Robot-relative angle error (how much you need to turn or point the turret)
        double angleErrorRad = normalizeRadians(fieldAngleRad - botHeadingRad);
        // - angleErrorDeg  -> "how many degrees left/right from current heading"
        double angleErrorDeg = Math.toDegrees(angleErrorRad);

        Vector robotVelocity = PedroComponent.follower().getVelocity();

        double sotmVx = robotVelocity.getXComponent();
        double sotmVy = robotVelocity.getYComponent();
        double sotmSpeed = robotVelocity.getMagnitude();

        boolean driverTranslationCommanded =
                Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y) > SOTM_DRIVER_STICK_DEADBAND;

// If driver released the sticks and robot is nearly stopped, do NOT lead.
// This prevents the turret from aiming like the robot is still moving.
        if (!driverTranslationCommanded && sotmSpeed < SOTM_STOP_LEAD_SPEED_IN_PER_SEC) {
            sotmVx = 0.0;
            sotmVy = 0.0;
            sotmSpeed = 0.0;
        }
        double sotmOmegaRawRadPerSec = PedroComponent.follower().poseTracker.getAngularVelocity();
        double sotmOmegaRadPerSec = updateSotmFilteredOmega(sotmOmegaRawRadPerSec);

        SOTMResult sotmResult = calculateSotmResult(
                botxvalue,
                botyvalue,
                botHeadingRad,
                shootTargetX,
                shootTargetY,
                sotmVx,
                sotmVy,
                sotmSpeed,
                sotmOmegaRadPerSec,
                sotmControlActive
        );

        if (!SHOT_TUNING_MODE) {
            boolean activelyShootingOrPreparing =
                    sotmFireRequestActive || gamepad1.left_bumper || shooterFollowEnabled;

            if (activelyShootingOrPreparing) {
                targetRPM = applySotmRpmCompensation(
                        shotSol.rpm,
                        sotmResult.radialVelocityInPerSec,
                        rpmLimitEnabled
                );
            } else if (shooterIdleMode) {
                targetRPM = SHOOTER_IDLE_RPM;
            } else {
                targetRPM = 0.0;
            }
        }

        if (sotmResult.valid && (sotmControlActive || SOTM_ALWAYS_TRACK_TARGETS)) {
            TurretSubsystem.INSTANCE.setTargetAngleFromRobotFrontRelativeDegrees(
                    sotmResult.turretRobotRelativeAimDeg
            );
        } else {
            TurretSubsystem.INSTANCE.setTargetAngleFromRobotFrontRelativeDegrees(angleErrorDeg);
        }

        // Always track with turret. When SOTM is active and moving, this target is lead-compensated.
        //TurretSubsystem.INSTANCE.setTargetAngleFromRobotFrontRelativeDegrees(sotmResult.turretRobotRelativeAimDeg);
        double shooterDistanceForBallistics = sotmResult.distanceForBallisticsInches;
        double turretMeasuredDeg = TurretSubsystem.INSTANCE.getMeasuredAngleDegrees();
        double turretMeasuredVelDegPerSec = TurretSubsystem.INSTANCE.getMeasuredVelocityDegPerSec();
        double turretCommandedTargetDeg = TurretSubsystem.INSTANCE.getTargetAngleDegrees();
        double desiredTurretAimUnwrappedDeg =
                robotFrontRelativeToTurretUnwrappedDegrees(sotmResult.turretRobotRelativeAimDeg);
        double turretGoalErrorDeg =
                smallestWrappedDeltaDegrees(desiredTurretAimUnwrappedDeg, turretMeasuredDeg);
        double turretConstraintErrorDeg =
                smallestWrappedDeltaDegrees(desiredTurretAimUnwrappedDeg, turretCommandedTargetDeg);
        boolean turretTargetReachable =
                Math.abs(turretConstraintErrorDeg) <= Math.max(0.0, SOTM_REACHABILITY_TOLERANCE_DEG);
        boolean turretAimAtGoal =
                Math.abs(turretGoalErrorDeg) <= Math.max(0.0, SOTM_FIRE_AIM_TOLERANCE_DEG);
        boolean turretSpeedGateSatisfied =
                Math.abs(turretMeasuredVelDegPerSec) <= Math.max(0.0, SOTM_FIRE_MAX_TURRET_SPEED_DEG_PER_SEC);
        boolean turretReadyGateSatisfied =
                !SOTM_REQUIRE_TURRET_READY_FOR_FIRE || TurretSubsystem.INSTANCE.isTurretReady();
        boolean turretAimGateSatisfied =
                turretTargetReachable &&
                        turretAimAtGoal &&
                        turretSpeedGateSatisfied &&
                        turretReadyGateSatisfied;
        boolean sotmFireGateSatisfied = !sotmControlActive || sotmResult.valid;
        boolean canShootAtGoal = true;
        //turretAimGateSatisfied && sotmFireGateSatisfied;


        double angletangent = 0;
        double shootingangle = 0;
        // Add location based shooting angle here eventually
        //double shootingangle = Math.toDegrees(Math.atan2(144-botyvalue,botxvalue)
//        if (gamepad1.y) {
//            PedroComponent.follower().setPose(new Pose(71, 8, Math.toRadians(270)));
//        }

        // MT1 emergency relocalization stays available even when SOTM is enabled.
        // This is the rescue path for cases where odometry / heading become bad
        // enough that MT2 translation-only blending can no longer be trusted.
        if (dpadUpActive && result != null && result.isValid() && mt1Valid) {
            PedroComponent.follower().setPose(MT1PedroPose);
            currentBotPose = MT1PedroPose;
            botHeadingRad = MT1PedroPose.getHeading();
            botxvalue = MT1PedroPose.getX();
            botyvalue = MT1PedroPose.getY();
            resetLimelightVisionBlendState();
        }

        // Legacy limelight heading-assist block.
        // Keep it only when SOTM is disabled, otherwise it fights moving-shot turret targeting.
        if (dpadUpActive && !SOTM_ENABLED) {
            adjustLimelight = true;
            if (result != null && result.isValid()) {
                // RPM no longer comes from this path; the shot calibration table
                // drives RPM selection below. This block is kept only for legacy
                // heading-assist turret rotation.

                if (GlobalRobotData.allianceSide == GlobalRobotData.COLOR.RED) {
                    List<LLResultTypes.FiducialResult> tag24Results = result.getFiducialResults().stream()
                            .filter(r -> r.getFiducialId() == 24)
                            .collect(Collectors.toList());

                    if (!tag24Results.isEmpty()) {
                        hasResults = true;
                        double targetX = tag24Results.get(0).getTargetXDegrees();
                        if (targetX != 0.5 && tag24Results.get(0).getTargetYDegrees() > 10) {
                            rotate = (-targetX) * (shooterTargetkP + 0.01);
                            goToTargetAngle = false;
                        }
                        else if(targetX != 0.5){
                            rotate = (-targetX) * (shooterTargetkP + 0.01);
                            goToTargetAngle = false;
                        }
                    } else {
                        hasResults = false;
                    }
                } else {
                    List<LLResultTypes.FiducialResult> tag20Results = result.getFiducialResults().stream()
                            .filter(r -> r.getFiducialId() == 20)
                            .collect(Collectors.toList());

                    if (!tag20Results.isEmpty()) {
                        hasResults = true;
                        double targetX = tag20Results.get(0).getTargetXDegrees();
                        if (targetX != -0.5 && tag20Results.get(0).getTargetYDegrees() > 10) {
                            rotate = (-targetX) * (shooterTargetkP + 0.01);
                            goToTargetAngle = false;
                        }
                        else if(targetX !=  -0.5) {
                            rotate = (-targetX) * (shooterTargetkP + 0.01);
                            goToTargetAngle = false;
                        }
                    } else {
                        hasResults = false;
                    }
                }

                if (hasResults) {
                    LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.WHITE);
//                    if (shoot) {
//                        if (hasResults && yOffset > 9.) {
//                            ShooterSubsystem.INSTANCE.setClosePID();
//                            targetRPM = calculateShooterRPM(yOffset);
//                        } else if (hasResults && yOffset <= 9.) {
//                            ShooterSubsystem.INSTANCE.setFarPID();
//                            targetRPM = 4330;
//                        }
//                        ShooterSubsystem.INSTANCE.spinUp(targetRPM);
                }
//                } else {
//                    LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.GREEN);
//                }

            }
        } else {
            // no valid result or bumper not held
            // Always update the distance flag; fire gates downstream read it regardless
            // of what the LEDs are currently showing.
            tooCloseWarningActive = (ODODistance < minDisatanceForShooting);

            // Arm the red "too close" warning flash only on a fresh fire-button press
            // while too close. Previously the LEDs latched into a persistent red strobe
            // whenever the robot was close to the goal, which hid the ball count when
            // intaking near the target. Now the driver sees the ball count by default,
            // and only gets the red warning when they actively try to shoot.
            long nowMsForLeds = System.currentTimeMillis();
            boolean sotmFireRisingEdge = sotmFireRequestActive && !prevSotmFireRequestActive;
            if (sotmFireRisingEdge && tooCloseWarningActive) {
                LEDControlSubsystem.INSTANCE.startStrobe(
                        LEDControlSubsystem.LedColor.OFF,
                        LEDControlSubsystem.LedColor.RED,
                        Math.max(10L, TOO_CLOSE_FIRE_ATTEMPT_FLASH_PERIOD_MS)
                );
                tooCloseWarningFlashEndMs =
                        nowMsForLeds + Math.max(0L, TOO_CLOSE_FIRE_ATTEMPT_FLASH_MS);
            }

            if (nowMsForLeds < tooCloseWarningFlashEndMs) {
                // Flash in progress; let the strobe animation run. We don't call setBoth
                // here because that would cancel the strobe via
                // LEDControlSubsystem.setBoth() stopping the strobing animation.
            } else {
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
            }
        }

        boolean leftBumperPressedNow = gamepad1.left_bumper;
        boolean leftBumperRisingEdge = leftBumperPressedNow && !prevGamepad1LeftBumper;
        if (!SHOT_TUNING_MODE) {
            if (leftBumperRisingEdge) {
                shooterFollowEnabled = true;
            }
            if (IntakeWithSensorsSubsystem.INSTANCE.getBallCount() >= 3) {
                shooterFollowEnabled = true;
            }
        } else {
            shooterFollowEnabled = false;
        }

        // Continuously read RPM from the calibration table at the current pose
        // and only spin the flywheel when explicitly enabled (left-bumper latch
        // or 3-ball auto-enable). In shot tuning mode we keep the operator's
        // manual d-pad RPM value. SOTM moving-shot compensation is not applied
        // here; the table is evaluated at the current pose. If/when SOTM is
        // revived, it should project a future (x, y) and look that up instead.
//        if (!SHOT_TUNING_MODE) {
//            targetRPM = shotSol.rpm;
//        }

        // Allow RB/RT fire request to actively prime flywheel RPM so shoot gating can clear.
        // This still avoids pre-spinning during Init or idle driving. Runs in both
        // normal and tuning mode so gamepad 1 right trigger / bumper brings the
        // shooter up to targetRPM before firing either way.
        boolean shooterPrimeRequested =
                shooterIdleMode || shooterFollowEnabled || sotmFireRequestActive;
        if (matchHasStarted && shooterPrimeRequested) {
            ShooterSubsystem.INSTANCE.spinUp(targetRPM);
            if (!SHOT_TUNING_MODE) {
                adjustOdo = true;
            }
        }

        if (leftBumperPressedNow || shooterFollowEnabled || sotmControlActive) {
            goToTargetAngle = false;
//        } else if (gamepad1.right_trigger > 0.1) {
//            targetAngleDeg = 180.0 + angleAllianceOffset;
//            goToTargetAngle = true;
//            adjustOdo = false;
//        } else if (gamepad1.left_trigger > 0.1) {
//            targetAngleDeg = -90.0 + angleAllianceOffset;
//            goToTargetAngle = true;
//            adjustOdo = false;
//        } else if (gamepad1.dpad_left) {
//            targetAngleDeg = 90.0 + angleAllianceOffset;
//            goToTargetAngle = true;
//            adjustOdo = false;
//        } else if (gamepad1.dpad_up) {
//            targetAngleDeg = 0.0 + angleAllianceOffset;
//            goToTargetAngle = true;
            //adjustOdo = false;
        } else {
            goToTargetAngle = false;
            adjustOdo = false;
        }

        targetAngleRad = Math.toRadians(targetAngleDeg);

//        if (targetRPM == 0 && IntakeWithSensorsSubsystem.INSTANCE.getBallCount() >= 3) {
//            targetRPM = 3000;
//            ShooterSubsystem.INSTANCE.spinUp(targetRPM);
//        }
        if (gamepad1.a) {
            targetAngleDeg =
                    GlobalRobotData.allianceSide == GlobalRobotData.COLOR.RED
                            ? RED_LEVER_ANGLE_DEG
                            : BLUE_LEVER_ANGLE_DEG;

            targetAngleRad = Math.toRadians(targetAngleDeg);
            goToTargetAngle = true;
            adjustOdo = false;
        } else {
            goToTargetAngle = false;
        }


        //mR. TODONE 😎👌👌
        if (goToTargetAngle) {
            double targetAngleDiff = botHeadingRad - targetAngleRad;
            if (targetAngleDiff > Math.PI) {
                targetAngleDiff = (targetAngleDiff - 2 * (Math.PI));
            } else if (targetAngleDiff < -Math.PI) {
                targetAngleDiff = (2 * (Math.PI) + targetAngleDiff);
            }
            rotate = targetAngleDiff * propAngleGain;
            if (rotate > 0.0) {
                rotate = rotate + minAnglePower;
            } else if (rotate < 0.0) {
                rotate = rotate - minAnglePower;
            }
            rotate = Math.max(Math.min(rotate, maxRotate), -maxRotate);
        }
        if (!automatedDrive) {
            //Make the last parameter false for field-centric

            //This is the normal version to use in the TeleOp
            if (!hold) {
                PedroComponent.follower().setTeleOpDrive(
                        driving,
                        strafe,
                        rotate,
                        false // field Centric
                );
            }
        }

        //Automated PathFollowing
       /* if (gamepad1.aWasPressed()) {
            PedroComponent.follower().followPath(pathChain.get());
            automatedDrive = true;
        }*/


        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !PedroComponent.follower().isBusy())) {
            PedroComponent.follower().startTeleopDrive();
            automatedDrive = false;
        }

        double rpmShooter1 = ShooterSubsystem.INSTANCE.getShooter1RPM();
        double rpmShooter2 = ShooterSubsystem.INSTANCE.getShooter2RPM();
        double rpmOuttake = IntakeWithSensorsSubsystem.INSTANCE.getMotor3RPM();
        double activeShootToleranceRpm =
                targetRPM >= FAR_SHOT_RPM_THRESHOLD
                        ? FAR_SHOT_AT_SPEED_TOLERANCE_RPM
                        : SHOOT_GATE_AT_SPEED_TOLERANCE_RPM;

        boolean shooterAtSpeed75 = isShooterReadyForFeed(
                activeShootToleranceRpm,
                rpmShooter1,
                rpmShooter2
        );

        // Optional simple moving average
        if (SHOW_SMOOTHED) {
            if (windowShooter1.length != Math.max(1, SMOOTH_WINDOW)) {
                // if you change SMOOTH_WINDOW live, re-init the buffer on the fly
                windowShooter1 = new double[Math.max(1, SMOOTH_WINDOW)];
                wIdxShooter1 = 0;
                wFillShooter1 = 0;
                rpmShooter1Smoothed = 0.0;
            }
            windowShooter1[wIdxShooter1] = rpmShooter1;
            wIdxShooter1 = (wIdxShooter1 + 1) % windowShooter1.length;
            if (wFillShooter1 < windowShooter1.length) wFillShooter1++;

            double sum = 0.0;
            for (int i = 0; i < wFillShooter1; i++) sum += windowShooter1[i];
            rpmShooter1Smoothed = sum / wFillShooter1;

            if (windowShooter2.length != Math.max(1, SMOOTH_WINDOW)) {
                // if you change SMOOTH_WINDOW live, re-init the buffer on the fly
                windowShooter2 = new double[Math.max(1, SMOOTH_WINDOW)];
                wIdxShooter2 = 0;
                wFillShooter2 = 0;
                rpmShooter2Smoothed = 0.0;
            }
            windowShooter2[wIdxShooter2] = rpmShooter2;
            wIdxShooter2 = (wIdxShooter2 + 1) % windowShooter2.length;
            if (wFillShooter2 < windowShooter2.length) wFillShooter2++;

            sum = 0.0;
            for (int i = 0; i < wFillShooter2; i++) sum += windowShooter2[i];
            rpmShooter2Smoothed = sum / wFillShooter2;

            if (windowOuttake.length != Math.max(1, SMOOTH_WINDOW)) {
                // if you change SMOOTH_WINDOW live, re-init the buffer on the fly
                windowOuttake = new double[Math.max(1, SMOOTH_WINDOW)];
                wIdxOuttake = 0;
                wFillOuttake = 0;
                rpmOuttakeSmoothed = 0.0;
            }
            windowOuttake[wIdxOuttake] = rpmOuttake;
            wIdxOuttake = (wIdxOuttake + 1) % windowOuttake.length;
            if (wFillOuttake < windowOuttake.length) wFillOuttake++;

            sum = 0.0;
            for (int i = 0; i < wFillOuttake; i++) sum += windowOuttake[i];
            rpmOuttakeSmoothed = sum / wFillOuttake;

        }

        // Graph
        telemetryM.addData("Shooter1_RPM", rpmShooter1);
        telemetryM.addData("Calc Shooter1_RPM", ShooterSubsystem.INSTANCE.getShooter1RpmDelta());
        telemetryM.addData("Shooter2_RPM", rpmShooter2);
        telemetryM.addData("Calc Shooter2_RPM", ShooterSubsystem.INSTANCE.getShooter1RpmDelta());
        telemetryM.addData("Calc Avg_RPM", ShooterSubsystem.INSTANCE.getAverageRpmDelta());
        telemetryM.addData("Outtake_RPM", rpmOuttake);

        if (SHOW_SMOOTHED) {
            telemetryM.addData("Shooter1_RPM (smoothed)", rpmShooter1Smoothed);
            telemetryM.addData("Shooter2_RPM (smoothed)", rpmShooter2Smoothed);
            telemetryM.addData("Outtake_RPM (smoothed)", rpmOuttakeSmoothed);
        }

        boolean bb0 = IntakeWithSensorsSubsystem.INSTANCE.isSensor0Broken();
        boolean bb1 = IntakeWithSensorsSubsystem.INSTANCE.isSensor1Broken();
        boolean bb2 = IntakeWithSensorsSubsystem.INSTANCE.isSensor2Broken();

        updateShotInfoBreakbeamTracking(nowMs, bb0, bb1, bb2);
        maybeLogDumbShootRpmSample(nowMs, bb0, bb1, bb2, rpmShooter1, rpmShooter2);

        if (!SHOT_TUNING_MODE) {
            telemetryM.addData("ballCount", IntakeWithSensorsSubsystem.INSTANCE.getBallCount());
            telemetryM.addData("BB_sensor0", bb0 ? 1 : 0);
            telemetryM.addData("BB_sensor1", bb1 ? 1 : 0);
            telemetryM.addData("BB_sensor2", bb2 ? 1 : 0);
        }

        // =============================================
        // DEBUG: Intake state flags for troubleshooting
        // =============================================


        telemetry.addData("INT_isIntaking", IntakeWithSensorsSubsystem.INSTANCE.isIntakingActive());
        telemetry.addData("INT_shooting", IntakeWithSensorsSubsystem.INSTANCE.isShooting());
        telemetry.addData("theNoteRenyLeft", GlobalRobotData.endAutonTurretAngleDegrees);
        telemetry.addData("INT_singleBallFeedActive", IntakeWithSensorsSubsystem.INSTANCE.isSingleBallFeedActive());
        telemetry.addData("INT_multiSingleShotActive", IntakeWithSensorsSubsystem.INSTANCE.isMultiSingleShotActive());
        telemetry.addData("INT_shootSeqActive", IntakeWithSensorsSubsystem.INSTANCE.isShootSequenceActive());
        telemetry.addData("INT_shotInProgress", IntakeWithSensorsSubsystem.INSTANCE.isShotInProgress());
        telemetry.addData("INT_m1Enabled", IntakeWithSensorsSubsystem.INSTANCE.isMotor1Enabled());
        telemetry.addData("INT_m2Enabled", IntakeWithSensorsSubsystem.INSTANCE.isMotor2Enabled());
        telemetry.addData("INT_m3Enabled", IntakeWithSensorsSubsystem.INSTANCE.isMotor3Enabled());
        telemetry.addData("INT_direction", IntakeWithSensorsSubsystem.INSTANCE.getCurrentDirection());
        telemetry.addData("INT_multiReq", IntakeWithSensorsSubsystem.INSTANCE.getMultiSingleShotRequested());
        telemetry.addData("INT_multiDone", IntakeWithSensorsSubsystem.INSTANCE.getMultiSingleShotCompleted());

        double loopTimeMs = timer.getMs();
        if (SHOT_TUNING_MODE) {
            telemetry.addData("bot_x", String.format("%.1f", botxvalue));
            telemetry.addData("bot_y", String.format("%.1f", botyvalue));
            telemetry.addData("bot_heading_deg", String.format("%.1f", Math.toDegrees(botHeadingRad)));
            telemetry.addData("rpm_target", String.format("%.0f", targetRPM));
            telemetry.addData("hood_target", String.format("%.3f", shooterHoodPos));
            telemetry.addData("target_x", String.format("%.1f", SHOT_TUNING_TARGET_X_IN));
            telemetry.addData("target_y", String.format("%.1f", SHOT_TUNING_TARGET_Y_IN));
            telemetry.addData("turret_deg", String.format("%.1f", TurretSubsystem.INSTANCE.getMeasuredAngleDegrees()));
            telemetry.addData("table_rpm", String.format("%.0f", tableShotSol.rpm));
            telemetry.addData("table_hood", String.format("%.3f", tableShotSol.hoodPos));
            telemetry.addData("table_target", String.format("(%.1f, %.1f)", tableShotSol.aimX, tableShotSol.aimY));
        } else {
            telemetryM.addData("LoopTime_ms", loopTimeMs);
            telemetry.addData("rotate", rotate);
            telemetry.addData("turretTargetDeg", TurretSubsystem.INSTANCE.getTargetAngleDegrees());
            telemetry.addData("turretMeasuredDeg", TurretSubsystem.INSTANCE.getMeasuredAngleDegrees());
            telemetry.addData("fieldAngleDeg", fieldAngleDeg);
            telemetry.addData("angleErrorDeg", angleErrorDeg);
            telemetry.addData("turretReady", TurretSubsystem.INSTANCE.isTurretReady());
            telemetry.addData("boostActive", ShooterSubsystem.INSTANCE.boostActive);
            telemetry.addData("shooterFollowEnabled", shooterFollowEnabled);
            telemetry.addData("SOTM_active", sotmControlActive);
            telemetry.addData("SOTM_valid", sotmResult.valid);
            telemetry.addData("SOTM_leadApplied", sotmResult.leadApplied);
            telemetry.addData("SOTM_speed", sotmResult.speedInPerSec);
            telemetry.addData("SOTM_totalTofSec", sotmResult.totalTimeSeconds);
            telemetry.addData("SOTM_leadX", sotmResult.leadXInches);
            telemetry.addData("SOTM_leadY", sotmResult.leadYInches);
            telemetry.addData("SOTM_radialVel", sotmResult.radialVelocityInPerSec);
            telemetry.addData("SOTM_omegaRawDegS", Math.toDegrees(sotmOmegaRawRadPerSec));
            telemetry.addData("SOTM_omegaFiltDegS", Math.toDegrees(sotmOmegaRadPerSec));
            telemetry.addData("SOTM_effectiveDist", sotmResult.effectiveDistanceInches);
            telemetry.addData("SOTM_turretAimDeg", sotmResult.turretRobotRelativeAimDeg);
            telemetry.addData("SOTM_turretLagCompDeg", sotmResult.turretLagCompensationDeg);
            telemetry.addData("SOTM_turretGate", turretAimGateSatisfied);
            telemetry.addData("SOTM_turretGoalErrDeg", turretGoalErrorDeg);
            telemetry.addData("SOTM_turretConstraintErrDeg", turretConstraintErrorDeg);
            telemetry.addData("SOTM_turretReachable", turretTargetReachable);
            telemetry.addData("SOTM_turretSpeedDegS", turretMeasuredVelDegPerSec);
            telemetry.addData("SOTM_turretSpeedGate", turretSpeedGateSatisfied);
            telemetry.addData("SOTM_fireGate", sotmFireGateSatisfied);
            telemetry.addData("SOTM_canShootGate", canShootAtGoal);
            telemetry.addData("SOTM_shooterAtSpeed75", shooterAtSpeed75);
            telemetry.addData("SHOOT_batteryV", ShooterSubsystem.INSTANCE.getBatteryVoltageRaw());
            telemetry.addData("SHOOT_batteryVFiltered", ShooterSubsystem.INSTANCE.getBatteryVoltageFiltered());
            telemetry.addData("SHOOT_voltageCompGain", ShooterSubsystem.INSTANCE.getVoltageCompGain());
            telemetry.addData("SHOOT_cmdPreVComp", ShooterSubsystem.INSTANCE.getCommandPreVoltageComp());
            telemetry.addData("SHOOT_cmdPostVComp", ShooterSubsystem.INSTANCE.getCommandPostVoltageComp());
            telemetry.addData("SHOOT_cmdSaturated", ShooterSubsystem.INSTANCE.isCommandSaturated());
            telemetry.addData("SHOOT_boostMult1", ShooterSubsystem.INSTANCE.getActiveBoostMultiplier1());
            telemetry.addData("SHOOT_boostMult2", ShooterSubsystem.INSTANCE.getActiveBoostMultiplier2());
            telemetry.addData("SHOOT_burstProfileId", BURST_PROFILE_ID);
            telemetry.addData("HYBRID_feedBoostActive", ShooterSubsystem.INSTANCE.isHybridShotFeedBoostActive());
            telemetry.addData("HYBRID_phase", ShooterSubsystem.INSTANCE.getHybridShotFeedBoostPhaseName());
            telemetry.addData("HYBRID_tSinceShotFeedStartMs",
                    ShooterSubsystem.INSTANCE.getHybridTimeSinceShotFeedStartMs());
            telemetry.addData("HYBRID_expectedContactMs", ShooterSubsystem.INSTANCE.getHybridExpectedContactMsForCurrentPhase());
            telemetry.addData("HYBRID_preBoostAmountActive", ShooterSubsystem.INSTANCE.getActivePreBoostAmount());
            telemetry.addData("HYBRID_lastAdvanceReason", ShooterSubsystem.INSTANCE.getHybridLastAdvanceReason());
            telemetry.addData("HYBRID_lastAdvanceRelMs", ShooterSubsystem.INSTANCE.getHybridLastAdvanceAtRelMs());
            telemetry.addData("SOTM_tooCloseBlock", tooCloseWarningActive);
            telemetry.addData("SOTM_shotGateLedState", shotGateLedState);
            if (shotSol != null) {
                telemetry.addData("TABLE_rpm", String.format("%.0f", shotSol.rpm));
                telemetry.addData("TABLE_hood", String.format("%.3f", shotSol.hoodPos));
                telemetry.addData("TABLE_aim", String.format("(%.1f, %.1f)", shotSol.aimX, shotSol.aimY));
                telemetry.addData("TABLE_nearest_in", String.format("%.2f", shotSol.nearestDistanceIn));
                telemetry.addData("TABLE_extrapolated", shotSol.extrapolated ? "YES" : "no");
            }
            telemetryM.addData("targetRPM", ShooterSubsystem.INSTANCE.getTargetRpm());
        }

        if (ENABLE_TURRET_LOGGING && turretLogger != null) {
            long nowLogMs = System.currentTimeMillis();

            if (nowLogMs - lastTurretLogMs >= TURRET_LOG_PERIOD_MS) {
                lastTurretLogMs = nowLogMs;

                double turretTargetDeg = TurretSubsystem.INSTANCE.getTargetAngleDegrees();
                double turretTargetCorrectedDeg = TurretSubsystem.INSTANCE.getCorrectedTargetAngleDegrees();
                double turretCommandedDeg = TurretSubsystem.INSTANCE.getCommandedAngleDegrees();
                double turretServoCommandDeg = TurretSubsystem.INSTANCE.getServoCommandAngleDegrees();
                double turretErrorDeg = turretTargetDeg - turretMeasuredDeg;
                double turretOuterTrimDeg = TurretSubsystem.INSTANCE.getOuterLoopTrimDegrees();
                double turretCommandDiffDeg = TurretSubsystem.INSTANCE.getCommandDiffDegrees();
                double turretRateStepDeg = TurretSubsystem.INSTANCE.getRateLimitedStepDegrees();
                double turretLeftCmdDeg = TurretSubsystem.INSTANCE.getLeftTurretAngleCommandDegrees();
                double turretRightCmdDeg = TurretSubsystem.INSTANCE.getRightTurretAngleCommandDegrees();
                double turretServoPos = TurretSubsystem.INSTANCE.getCurrentServoPosition();
                double turretLeftServoPos = TurretSubsystem.INSTANCE.getCurrentLeftServoPosition();
                double turretRightServoPos = TurretSubsystem.INSTANCE.getCurrentRightServoPosition();
                double turretQuadRawDeg = TurretSubsystem.INSTANCE.getQuadRawAngleDegrees();
                double turretQuadOffsetDeg = TurretSubsystem.INSTANCE.getQuadratureOffsetDegrees();
                double turretEncoderVelDegPerSec = TurretSubsystem.INSTANCE.getMeasuredVelocityDegPerSec();
                double turretAbsRawDeg = TurretSubsystem.INSTANCE.getAbsoluteEncoderRawDegrees();
                double turretAbsDeg = TurretSubsystem.INSTANCE.getAbsoluteEncoderTurretAngleDegrees();
                double turretAbsDeltaDeg = TurretSubsystem.INSTANCE.getAbsoluteEncoderTurretDeltaDegrees();
                double turretLearnedServoOffsetDeg = TurretSubsystem.INSTANCE.getLearnedServoCommandOffsetDegrees();
                double turretAbsStartupErrorDeg = TurretSubsystem.INSTANCE.getAbsoluteStartupErrorDegrees();
                int turretEncoderTicks = TurretSubsystem.INSTANCE.getCurrentEncoderTicks();

                while (turretErrorDeg > 180) turretErrorDeg -= 360;
                while (turretErrorDeg < -180) turretErrorDeg += 360;

                double turretLogDtMs = 0.0;
                if (prevTurretLogMs > 0L && nowLogMs >= prevTurretLogMs) {
                    turretLogDtMs = nowLogMs - prevTurretLogMs;
                }
                double turretTargetDeltaDeg = 0.0;
                if (Double.isFinite(prevTurretLogTargetDeg)) {
                    turretTargetDeltaDeg = smallestWrappedDeltaDegrees(turretTargetDeg, prevTurretLogTargetDeg);
                }
                double turretMeasuredDeltaDeg = 0.0;
                if (Double.isFinite(prevTurretLogMeasuredDeg)) {
                    turretMeasuredDeltaDeg = smallestWrappedDeltaDegrees(turretMeasuredDeg, prevTurretLogMeasuredDeg);
                }
                double turretServoCommandDeltaDeg = 0.0;
                if (Double.isFinite(prevTurretLogServoCommandDeg)) {
                    turretServoCommandDeltaDeg = smallestWrappedDeltaDegrees(turretServoCommandDeg, prevTurretLogServoCommandDeg);
                }
                double turretServoPosDelta = 0.0;
                if (Double.isFinite(prevTurretLogServoPos)) {
                    turretServoPosDelta = turretServoPos - prevTurretLogServoPos;
                }

                double unclippedServoCommandDeg = turretCommandedDeg + turretOuterTrimDeg;
                boolean turretLimitClipped =
                        Math.abs(unclippedServoCommandDeg - turretServoCommandDeg) > 1e-6;
                double turretTrimAsServoDeltaPos =
                        Math.abs(turretOuterTrimDeg) / Math.max(1e-6, TurretSubsystem.TURRET_TRAVEL_DEGREES);
                boolean turretCmdAboveDeadbandGuess =
                        turretTrimAsServoDeltaPos >= Math.max(0.0, TURRET_DIAG_SERVO_DEADBAND_POS_GUESS);
                boolean turretTargetStable =
                        Math.abs(turretTargetDeltaDeg) <= Math.max(0.0, TURRET_DIAG_TARGET_STABLE_DELTA_DEG);
                boolean stictionCandidateNow =
                        turretTargetStable &&
                                Math.abs(turretErrorDeg) >= Math.max(0.0, TURRET_DIAG_STICTION_ERROR_MIN_DEG) &&
                                Math.abs(turretEncoderVelDegPerSec) <= Math.max(0.0, TURRET_DIAG_STICTION_VELOCITY_MAX_DEG_PER_SEC) &&
                                turretCmdAboveDeadbandGuess &&
                                !turretLimitClipped;
                if (stictionCandidateNow) {
                    if (turretStictionCandidateStartMs == 0L) {
                        turretStictionCandidateStartMs = nowLogMs;
                    }
                } else {
                    turretStictionCandidateStartMs = 0L;
                }
                boolean turretStictionSuspect =
                        stictionCandidateNow &&
                                turretStictionCandidateStartMs > 0L &&
                                (nowLogMs - turretStictionCandidateStartMs) >= Math.max(0L, TURRET_DIAG_STICTION_MIN_TIME_MS);

                long matchT = (logStartMs == 0L) ? 0L : (nowLogMs - logStartMs);

                turretLogger.addRow(
                        nowLogMs,
                        matchT,
                        botxvalue,
                        botyvalue,
                        Math.toDegrees(botHeadingRad),
                        shootTargetX,
                        shootTargetY,
                        fieldAngleDeg,
                        angleErrorDeg,
                        turretTargetDeg,
                        turretTargetCorrectedDeg,
                        turretCommandedDeg,
                        turretServoCommandDeg,
                        turretMeasuredDeg,
                        turretErrorDeg,
                        turretOuterTrimDeg,
                        turretCommandDiffDeg,
                        turretRateStepDeg,
                        turretLeftCmdDeg,
                        turretRightCmdDeg,
                        turretServoPos,
                        turretLeftServoPos,
                        turretRightServoPos,
                        turretQuadRawDeg,
                        turretQuadOffsetDeg,
                        turretEncoderVelDegPerSec,
                        turretAbsRawDeg,
                        turretAbsDeg,
                        turretAbsDeltaDeg,
                        turretLearnedServoOffsetDeg,
                        turretAbsStartupErrorDeg,
                        turretEncoderTicks,
                        turretLogDtMs,
                        turretTargetDeltaDeg,
                        turretMeasuredDeltaDeg,
                        turretServoCommandDeltaDeg,
                        turretServoPosDelta,
                        turretTrimAsServoDeltaPos,
                        turretCmdAboveDeadbandGuess,
                        turretTargetStable,
                        turretLimitClipped,
                        turretStictionSuspect,
                        TurretSubsystem.INSTANCE.isTurretReady(),
                        driving,
                        strafe,
                        rotate,
                        ODODistance,
                        xOffset,
                        yOffset,
                        hasResults,
                        loopTimeMs
                );

                prevTurretLogTargetDeg = turretTargetDeg;
                prevTurretLogMeasuredDeg = turretMeasuredDeg;
                prevTurretLogServoCommandDeg = turretServoCommandDeg;
                prevTurretLogServoPos = turretServoPos;
                prevTurretLogMs = nowLogMs;
            }
        }

        if (ENABLE_SOTM_LOGGING && sotmLogger != null) {
            long nowSotmLogMs = System.currentTimeMillis();
            if (nowSotmLogMs - lastSotmLogMs >= SOTM_LOG_PERIOD_MS) {
                lastSotmLogMs = nowSotmLogMs;
                long matchT = (logStartMs == 0L) ? 0L : (nowSotmLogMs - logStartMs);

                sotmLogger.addRow(
                        nowSotmLogMs,
                        matchT,
                        sotmFireRequestActive,
                        sotmControlActive,
                        rightBumperActive,
                        rightTriggerActive,
                        botxvalue,
                        botyvalue,
                        Math.toDegrees(botHeadingRad),
                        mt1Valid,
                        mt1PedroX,
                        mt1PedroY,
                        mt1PedroHeadingDeg,
                        mt2Valid,
                        mt2PedroX,
                        mt2PedroY,
                        mt2PedroHeadingDeg,
                        LIMELIGHT_VISION_BLEND_ENABLED,
                        limelightBlendSource,
                        limelightBlendPoseValid,
                        limelightBlendPoseAccepted,
                        limelightBlendPoseApplied,
                        limelightBlendGateReason,
                        limelightVisionConsecutiveAccepts,
                        limelightVisionLoopsSinceApply,
                        limelightBlendRawDxIn,
                        limelightBlendRawDyIn,
                        limelightBlendRawDistIn,
                        limelightBlendRawHeadingErrDeg,
                        limelightVisionBiasXIn,
                        limelightVisionBiasYIn,
                        limelightVisionBiasHeadingDeg,
                        limelightVisionLastNudgeXIn,
                        limelightVisionLastNudgeYIn,
                        limelightVisionLastNudgeHeadingDeg,
                        limelightBlendAlpha,
                        shootTargetX,
                        shootTargetY,
                        fieldAngleDeg,
                        angleErrorDeg,
                        ODODistance,
                        sotmResult.valid,
                        sotmResult.leadApplied,
                        sotmResult.speedInPerSec,
                        sotmVx,
                        sotmVy,
                        Math.toDegrees(sotmOmegaRawRadPerSec),
                        Math.toDegrees(sotmOmegaRadPerSec),
                        sotmResult.totalTimeSeconds,
                        sotmResult.leadXInches,
                        sotmResult.leadYInches,
                        sotmResult.radialVelocityInPerSec,
                        sotmResult.effectiveDistanceInches,
                        shooterDistanceForBallistics,
                        sotmResult.turretRobotRelativeAimDeg,
                        sotmResult.turretLagCompensationDeg,
                        TurretSubsystem.INSTANCE.getTargetAngleDegrees(),
                        TurretSubsystem.INSTANCE.getMeasuredAngleDegrees(),
                        TurretSubsystem.INSTANCE.isTurretReady(),
                        turretAimGateSatisfied,
                        turretGoalErrorDeg,
                        turretConstraintErrorDeg,
                        turretTargetReachable,
                        turretMeasuredVelDegPerSec,
                        turretSpeedGateSatisfied,
                        sotmFireGateSatisfied,
                        canShootAtGoal,
                        ShooterSubsystem.INSTANCE.getTargetRpm(),
                        rpmShooter1,
                        rpmShooter2,
                        shooterAtSpeed75,
                        ShooterSubsystem.INSTANCE.getBatteryVoltageRaw(),
                        ShooterSubsystem.INSTANCE.getBatteryVoltageFiltered(),
                        ShooterSubsystem.INSTANCE.getVoltageCompGain(),
                        ShooterSubsystem.INSTANCE.getCommandPreVoltageComp(),
                        ShooterSubsystem.INSTANCE.getCommandPostVoltageComp(),
                        ShooterSubsystem.INSTANCE.isCommandSaturated(),
                        dumbShootTimerActive,
                        IntakeWithSensorsSubsystem.INSTANCE.getBallCount(),
                        IntakeWithSensorsSubsystem.INSTANCE.getMotor1EncoderTicks(),
                        IntakeWithSensorsSubsystem.INSTANCE.getMotor3EncoderTicks(),
                        hold,
                        driving,
                        strafe,
                        rotate,
                        loopTimeMs
                );
            }
        }

        if (testShooter) {
            telemetry.addData("kP", ShooterSubsystem.kP);
            telemetry.addData("kI", ShooterSubsystem.kI);
            telemetry.addData("kD", ShooterSubsystem.kD);
            telemetry.addData("kS", ShooterSubsystem.kS);
            telemetry.addData("kV", ShooterSubsystem.kV);
            telemetry.addData("kA", ShooterSubsystem.kA);
            telemetry.addData("HEADROOM", ShooterSubsystem.HEADROOM);
            telemetry.addData("SLEW_PER_SECOND", ShooterSubsystem.SLEW_PER_SECOND);
            telemetry.addData("I_ZONE", ShooterSubsystem.I_ZONE);
        }

        if (result == null) {
            telemetry.addData("Limelight", "No result object");
        } else if (!result.isValid()) {
            telemetry.addData("Limelight", "No valid target");
        } else {
            telemetry.addData("Limelight", "Target seen!");
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ta", result.getTa());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("LL distance", distanceLL);
            Pose3D MT2Pose = result.getBotpose_MT2();
            if (MT2Pose != null) {
                telemetry.addData("MT2 Raw Angle", MT2Pose.getOrientation().getYaw());
                telemetry.addData("MT2 Raw X", MT2Pose.getPosition().x);
                telemetry.addData("MT2 Raw Y", MT2Pose.getPosition().y);
            }
        }
        telemetry.addData("MT1 valid", mt1Valid);
        telemetry.addData("MT1 Pedro X", mt1PedroX);
        telemetry.addData("MT1 Pedro Y", mt1PedroY);
        telemetry.addData("MT1 Pedro Heading", mt1PedroHeadingDeg);
        telemetry.addData("MT2 valid", mt2Valid);
        telemetry.addData("MT2 Pedro X", mt2PedroX);
        telemetry.addData("MT2 Pedro Y", mt2PedroY);
        telemetry.addData("MT2 Pedro Heading", mt2PedroHeadingDeg);
        telemetry.addData("LL Blend Source", limelightBlendSource);
        telemetry.addData("LL Blend Valid", limelightBlendPoseValid);
        telemetry.addData("LL Blend Accepted", limelightBlendPoseAccepted);
        telemetry.addData("LL Blend Applied", limelightBlendPoseApplied);
        telemetry.addData("LL Blend Gate", limelightBlendGateReason);
        telemetry.addData("LL Blend dX", limelightBlendRawDxIn);
        telemetry.addData("LL Blend dY", limelightBlendRawDyIn);
        telemetry.addData("LL Blend Dist", limelightBlendRawDistIn);
        telemetry.addData("LL Blend dH", limelightBlendRawHeadingErrDeg);
        telemetry.addData("LL Offset X (cum)", limelightVisionBiasXIn);
        telemetry.addData("LL Offset Y (cum)", limelightVisionBiasYIn);
        telemetry.addData("LL Offset H (cum)", limelightVisionBiasHeadingDeg);
        telemetry.addData("LL Nudge X", limelightVisionLastNudgeXIn);
        telemetry.addData("LL Nudge Y", limelightVisionLastNudgeYIn);
        telemetry.addData("LL Nudge H", limelightVisionLastNudgeHeadingDeg);
        telemetry.addData("LL Applied", limelightBlendPoseApplied);
        telemetry.addData("LL Streak", limelightVisionConsecutiveAccepts);
        telemetry.addData("LL Loops Since Apply", limelightVisionLoopsSinceApply);
        telemetry.addData("LL Blend Alpha", limelightBlendAlpha);
        telemetry.addData("ODO distance", ODODistance);
        telemetry.addData("ODO X-Location", botxvalue);
        telemetry.addData("ODO Y-Location", botyvalue);
        telemetry.addData("ODO Angle", Math.toDegrees(botHeadingRad));



        //Start Ian's control
        boolean leftTriggerActive = gamepad2.left_trigger > 0.1;
        if (!SHOT_TUNING_MODE && shotTuningPendingLabel) {
            labelShotTuningSample("UNRATED", "mode_exit");
        }

        // Shot-tuning overlay: when SHOT_TUNING_MODE is true, the driver uses all
        // of the normal driving / intake / shooter / dumbShoot controls, and this
        // block layers on three overrides:
        //   * Gamepad 2 D-pad up/down     -> adjust targetRPM in SHOT_TUNING_RPM_STEP increments
        //   * Gamepad 2 D-pad left/right  -> adjust shooterHoodPos in SHOT_TUNING_HOOD_STEP increments
        //   * Gamepad 1 X / B             -> label the pending shot as KEEP / IGNORE
        // Shot-tuning sample capture happens wherever the real fire event occurs
        // (see the dumbShoot block below). The goal target override itself
        // (SHOT_TUNING_TARGET_X_IN / Y_IN, adjusted via gamepad 1 D-pad) is
        // already applied earlier where shootTargetX/Y are computed.
        if (SHOT_TUNING_MODE) {
            shooterFollowEnabled = false;

            boolean tuningRpmUp = gamepad2.dpad_up;
            boolean tuningRpmDown = gamepad2.dpad_down;
            double rpmStep = Math.max(0.0, SHOT_TUNING_RPM_STEP);
            if (tuningRpmUp && !prevTuningRpmDpadUp) {
                targetRPM = Math.max(0.0, targetRPM + rpmStep);
            }
            if (tuningRpmDown && !prevTuningRpmDpadDown) {
                targetRPM = Math.max(0.0, targetRPM - rpmStep);
            }
            prevTuningRpmDpadUp = tuningRpmUp;
            prevTuningRpmDpadDown = tuningRpmDown;

            boolean tuningHoodRight = gamepad2.dpad_right;
            boolean tuningHoodLeft = gamepad2.dpad_left;
            double hoodStep = Math.max(0.0, SHOT_TUNING_HOOD_STEP);
            if (tuningHoodRight && !prevTuningHoodDpadRight) {
                shooterHoodPos = Math.min(1.0, shooterHoodPos + hoodStep);
            }
            if (tuningHoodLeft && !prevTuningHoodDpadLeft) {
                shooterHoodPos = Math.max(0.0, shooterHoodPos - hoodStep);
            }
            prevTuningHoodDpadRight = tuningHoodRight;
            prevTuningHoodDpadLeft = tuningHoodLeft;

            // Calibration-session waypoint advance. gamepad 1 back/start steps
            // CAL_POINT_INDEX down/up through the table, clamped to the valid
            // range. Only acts when CAL_SESSION_ACTIVE is true so casual tuning
            // outside a session does not accidentally move the pinned point.
            int tableSize = ShotCalibrationTable.active().size();
            boolean calDecNow = gamepad1.back;
            boolean calIncNow = gamepad1.start;
            if (CAL_SESSION_ACTIVE) {
                if (calDecNow && !prevCalPointDec) {
                    CAL_POINT_INDEX = Math.max(0, CAL_POINT_INDEX - 1);
                }
                if (calIncNow && !prevCalPointInc) {
                    CAL_POINT_INDEX = Math.min(tableSize - 1, CAL_POINT_INDEX + 1);
                }
            }
            prevCalPointDec = calDecNow;
            prevCalPointInc = calIncNow;

            if (gamepad1.xWasPressed() && shotTuningPendingLabel) {
                labelShotTuningSample("KEEP", "manual_keep");
            } else if (gamepad1.bWasPressed() && shotTuningPendingLabel) {
                labelShotTuningSample("IGNORE", "manual_ignore");
            }
        }

        {
//            if (gamepad2.rightBumperWasPressed()) {
//                this.shoot = true;
//                // Starting a new spin-up cancels any previous dumbShoot timeout
//                dumbShootTimerActive = false;
//                dumbShootSettleActive = false;
//                // In shot-tuning mode skip the auto-RPM recompute so the operator's
//                // manual d-pad RPM value is preserved. testShooter also preserves
//                // whatever targetRPM was set manually. Normal mode re-pulls from
//                // the table at the current pose (shotSol was computed earlier
//                // this loop).
//                if (!SHOT_TUNING_MODE && !testShooter) {
//                    targetRPM = applySotmRpmCompensation(
//                            shotSol.rpm,
//                            sotmResult.radialVelocityInPerSec,
//                            rpmLimitEnabled
//                    );
//                    shooterFollowEnabled = true;
//                }
//                //ShooterSubsystem.INSTANCE.increaseShooterRPMBy10();
//                telemetry.addData("Target Shooter Speed", targetRPM);
//                ShooterSubsystem.INSTANCE.spinUp(targetRPM);
//            } else if (gamepad2.leftBumperWasPressed()) {
//                ShooterSubsystem.INSTANCE.stop();
//                dumbShootTimerActive = false;
//                dumbShootSettleActive = false;
//                shooterFollowEnabled = false;
//                ShooterSubsystem.INSTANCE.resetHybridShotFeedBoostController();
//                //ShooterSubsystem.INSTANCE.decreaseShooterRPMBy10();
//            }

            if ((gamepad2.xWasPressed()) &&
                    shooterAtSpeed75 &&
                    canShootAtGoal) {
                int startBallCount = IntakeWithSensorsSubsystem.INSTANCE.getBallCount();
                boolean started = IntakeWithSensorsSubsystem.INSTANCE.shootMultipleSingleShots(startBallCount);
                if (started) {
                    startShotInfoSequence("multi_single_shot", nowMs, startBallCount, bb0, bb1, bb2);
                }
                //            ShooterSubsystem.INSTANCE.decreaseShooterHoodPosInc();
            }
            //        if (gamepad2.yWasPressed()) {
            //            ShooterSubsystem.INSTANCE.increaseShooterHoodPosInc();
            //        }

            boolean canFireTriggerShot =
                    shooterAtSpeed75 &&
                            canShootAtGoal &&
                            !tooCloseWarningActive;
            String desiredShotGateLedState = "NONE";
            if (sotmFireRequestActive && !tooCloseWarningActive) {
                if (!turretAimGateSatisfied) {
                    desiredShotGateLedState = "TURRET_BLOCK";
                } else if (!shooterAtSpeed75) {
                    desiredShotGateLedState = "RPM_BLOCK";
                }
            }
            if (!desiredShotGateLedState.equals(shotGateLedState)) {
                if ("TURRET_BLOCK".equals(desiredShotGateLedState)) {
                    LEDControlSubsystem.INSTANCE.startStrobe(
                            LEDControlSubsystem.LedColor.OFF,
                            LEDControlSubsystem.LedColor.ORANGE,
                            Math.max(10L, SHOOT_BLOCK_LED_STROBE_MS)
                    );
                } else if ("RPM_BLOCK".equals(desiredShotGateLedState)) {
                    LEDControlSubsystem.INSTANCE.startStrobe(
                            LEDControlSubsystem.LedColor.OFF,
                            LEDControlSubsystem.LedColor.BLUE,
                            Math.max(10L, SHOOT_BLOCK_LED_STROBE_MS)
                    );
                }
                shotGateLedState = desiredShotGateLedState;
            }
            maybeFinalizeShotInfoSequence(nowMs, sotmFireRequestActive, leftTriggerActive);
            if (sotmFireRequestActive && !tooCloseWarningActive) {
                // Only right-trigger shooting should lock robot translation/rotation.
                // Right-bumper SOTM stays fully driveable.
                hold = rightTriggerActive;

                boolean useFarBoostProfile = ODODistance >= ShooterSubsystem.HYBRID_NEAR_FAR_DISTANCE_THRESHOLD_IN;
                boolean farNoBoostShot = targetRPM >= FAR_NO_BOOST_RPM_THRESHOLD;

                if (canFireTriggerShot && !dumbShootTimerActive && !dumbShootSettleActive) {
                    int startBallCountBeforeDumbShoot = IntakeWithSensorsSubsystem.INSTANCE.getBallCount();

                    ShooterSubsystem.INSTANCE.boostOverride = false;
                    ShooterSubsystem.INSTANCE.resetHybridShotFeedBoostController();

                    if (farNoBoostShot) {
                        IntakeWithSensorsSubsystem.INSTANCE.setDumbShootFixedDelayMs(FAR_NO_BOOST_BETWEEN_SHOTS_MS);
                    } else {
                        IntakeWithSensorsSubsystem.INSTANCE.setDumbShootDistanceForDelayInches(ODODistance);
                    }

                    IntakeWithSensorsSubsystem.INSTANCE.dumbShoot();

                    startShotInfoSequence(
                            farNoBoostShot ? "dumbshoot_far_no_boost" : "dumbshoot",
                            nowMs,
                            startBallCountBeforeDumbShoot,
                            bb0,
                            bb1,
                            bb2
                    );

                    shotSequenceLinkedDumbShootRpmSequenceId = startDumbShootRpmLogSequence(
                            nowMs,
                            startBallCountBeforeDumbShoot,
                            bb0,
                            bb1,
                            bb2,
                            shotSequenceId
                    );

                    dumbShootTimerActive = true;
                    dumbShootStartTimeMs = nowMs;

                    if (!farNoBoostShot) {
                        if (ShooterSubsystem.ENABLE_HYBRID_SHOT_FEED_BOOST) {
                            ShooterSubsystem.INSTANCE.startHybridShotFeedBoostController(ODODistance);
                        } else {
                            ShooterSubsystem.INSTANCE.setBoostOn(useFarBoostProfile);
                        }
                    }

                    // In shot-tuning mode, capture a sample for this dumbShoot so
                    // the driver can label it KEEP / IGNORE on gamepad 1 X / B.
                    if (SHOT_TUNING_MODE) {
                        if (shotTuningPendingLabel) {
                            labelShotTuningSample("UNRATED", "next_shot_before_label");
                        }
                        shotTuningShotIdCounter++;
                        shotTuningPendingLabel = true;
                        shotTuningPendingShotId = shotTuningShotIdCounter;
                        shotTuningPendingFireMs = nowMs;
                        shotTuningPendingStartBallCount = startBallCountBeforeDumbShoot;
                        shotTuningPendingTargetRpm = targetRPM;
                        shotTuningPendingHoodPos = shooterHoodPos;
                        shotTuningPendingRpm1 = rpmShooter1;
                        shotTuningPendingRpm2 = rpmShooter2;
                        shotTuningPendingBotX = botxvalue;
                        shotTuningPendingBotY = botyvalue;
                        shotTuningPendingBotHeadingDeg = Math.toDegrees(botHeadingRad);
                        shotTuningPendingTargetX = shootTargetX;
                        shotTuningPendingTargetY = shootTargetY;
                        shotTuningPendingOdoDistance = ODODistance;
                        shotTuningPendingTurretTargetDeg = TurretSubsystem.INSTANCE.getTargetAngleDegrees();
                        shotTuningPendingTurretMeasuredDeg = TurretSubsystem.INSTANCE.getMeasuredAngleDegrees();

                        ShotSolution fireSol = ShotCalibrationTable.active().lookup(botxvalue, botyvalue);
                        shotTuningPendingTableRpm = fireSol.rpm;
                        shotTuningPendingTableHood = fireSol.hoodPos;
                        shotTuningPendingTableAimX = fireSol.aimX;
                        shotTuningPendingTableAimY = fireSol.aimY;
                        shotTuningPendingZone = ShootingZones.zoneLabel(botxvalue, botyvalue);

                        if (CAL_SESSION_ACTIVE) {
                            shotTuningPendingCalPointIndex = CAL_POINT_INDEX;
                            ShotSample target = ShotCalibrationTable.active().sampleAt(CAL_POINT_INDEX);
                            shotTuningPendingTargetPointX = target.x;
                            shotTuningPendingTargetPointY = target.y;
                        } else {
                            shotTuningPendingCalPointIndex = -1;
                            shotTuningPendingTargetPointX = Double.NaN;
                            shotTuningPendingTargetPointY = Double.NaN;
                        }
                    }
                }
            }  else if (leftTriggerActive && !prevLeftTriggerActive) {
                // Latch a single-shot request; actual firing waits for RPM to be in range
                hold = true;
                singleShotPending = true;
            } else if (gamepad2.aWasPressed()) {
                hold = false;
                this.shoot = false;
                //targetRPM = 1000;
                IntakeWithSensorsSubsystem.INSTANCE.intakeForward();//Hoping Forward is Intake (maybe change the method name)
//                ShooterSubsystem.INSTANCE.stop();
                if (shooterIdleMode) {
                    targetRPM = SHOOTER_IDLE_RPM;
                    shooterFollowEnabled = true;   // keep it spinning
                } else {
                    targetRPM = 0.0;
                    shooterFollowEnabled = false;
                    ShooterSubsystem.INSTANCE.stop();
                }
                shooterFollowEnabled = false;
                dumbShootTimerActive = false;
                dumbShootSettleActive = false;
                ShooterSubsystem.INSTANCE.resetHybridShotFeedBoostController();
                this.hasResults = false;
                LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.RED);
            } else if (gamepad2.b) {
                hold = false;
                IntakeWithSensorsSubsystem.INSTANCE.intakeReverse();
            } else {
                hold = false;
            }

            // If a single shot is pending and left trigger is still held (and right trigger is not),
            // wait for shooter RPM to be within tolerance, then fire exactly one ball.
            if (leftTriggerActive && !sotmFireRequestActive && singleShotPending &&
                    canShootAtGoal &&
                    shooterAtSpeed75) { // 75 RPM window for 58-RPM resolution
                if (IntakeWithSensorsSubsystem.INSTANCE.feedSingleBallFullPower()) {
                    startShotInfoSequence("single_ball_feed", nowMs, IntakeWithSensorsSubsystem.INSTANCE.getBallCount(), bb0, bb1, bb2);
                    singleShotPending = false;
                }
            }

            // If the left trigger is released, clear any pending single-shot request so the
            // next press is treated as a new request.
            if (!leftTriggerActive && prevLeftTriggerActive) {
                singleShotPending = false;
                hold = false;
            }
        }

        prevLeftTriggerActive = leftTriggerActive;
        prevGamepad1LeftBumper = leftBumperPressedNow;
        prevSotmFireRequestActive = sotmFireRequestActive;

        if (hold && !prevHold) {
            PedroComponent.follower().holdPoint(new BezierPoint(PedroComponent.follower().getPose()), PedroComponent.follower().getHeading(), false);
        } else if (!hold && prevHold){
            PedroComponent.follower().startTeleopDrive();
        }

        prevHold = hold;


        if (SHOT_TUNING_MODE) {
            // Keep manual hood value in shot tuning mode.
        } else if (testShooter) {
            // Do nothing
        } else {
            this.shooterHoodPos = shotSol.hoodPos;
        }
        ShooterSubsystem.INSTANCE.shooterHoodDrive(this.shooterHoodPos);

        telemetryM.update(telemetry);
        telemetry.update();
        timer.end();
    }

    @Override
    public void onStop() {
        super.onStop();
        matchHasStarted = false;
        ShooterSubsystem.INSTANCE.stop();
        targetRPM = 0.0;
        ShooterSubsystem.INSTANCE.resetHybridShotFeedBoostController();

        if (ENABLE_TURRET_LOGGING && turretLogger != null) {
            File savedFile = turretLogger.save();
            if (savedFile != null) {
                RobotLog.ii("Pickles2025Teleop", "Turret CSV saved: " + savedFile.getAbsolutePath());
            } else {
                RobotLog.ww("Pickles2025Teleop", "Turret CSV was not saved (logger not started, empty, or save failed).");
            }
        }

        if (ENABLE_SOTM_LOGGING && sotmLogger != null) {
            File savedFile = sotmLogger.save();
            if (savedFile != null) {
                RobotLog.ii("Pickles2025Teleop", "SOTM CSV saved: " + savedFile.getAbsolutePath());
            } else {
                RobotLog.ww("Pickles2025Teleop", "SOTM CSV was not saved (logger not started, empty, or save failed).");
            }
        }

        if (ENABLE_SHOT_INFO_LOGGING && shotInfoLogger != null) {
            // Ensure an in-progress sequence is not lost.
            if (shotLogSequenceActive) {
                finalizeShotInfoSequence("opmode_stop", System.currentTimeMillis());
            }
            File savedFile = shotInfoLogger.save();
            if (savedFile != null) {
                RobotLog.ii("Pickles2025Teleop", "Shot-info CSV saved: " + savedFile.getAbsolutePath());
            } else {
                RobotLog.ww("Pickles2025Teleop", "Shot-info CSV was not saved (logger not started, empty, or save failed).");
            }
        }

        if (ENABLE_DUMBSHOOT_RPM_LOGGING && dumbShootRpmLogger != null) {
            File savedFile = dumbShootRpmLogger.save();
            if (savedFile != null) {
                RobotLog.ii("Pickles2025Teleop", "Dumbshoot RPM CSV saved: " + savedFile.getAbsolutePath());
            } else {
                RobotLog.ww("Pickles2025Teleop", "Dumbshoot RPM CSV was not saved (logger not started, empty, or save failed).");
            }
        }

        if (ENABLE_SHOT_TUNING_LOGGING && shotTuningLogger != null) {
            if (shotTuningPendingLabel) {
                labelShotTuningSample("UNRATED", "opmode_stop");
            }
            File savedFile = shotTuningLogger.save();
            if (savedFile != null) {
                RobotLog.ii("Pickles2025Teleop", "Shot tuning CSV saved: " + savedFile.getAbsolutePath());
            } else {
                RobotLog.ww("Pickles2025Teleop", "Shot tuning CSV was not saved (logger not started, empty, or save failed).");
            }
        }
    }

    private boolean isShooterReadyForFeed(double toleranceRpm, double rpmShooter1, double rpmShooter2) {
        double targetRpm = ShooterSubsystem.INSTANCE.getTargetRpm();
        if (targetRpm < SHOOT_GATE_MIN_TARGET_RPM) {
            return false;
        }
        double averageShooterRpm = 0.5 * (rpmShooter1 + rpmShooter2);
        return Math.abs(targetRpm - averageShooterRpm) <= toleranceRpm;
    }

    private void labelShotTuningSample(String label, String labelReason) {
        if (!ENABLE_SHOT_TUNING_LOGGING || shotTuningLogger == null) return;
        if (!shotTuningPendingLabel) return;

        long nowMs = System.currentTimeMillis();
        long matchT = (logStartMs == 0L) ? 0L : (nowMs - logStartMs);
        long sinceFireMs = nowMs - shotTuningPendingFireMs;
        double rpm1Now = ShooterSubsystem.INSTANCE.getShooter1RPM();
        double rpm2Now = ShooterSubsystem.INSTANCE.getShooter2RPM();
        shotTuningLogger.addRow(
                nowMs,
                matchT,
                shotTuningPendingShotId,
                label,
                labelReason,
                shotTuningPendingFireMs,
                sinceFireMs,
                shotTuningPendingStartBallCount,
                shotTuningPendingTargetX,
                shotTuningPendingTargetY,
                shotTuningPendingBotX,
                shotTuningPendingBotY,
                shotTuningPendingBotHeadingDeg,
                shotTuningPendingOdoDistance,
                shotTuningPendingTargetRpm,
                shotTuningPendingHoodPos,
                shotTuningPendingRpm1,
                shotTuningPendingRpm2,
                targetRPM,
                shooterHoodPos,
                rpm1Now,
                rpm2Now,
                shotTuningPendingTurretTargetDeg,
                shotTuningPendingTurretMeasuredDeg,
                TurretSubsystem.INSTANCE.getTargetAngleDegrees(),
                TurretSubsystem.INSTANCE.getMeasuredAngleDegrees(),
                ShooterSubsystem.INSTANCE.boostActive,
                ShooterSubsystem.INSTANCE.secondBoostActive,
                ShooterSubsystem.INSTANCE.boostOverride,
                shotTuningPendingCalPointIndex,
                shotTuningPendingTargetPointX,
                shotTuningPendingTargetPointY,
                shotTuningPendingZone,
                shotTuningPendingTableRpm,
                shotTuningPendingTableHood,
                shotTuningPendingTableAimX,
                shotTuningPendingTableAimY
        );

        shotTuningPendingLabel = false;
    }

    private int startDumbShootRpmLogSequence(
            long nowMs,
            int expectedShots,
            boolean bb0,
            boolean bb1,
            boolean bb2,
            int linkedShotInfoSequenceId
    ) {
        if (!ENABLE_DUMBSHOOT_RPM_LOGGING || dumbShootRpmLogger == null) return -1;
        dumbShootRpmSequenceCounter++;
        dumbShootRpmLogSequenceId = dumbShootRpmSequenceCounter;
        dumbShootRpmLogStartMs = nowMs;
        dumbShootRpmLogExpectedShots = Math.max(0, expectedShots);
        dumbShootRpmLinkedShotInfoSequenceId = linkedShotInfoSequenceId;
        dumbShootRpmBurstProfileId = BURST_PROFILE_ID;
        dumbShootRpmBb0RiseCount = 0;
        dumbShootRpmBb0FallCount = 0;
        dumbShootRpmBb1RiseCount = 0;
        dumbShootRpmBb1FallCount = 0;
        dumbShootRpmBb2RiseCount = 0;
        dumbShootRpmBb2FallCount = 0;
        dumbShootRpmLogActive = true;
        dumbShootRpmPrevBbInitialized = true;
        dumbShootRpmPrevBb0 = bb0;
        dumbShootRpmPrevBb1 = bb1;
        dumbShootRpmPrevBb2 = bb2;
        lastDumbShootRpmLogMs = 0L;
        return dumbShootRpmLogSequenceId;
    }

    private void maybeLogDumbShootRpmSample(
            long nowMs,
            boolean bb0,
            boolean bb1,
            boolean bb2,
            double rpmShooter1,
            double rpmShooter2
    ) {
        if (!ENABLE_DUMBSHOOT_RPM_LOGGING || dumbShootRpmLogger == null) return;
        if (!dumbShootRpmLogActive) return;

        long sinceStartMs = nowMs - dumbShootRpmLogStartMs;
        if (sinceStartMs > Math.max(0L, DUMBSHOOT_RPM_LOG_DURATION_MS)) {
            dumbShootRpmLogActive = false;
            return;
        }

        if (lastDumbShootRpmLogMs > 0L &&
                !DUMBSHOOT_RPM_LOG_EVERY_LOOP &&
                nowMs - lastDumbShootRpmLogMs < Math.max(1L, DUMBSHOOT_RPM_LOG_PERIOD_MS)) {
            return;
        }
        lastDumbShootRpmLogMs = nowMs;

        boolean bb0RiseEdge = false;
        boolean bb0FallEdge = false;
        boolean bb1RiseEdge = false;
        boolean bb1FallEdge = false;
        boolean bb2RiseEdge = false;
        boolean bb2FallEdge = false;
        if (dumbShootRpmPrevBbInitialized) {
            bb0RiseEdge = !dumbShootRpmPrevBb0 && bb0;
            bb0FallEdge = dumbShootRpmPrevBb0 && !bb0;
            bb1RiseEdge = !dumbShootRpmPrevBb1 && bb1;
            bb1FallEdge = dumbShootRpmPrevBb1 && !bb1;
            bb2RiseEdge = !dumbShootRpmPrevBb2 && bb2;
            bb2FallEdge = dumbShootRpmPrevBb2 && !bb2;
        }
        if (bb0RiseEdge) dumbShootRpmBb0RiseCount++;
        if (bb0FallEdge) dumbShootRpmBb0FallCount++;
        if (bb1RiseEdge) dumbShootRpmBb1RiseCount++;
        if (bb1FallEdge) dumbShootRpmBb1FallCount++;
        if (bb2RiseEdge) dumbShootRpmBb2RiseCount++;
        if (bb2FallEdge) dumbShootRpmBb2FallCount++;
        dumbShootRpmPrevBb0 = bb0;
        dumbShootRpmPrevBb1 = bb1;
        dumbShootRpmPrevBb2 = bb2;
        dumbShootRpmPrevBbInitialized = true;

        long matchT = (logStartMs == 0L) ? 0L : (nowMs - logStartMs);
        double shooterAvgRpm = 0.5 * (rpmShooter1 + rpmShooter2);
        double loopTimeMs = timer.getMs();
        dumbShootRpmLogger.addRow(
                nowMs,
                matchT,
                dumbShootRpmLogSequenceId,
                dumbShootRpmLinkedShotInfoSequenceId,
                dumbShootRpmBurstProfileId,
                sinceStartMs,
                dumbShootRpmLogExpectedShots,
                dumbShootTimerActive,
                ShooterSubsystem.INSTANCE.getTargetRpm(),
                rpmShooter1,
                rpmShooter2,
                shooterAvgRpm,
                ShooterSubsystem.INSTANCE.getAverageRpmDelta(),
                ShooterSubsystem.INSTANCE.getShooter1Power(),
                ShooterSubsystem.INSTANCE.getShooter2Power(),
                isShooterReadyForFeed(
                        SHOOT_GATE_AT_SPEED_TOLERANCE_RPM,
                        rpmShooter1,
                        rpmShooter2
                ),
                ShooterSubsystem.INSTANCE.getBatteryVoltageRaw(),
                ShooterSubsystem.INSTANCE.getBatteryVoltageFiltered(),
                ShooterSubsystem.INSTANCE.getVoltageCompGain(),
                ShooterSubsystem.INSTANCE.getCommandPreVoltageComp(),
                ShooterSubsystem.INSTANCE.getCommandPostVoltageComp(),
                ShooterSubsystem.INSTANCE.isCommandSaturated(),
                ShooterSubsystem.INSTANCE.boostActive,
                ShooterSubsystem.INSTANCE.secondBoostActive,
                ShooterSubsystem.INSTANCE.boostOverride,
                ShooterSubsystem.INSTANCE.getActiveBoostMultiplier1(),
                ShooterSubsystem.INSTANCE.getActiveBoostMultiplier2(),
                ShooterSubsystem.INSTANCE.isHybridShotFeedBoostActive(),
                ShooterSubsystem.INSTANCE.getHybridShotFeedBoostPhaseName(),
                ShooterSubsystem.INSTANCE.getHybridTimeSinceShotFeedStartMs(),
                ShooterSubsystem.INSTANCE.getHybridExpectedContactMsForCurrentPhase(),
                ShooterSubsystem.INSTANCE.getActivePreBoostAmount(),
                ShooterSubsystem.INSTANCE.getHybridLastAdvanceReason(),
                ShooterSubsystem.INSTANCE.getHybridLastAdvanceAtRelMs(),
                ShooterSubsystem.INSTANCE.getMeasuredRpmFilteredBaseline(),
                ShooterSubsystem.INSTANCE.getMeasuredRpmDerivativeRpmPerSec(),
                ShooterSubsystem.INSTANCE.getRpmAtShotSequenceStart(),
                ShooterSubsystem.INSTANCE.getRpmDeltaFromTarget(),
                ShooterSubsystem.INSTANCE.getRpmDeltaFromBaseline(),
                ShooterSubsystem.INSTANCE.isRpmDropCandidateTargetDelta(),
                ShooterSubsystem.INSTANCE.isRpmDropCandidateBaselineDelta(),
                ShooterSubsystem.INSTANCE.isRpmDropCandidateDerivative(),
                ShooterSubsystem.INSTANCE.getShooter1CurrentA(),
                ShooterSubsystem.INSTANCE.getShooter2CurrentA(),
                ShooterSubsystem.INSTANCE.getAverageCurrentA(),
                ShooterSubsystem.INSTANCE.getCurrentFilteredBaselineA(),
                ShooterSubsystem.INSTANCE.getCurrentDerivativeAPerSec(),
                ShooterSubsystem.INSTANCE.getCurrentAtShotSequenceStartA(),
                ShooterSubsystem.INSTANCE.getCurrentDeltaFromBaselineA(),
                ShooterSubsystem.INSTANCE.isCurrentSpikeCandidateBaselineDelta(),
                ShooterSubsystem.INSTANCE.isCurrentSpikeCandidateDerivative(),
                bb0,
                bb1,
                bb2,
                bb0RiseEdge,
                bb0FallEdge,
                bb1RiseEdge,
                bb1FallEdge,
                bb2RiseEdge,
                bb2FallEdge,
                dumbShootRpmBb0RiseCount,
                dumbShootRpmBb0FallCount,
                dumbShootRpmBb1RiseCount,
                dumbShootRpmBb1FallCount,
                dumbShootRpmBb2RiseCount,
                dumbShootRpmBb2FallCount,
                IntakeWithSensorsSubsystem.INSTANCE.getBallCount(),
                loopTimeMs
        );
    }

    private void startShotInfoSequence(
            String reason,
            long nowMs,
            int startBallCount,
            boolean bb0,
            boolean bb1,
            boolean bb2
    ) {
        if (!ENABLE_SHOT_INFO_LOGGING || shotInfoLogger == null) return;
        if (shotLogSequenceActive) return;

        shotLogSequenceActive = true;
        shotSequenceStartMs = nowMs;
        shotSequenceId = ++shotSequenceIdCounter;
        shotSequenceStartBallCount = Math.max(0, startBallCount);
        shotSequenceExpectedShots = Math.max(1, Math.min(3, shotSequenceStartBallCount));
        shotSequenceReason = reason;
        shotSequenceBurstProfileId = BURST_PROFILE_ID;
        shotSequenceStartTargetRpm = ShooterSubsystem.INSTANCE.getTargetRpm();
        shotSequenceStartHoodPos = shooterHoodPos;
        shotSequenceStartBoostDelayMs = ShooterSubsystem.BOOST_DELAY_MS;
        shotSequenceStartPreBoostAmount =
                ShooterSubsystem.ENABLE_HYBRID_SHOT_FEED_BOOST
                        ? Math.max(0.0, ShooterSubsystem.HYBRID_PREBOOST1_AMOUNT)
                        : 0.0;
        shotSequenceStartBoostMult1 = ShooterSubsystem.INSTANCE.getActiveBoostMultiplier1();
        shotSequenceStartBoostMult2 = ShooterSubsystem.INSTANCE.getActiveBoostMultiplier2();
        shotSequenceLinkedDumbShootRpmSequenceId = -1;
        shotSequenceStartBb0 = bb0;
        shotSequenceStartBb1 = bb1;
        shotSequenceStartBb2 = bb2;
        shotBb0FallCount = 0;
        shotBb0RiseCount = 0;
        shotBb1FallCount = 0;
        shotBb1RiseCount = 0;
        shotBb2FallCount = 0;
        shotBb2RiseCount = 0;
        shotBb2ClearLoopCountTotal = 0;
        shotBb2ClearStreakCurrent = 0;
        shotBb2ClearStreakMax = 0;

        for (int i = 0; i < 3; i++) {
            shotBb0FallMs[i] = -1L;
            shotBb0RiseMs[i] = -1L;
            shotBb1FallMs[i] = -1L;
            shotBb1RiseMs[i] = -1L;
            shotBb2FallMs[i] = -1L;
            shotBb2RiseMs[i] = -1L;
            shotIntervalMs[i] = -1L;
        }
    }

    private void updateShotInfoBreakbeamTracking(long nowMs, boolean bb0, boolean bb1, boolean bb2) {
        if (!ENABLE_SHOT_INFO_LOGGING || shotInfoLogger == null) return;

        if (!bbPrevInitializedForShotLog) {
            prevBb0ForShotLog = bb0;
            prevBb1ForShotLog = bb1;
            prevBb2ForShotLog = bb2;
            bbPrevInitializedForShotLog = true;
            return;
        }

        if (!shotLogSequenceActive) {
            prevBb0ForShotLog = bb0;
            prevBb1ForShotLog = bb1;
            prevBb2ForShotLog = bb2;
            return;
        }

        long relMs = nowMs - shotSequenceStartMs;

        // Loop-level "clear beam" observability for sensor2.
        // bb2 == true means broken; clear/unbroken is !bb2.
        if (!bb2) {
            shotBb2ClearLoopCountTotal++;
            shotBb2ClearStreakCurrent++;
            if (shotBb2ClearStreakCurrent > shotBb2ClearStreakMax) {
                shotBb2ClearStreakMax = shotBb2ClearStreakCurrent;
            }
        } else {
            shotBb2ClearStreakCurrent = 0;
        }

        if (!prevBb0ForShotLog && bb0) {
            if (shotBb0RiseCount < 3) shotBb0RiseMs[shotBb0RiseCount] = relMs;
            shotBb0RiseCount++;
        } else if (prevBb0ForShotLog && !bb0) {
            if (shotBb0FallCount < 3) shotBb0FallMs[shotBb0FallCount] = relMs;
            shotBb0FallCount++;
        }

        if (!prevBb1ForShotLog && bb1) {
            if (shotBb1RiseCount < 3) shotBb1RiseMs[shotBb1RiseCount] = relMs;
            shotBb1RiseCount++;
        } else if (prevBb1ForShotLog && !bb1) {
            if (shotBb1FallCount < 3) shotBb1FallMs[shotBb1FallCount] = relMs;
            shotBb1FallCount++;
        }

        if (!prevBb2ForShotLog && bb2) {
            if (shotBb2RiseCount < 3) shotBb2RiseMs[shotBb2RiseCount] = relMs;
            shotBb2RiseCount++;
        } else if (prevBb2ForShotLog && !bb2) {
            if (shotBb2FallCount < 3) shotBb2FallMs[shotBb2FallCount] = relMs;
            shotBb2FallCount++;
            if (shotBb2FallCount == 1) {
                shotIntervalMs[0] = shotBb2FallMs[0];
            } else if (shotBb2FallCount == 2) {
                shotIntervalMs[1] = shotBb2FallMs[1] - shotBb2FallMs[0];
            } else if (shotBb2FallCount == 3) {
                shotIntervalMs[2] = shotBb2FallMs[2] - shotBb2FallMs[1];
            }
        }

        prevBb0ForShotLog = bb0;
        prevBb1ForShotLog = bb1;
        prevBb2ForShotLog = bb2;
    }

    private void maybeFinalizeShotInfoSequence(long nowMs, boolean rightTriggerActive, boolean leftTriggerActive) {
        if (!ENABLE_SHOT_INFO_LOGGING || shotInfoLogger == null) return;
        if (!shotLogSequenceActive) return;

        long elapsed = nowMs - shotSequenceStartMs;
        long lastBb2FallRelMs = shotBb2FallCount > 0
                ? shotBb2FallMs[Math.min(shotBb2FallCount, 3) - 1]
                : -1L;
        long sinceLastBb2FallMs = lastBb2FallRelMs >= 0 ? (elapsed - lastBb2FallRelMs) : Long.MAX_VALUE;
        boolean expectedShotsSeen = shotBb2FallCount >= shotSequenceExpectedShots;
        boolean expectedShotsSettled =
                expectedShotsSeen &&
                        sinceLastBb2FallMs >= Math.max(0L, SHOT_INFO_EXPECTED_SHOTS_FINALIZE_GRACE_MS);

        // For dumbshoot: avoid early "idle_end" finalization, which can truncate
        // breakbeam capture before all expected transitions occur.
        if ("dumbshoot".equals(shotSequenceReason)) {
            boolean minCaptureSatisfied =
                    elapsed >= Math.max(0L, SHOT_INFO_DUMBSHOOT_MIN_CAPTURE_MS);
            if (expectedShotsSettled && minCaptureSatisfied) {
                finalizeShotInfoSequence("expected_shots_seen", nowMs);
                return;
            }

            // If two shots were detected, prefer a short targeted window for shot3.
            if (shotSequenceExpectedShots >= 3 && shotBb2FallCount >= 2 && shotBb2FallMs[1] >= 0L) {
                long sinceSecondShotMs = elapsed - shotBb2FallMs[1];
                if (minCaptureSatisfied &&
                        sinceSecondShotMs >= Math.max(0L, SHOT_INFO_DUMBSHOOT_AFTER_SECOND_TIMEOUT_MS)) {
                    finalizeShotInfoSequence("third_missing_after_second", nowMs);
                    return;
                }
            }

            if (elapsed >= Math.max(0L, SHOT_INFO_DUMBSHOOT_TIMEOUT_MS)) {
                finalizeShotInfoSequence("dumbshoot_timeout", nowMs);
            }
            return;
        }

        if (expectedShotsSettled) {
            finalizeShotInfoSequence("expected_shots_seen", nowMs);
            return;
        }

        if (elapsed >= SHOT_INFO_LOG_TIMEOUT_MS) {
            finalizeShotInfoSequence("timeout", nowMs);
            return;
        }

        // If triggers are released and intake/shoot modes are idle, finalize early after short grace.
        boolean intakeBusy =
                IntakeWithSensorsSubsystem.INSTANCE.isSingleBallFeedActive() ||
                        IntakeWithSensorsSubsystem.INSTANCE.isMultiSingleShotActive() ||
                        IntakeWithSensorsSubsystem.INSTANCE.isShootSequenceActive() ||
                        IntakeWithSensorsSubsystem.INSTANCE.isShooting();
        if (!rightTriggerActive && !leftTriggerActive && !intakeBusy && elapsed > 300) {
            finalizeShotInfoSequence("idle_end", nowMs);
        }
    }

    private void finalizeShotInfoSequence(String endReason, long nowMs) {
        if (!ENABLE_SHOT_INFO_LOGGING || shotInfoLogger == null) return;
        if (!shotLogSequenceActive) return;

        long matchT = (logStartMs == 0L) ? 0L : (nowMs - logStartMs);
        long durationMs = nowMs - shotSequenceStartMs;
        long bb2ClearGap12Ms = (shotBb2RiseMs[0] >= 0L && shotBb2FallMs[1] >= 0L)
                ? (shotBb2FallMs[1] - shotBb2RiseMs[0])
                : -1L;
        long bb2ClearGap23Ms = (shotBb2RiseMs[1] >= 0L && shotBb2FallMs[2] >= 0L)
                ? (shotBb2FallMs[2] - shotBb2RiseMs[1])
                : -1L;

        shotInfoLogger.addRow(
                nowMs,
                matchT,
                shotSequenceId,
                shotSequenceReason,
                shotSequenceStartBallCount,
                shotSequenceExpectedShots,
                shotSequenceStartBb0,
                shotSequenceStartBb1,
                shotSequenceStartBb2,
                shotBb0FallMs[0],
                shotBb0RiseMs[0],
                shotBb0FallMs[1],
                shotBb0RiseMs[1],
                shotBb0FallMs[2],
                shotBb0RiseMs[2],
                shotBb1FallMs[0],
                shotBb1RiseMs[0],
                shotBb1FallMs[1],
                shotBb1RiseMs[1],
                shotBb1FallMs[2],
                shotBb1RiseMs[2],
                shotBb2FallMs[0],
                shotBb2RiseMs[0],
                shotBb2FallMs[1],
                shotBb2RiseMs[1],
                shotBb2FallMs[2],
                shotBb2RiseMs[2],
                shotIntervalMs[0],
                shotIntervalMs[1],
                shotIntervalMs[2],
                bb2ClearGap12Ms,
                bb2ClearGap23Ms,
                shotBb2ClearLoopCountTotal,
                shotBb2ClearStreakMax,
                shotBb0FallCount,
                shotBb0RiseCount,
                shotBb1FallCount,
                shotBb1RiseCount,
                shotBb2FallCount,
                shotBb2RiseCount,
                endReason,
                durationMs,
                shotSequenceBurstProfileId,
                shotSequenceStartTargetRpm,
                shotSequenceStartHoodPos,
                shotSequenceStartBoostDelayMs,
                shotSequenceStartPreBoostAmount,
                shotSequenceStartBoostMult1,
                shotSequenceStartBoostMult2,
                shotSequenceLinkedDumbShootRpmSequenceId
        );

        shotLogSequenceActive = false;
        shotSequenceStartMs = 0L;
        shotSequenceStartBallCount = 0;
        shotSequenceExpectedShots = 0;
        shotSequenceReason = "";
        shotSequenceBurstProfileId = 0;
        shotSequenceStartTargetRpm = 0.0;
        shotSequenceStartHoodPos = 0.0;
        shotSequenceStartBoostDelayMs = 0L;
        shotSequenceStartPreBoostAmount = 0.0;
        shotSequenceStartBoostMult1 = 0.0;
        shotSequenceStartBoostMult2 = 0.0;
        shotSequenceLinkedDumbShootRpmSequenceId = -1;
    }

    private static class SOTMResult {
        final boolean valid;
        final boolean leadApplied;
        final double speedInPerSec;
        final double totalTimeSeconds;
        final double leadXInches;
        final double leadYInches;
        final double radialVelocityInPerSec;
        final double effectiveDistanceInches;
        final double distanceForBallisticsInches;
        final double turretRobotRelativeAimDeg;
        final double turretLagCompensationDeg;

        SOTMResult(
                boolean valid,
                boolean leadApplied,
                double speedInPerSec,
                double totalTimeSeconds,
                double leadXInches,
                double leadYInches,
                double radialVelocityInPerSec,
                double effectiveDistanceInches,
                double distanceForBallisticsInches,
                double turretRobotRelativeAimDeg,
                double turretLagCompensationDeg
        ) {
            this.valid = valid;
            this.leadApplied = leadApplied;
            this.speedInPerSec = speedInPerSec;
            this.totalTimeSeconds = totalTimeSeconds;
            this.leadXInches = leadXInches;
            this.leadYInches = leadYInches;
            this.radialVelocityInPerSec = radialVelocityInPerSec;
            this.effectiveDistanceInches = effectiveDistanceInches;
            this.distanceForBallisticsInches = distanceForBallisticsInches;
            this.turretRobotRelativeAimDeg = turretRobotRelativeAimDeg;
            this.turretLagCompensationDeg = turretLagCompensationDeg;
        }
    }

    private SOTMResult calculateSotmResult(
            double botX,
            double botY,
            double botHeadingRad,
            double targetX,
            double targetY,
            double vxInPerSec,
            double vyInPerSec,
            double speedInPerSec,
            double omegaRadPerSec,
            boolean sotmControlActive
    ) {
        double dx = targetX - botX;
        double dy = targetY - botY;
        double realDistance = Math.hypot(dx, dy);
        if (!Double.isFinite(realDistance) || realDistance < 1e-6) {
            return new SOTMResult(
                    false,
                    false,
                    speedInPerSec,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    minDisatanceForShooting,
                    0.0,
                    0.0
            );
        }

        double totalTimeSec = getSotmTotalTimeSeconds(realDistance);
        double leadX = 0.0;
        double leadY = 0.0;
        boolean leadApplied = sotmControlActive && speedInPerSec > SOTM_MIN_SPEED_IN_PER_SEC;
        if (leadApplied) {
            leadX = vxInPerSec * totalTimeSec * SOTM_LEAD_GAIN;
            leadY = vyInPerSec * totalTimeSec * SOTM_LEAD_GAIN;
            double leadMagnitude = Math.hypot(leadX, leadY);
            if (leadMagnitude > SOTM_MAX_LEAD_IN && leadMagnitude > 1e-6) {
                double scale = SOTM_MAX_LEAD_IN / leadMagnitude;
                leadX *= scale;
                leadY *= scale;
            }
        }

        double virtualTargetX = targetX - leadX;
        double virtualTargetY = targetY - leadY;
        double virtualFieldAngleRad = Math.atan2(virtualTargetY - botY, virtualTargetX - botX);
        double turretRobotRelativeAimDeg = Math.toDegrees(normalizeRadians(virtualFieldAngleRad - botHeadingRad));
        if (leadApplied) {
            turretRobotRelativeAimDeg += Math.toDegrees(
                    omegaRadPerSec * totalTimeSec * SOTM_ANGULAR_LEAD_GAIN
            );
        }

        double turretLagCompensationDeg = 0.0;
        if (sotmControlActive && SOTM_TURRET_LAG_COMP_ENABLED) {
            turretLagCompensationDeg = Math.toDegrees(omegaRadPerSec) * SOTM_TURRET_LAG_COMP_SEC;
            double maxLagCompDeg = Math.max(0.0, SOTM_TURRET_LAG_COMP_MAX_DEG);
            turretLagCompensationDeg = Math.max(
                    -maxLagCompDeg,
                    Math.min(maxLagCompDeg, turretLagCompensationDeg)
            );
            turretRobotRelativeAimDeg += turretLagCompensationDeg;
        }

        // Keep effective distance tied to real target vector (Stage 3 guidance, avoids double-counting).
        double radialVelocityInPerSec = (vxInPerSec * dx + vyInPerSec * dy) / realDistance;
        double effectiveDistance = Math.max(0.0, realDistance - (radialVelocityInPerSec * totalTimeSec));
        double distanceForBallistics = leadApplied
                ? Math.max(minDisatanceForShooting, effectiveDistance)
                : realDistance;

        boolean valid =
                Double.isFinite(totalTimeSec) &&
                        Double.isFinite(leadX) &&
                        Double.isFinite(leadY) &&
                        Double.isFinite(radialVelocityInPerSec) &&
                        Double.isFinite(effectiveDistance) &&
                        Double.isFinite(distanceForBallistics) &&
                        Double.isFinite(turretRobotRelativeAimDeg);

        return new SOTMResult(
                valid,
                leadApplied,
                speedInPerSec,
                totalTimeSec,
                leadX,
                leadY,
                radialVelocityInPerSec,
                effectiveDistance,
                distanceForBallistics,
                turretRobotRelativeAimDeg,
                turretLagCompensationDeg
        );
    }

    private double getSotmTotalTimeSeconds(double distanceInches) {
        return SOTM_MECHANICAL_DELAY_SEC +
                SOTM_BALL_TRANSFER_TIME_SEC +
                getSotmFlightTimeSeconds(distanceInches);
    }

    private double getSotmFlightTimeSeconds(double distanceInches) {
        if (SOTM_USE_TOF_LOOKUP) {
            return interpolateSotmLookup(
                    distanceInches,
                    SOTM_TOF_DISTANCE_IN,
                    SOTM_TOF_FLIGHT_TIME_SEC
            );
        }
        return distanceInches / Math.max(1e-6, SOTM_ESTIMATED_BALL_SPEED_IN_PER_SEC);
    }

    private static double interpolateSotmLookup(double x, double[] xs, double[] ys) {
        if (xs.length == 0 || ys.length == 0 || xs.length != ys.length) return 0.0;
        if (x <= xs[0]) return ys[0];
        int last = xs.length - 1;
        if (x >= xs[last]) return ys[last];

        for (int i = 0; i < last; i++) {
            double x0 = xs[i];
            double x1 = xs[i + 1];
            if (x >= x0 && x <= x1) {
                double t = (x - x0) / Math.max(1e-6, x1 - x0);
                return ys[i] + (t * (ys[i + 1] - ys[i]));
            }
        }
        return ys[last];
    }

    private double updateSotmFilteredOmega(double rawOmegaRadPerSec) {
        double alpha = Math.max(0.0, Math.min(1.0, SOTM_OMEGA_FILTER_ALPHA));
        if (!sotmOmegaFilterInitialized) {
            sotmFilteredOmegaRadPerSec = rawOmegaRadPerSec;
            sotmOmegaFilterInitialized = true;
            return sotmFilteredOmegaRadPerSec;
        }
        sotmFilteredOmegaRadPerSec =
                (alpha * rawOmegaRadPerSec) + ((1.0 - alpha) * sotmFilteredOmegaRadPerSec);
        return sotmFilteredOmegaRadPerSec;
    }

    private static double robotFrontRelativeToTurretUnwrappedDegrees(double robotFrontRelativeDegrees) {
        return (TurretSubsystem.ROBOT_FRONT_RELATIVE_SIGN * robotFrontRelativeDegrees) +
                TurretSubsystem.ROBOT_FRONT_TO_TURRET_ZERO_OFFSET_DEGREES;
    }

    private static double smallestWrappedDeltaDegrees(double targetDegrees, double currentDegrees) {
        double delta = targetDegrees - currentDegrees;
        while (delta > 180.0) delta -= 360.0;
        while (delta < -180.0) delta += 360.0;
        return delta;
    }

    private static double normalizeDegrees(double angleDegrees) {
        while (angleDegrees >= 180.0) angleDegrees -= 360.0;
        while (angleDegrees < -180.0) angleDegrees += 360.0;
        return angleDegrees;
    }

    /**
     * Look up the calibration table for the given robot pose, transparently
     * mirroring in and out when the alliance is RED. The calibration samples
     * are all authored in BLUE-side coordinates: aim points cluster near
     * {@code x ≈ 0, y ≈ 130} (the blue goal). For RED we reflect the robot
     * pose across the field centerline before lookup. Zone offsets are applied
     * in that same blue-authored calibration space, then the returned aim point
     * is reflected back, so the same table and trims drive both sides.
     * Uses Pedro's {@link Pose#mirror(double)} so the mirror axis stays
     * consistent with everywhere else we flip red/blue poses, and the
     * field-width argument lets us retune if an event's field is sized
     * slightly differently than the nominal 144".
     * RPM and hood position are invariant under the mirror (distance to the
     * mirrored goal equals distance to the real goal), so they pass through
     * unchanged.
     */
    private static ShotSolution lookupShotForAlliance(double botX, double botY) {
        boolean isRed = GlobalRobotData.allianceSide == GlobalRobotData.COLOR.RED;
        if (!isRed) {
            return applyShotZoneOffsets(
                    botX,
                    botY,
                    ShotCalibrationTable.active().lookup(botX, botY)
            );
        }
        Pose mirroredBot = new Pose(botX, botY, 0.0).mirror(FIELD_WIDTH_IN);
        ShotSolution sol = applyShotZoneOffsets(
                mirroredBot.getX(),
                mirroredBot.getY(),
                ShotCalibrationTable.active().lookup(mirroredBot.getX(), mirroredBot.getY())
        );
        Pose mirroredAim = new Pose(sol.aimX, sol.aimY, 0.0).mirror(FIELD_WIDTH_IN);
        return new ShotSolution(
                sol.rpm,
                sol.hoodPos,
                mirroredAim.getX(),
                mirroredAim.getY(),
                sol.sourceIdxs,
                sol.weights,
                sol.extrapolated,
                sol.nearestDistanceIn
        );
    }

    /**
     * Apply a configurable global delta on top of the table solution for the
     * legal shooting zone that contains the current robot pose. OUT-of-zone
     * poses are left unchanged so diagnostics near boundaries do not get an
     * unexpected trim. Callers pass blue-side coordinates, so the same offsets
     * are mirror-safe for both alliances.
     */
    private static ShotSolution applyShotZoneOffsets(double botX, double botY, ShotSolution baseSol) {
        String zone = ShootingZones.zoneLabel(botX, botY);
        double rpmOffset = 0.0;
        double aimXOffset = 0.0;
        double aimYOffset = 0.0;
        if ("A".equals(zone)) {
            rpmOffset = SHOT_ZONE_A_RPM_OFFSET;
            aimXOffset = SHOT_ZONE_A_TARGET_X_OFFSET_IN;
            aimYOffset = SHOT_ZONE_A_TARGET_Y_OFFSET_IN;
        } else if ("B".equals(zone)) {
            rpmOffset = SHOT_ZONE_B_RPM_OFFSET;
            aimXOffset = SHOT_ZONE_B_TARGET_X_OFFSET_IN;
            aimYOffset = SHOT_ZONE_B_TARGET_Y_OFFSET_IN;
        }
        if (Math.abs(rpmOffset) < 1e-6 &&
                Math.abs(aimXOffset) < 1e-6 &&
                Math.abs(aimYOffset) < 1e-6) {
            return baseSol;
        }
        return new ShotSolution(
                baseSol.rpm + rpmOffset,
                baseSol.hoodPos,
                baseSol.aimX + aimXOffset,
                baseSol.aimY + aimYOffset,
                baseSol.sourceIdxs,
                baseSol.weights,
                baseSol.extrapolated,
                baseSol.nearestDistanceIn
        );
    }

    private static Pose convertLimelightBotposeToPedro(Pose3D botpose) {
        double xInches = DistanceUnit.METER.toInches(botpose.getPosition().x);
        double yInches = DistanceUnit.METER.toInches(botpose.getPosition().y);
        Pose2D pose2d = new Pose2D(
                DistanceUnit.INCH,
                xInches,
                yInches,
                AngleUnit.DEGREES,
                botpose.getOrientation().getYaw()
        );
        Pose ftcStandardPose = PoseConverter.pose2DToPose(pose2d, InvertedFTCCoordinates.INSTANCE);
        return new Pose(
                (ftcStandardPose.getY() + 72),
                (-(ftcStandardPose.getX()) + 72),
                ftcStandardPose.getHeading() - Math.toRadians(90)
        );
    }

    private void resetLimelightVisionBlendState() {
        Arrays.fill(limelightVisionDxHistory, 0.0);
        Arrays.fill(limelightVisionDyHistory, 0.0);
        Arrays.fill(limelightVisionDhHistoryDeg, 0.0);
        limelightVisionHistoryNextIdx = 0;
        limelightVisionHistoryFill = 0;
        limelightVisionBiasXIn = 0.0;
        limelightVisionBiasYIn = 0.0;
        limelightVisionBiasHeadingDeg = 0.0;
        limelightVisionLastNudgeXIn = 0.0;
        limelightVisionLastNudgeYIn = 0.0;
        limelightVisionLastNudgeHeadingDeg = 0.0;
        limelightVisionLoopsSinceApply = Integer.MAX_VALUE / 2;
        limelightVisionConsecutiveAccepts = 0;
        // Also clear any trim the PoseTracker currently has applied so the
        // next correction starts from a clean slate (important after dpad-up
        // emergency full-pose reset, which already zeroes offsets internally,
        // and on opmode init).
        try {
            PoseTracker pt = PedroComponent.follower().poseTracker;
            pt.setXOffset(0.0);
            pt.setYOffset(0.0);
            pt.setHeadingOffset(0.0);
        } catch (Exception ignored) {
            // Follower may not be initialized yet on very early calls.
        }
    }

    private void pushLimelightVisionHistory(double dxIn, double dyIn, double dhDeg) {
        limelightVisionDxHistory[limelightVisionHistoryNextIdx] = dxIn;
        limelightVisionDyHistory[limelightVisionHistoryNextIdx] = dyIn;
        limelightVisionDhHistoryDeg[limelightVisionHistoryNextIdx] = dhDeg;
        limelightVisionHistoryNextIdx =
                (limelightVisionHistoryNextIdx + 1) % LIMELIGHT_VISION_HISTORY_CAPACITY;
        limelightVisionHistoryFill = Math.min(
                LIMELIGHT_VISION_HISTORY_CAPACITY,
                limelightVisionHistoryFill + 1
        );
    }

    private double getRecentVisionHistoryMedian(double[] history) {
        int count = Math.min(
                Math.max(1, LIMELIGHT_VISION_BIAS_WINDOW),
                Math.min(limelightVisionHistoryFill, LIMELIGHT_VISION_HISTORY_CAPACITY)
        );
        if (count <= 0) {
            return 0.0;
        }
        double[] values = new double[count];
        for (int i = 0; i < count; i++) {
            int historyIdx = limelightVisionHistoryNextIdx - count + i;
            while (historyIdx < 0) historyIdx += LIMELIGHT_VISION_HISTORY_CAPACITY;
            values[i] = history[historyIdx % LIMELIGHT_VISION_HISTORY_CAPACITY];
        }
        Arrays.sort(values);
        int mid = count / 2;
        if ((count & 1) == 1) {
            return values[mid];
        }
        return 0.5 * (values[mid - 1] + values[mid]);
    }

    private double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    private double applySotmRpmCompensation(
            double baseRpm,
            double radialVelocityInPerSec,
            boolean topTriangleLimitButtonHeld
    ) {
        double rpm = baseRpm;
        boolean away = false;

        if (SOTM_RPM_DIRECTION_COMP_ENABLED && Double.isFinite(radialVelocityInPerSec)) {
            double absRadial = Math.abs(radialVelocityInPerSec);

            // Buffer zone: if barely moving toward/away, do NOT change RPM.
            double deadband = Math.max(0.0, SOTM_RPM_DIRECTION_MIN_RADIAL_SPEED_IN_PER_SEC);

            // Speed where we allow the full multiplier.
            // Tune this based on robot speed. Higher = less aggressive.
            double fullEffectSpeed = 45.0;

            if (absRadial > deadband) {
                double t = (absRadial - deadband) / Math.max(1.0, fullEffectSpeed - deadband);
                t = Math.max(0.0, Math.min(1.0, t));

                // Smooth curve: small correction at medium speed, full correction at high speed.
                t = t * t;

                if (radialVelocityInPerSec > 0.0) {
                    // Driving meaningfully TOWARD goal: slightly reduce RPM.
                    double towardMult = 0.9;
                    rpm *= 1.0 - ((1.0 - towardMult) * t);
                } else {
                    // Driving meaningfully AWAY from goal: increase RPM, but not crazy.
                    double awayMult = 1.9; // 1.5 was very aggressive
                    rpm *= 1.0 + ((awayMult - 1.0) * t);
                }
            }
        }

        if (SOTM_TOP_TRIANGLE_RPM_LIMIT_ENABLED && topTriangleLimitButtonHeld && !away) {
            rpm = Math.min(rpm, SHOOTER_IDLE_RPM);
        }

        return Math.max(0.0, rpm);
    }

    private Pose getTurretSafePose(Pose rawPose) {
        long nowMs = System.currentTimeMillis();

        if (lastRawPoseForSlipCheck == null) {
            lastRawPoseForSlipCheck = rawPose.copy();
            lastTurretAimPose = rawPose.copy();
            return rawPose;
        }

        double dx = rawPose.getX() - lastRawPoseForSlipCheck.getX();
        double dy = rawPose.getY() - lastRawPoseForSlipCheck.getY();
        double jump = Math.hypot(dx, dy);

        boolean odometryJumped = jump > TURRET_ODOMETRY_SLIP_JUMP_IN;

        if (odometryJumped) {
            lastOdoSlipMs = nowMs;
        }

        lastRawPoseForSlipCheck = rawPose.copy();

        if (nowMs - lastOdoSlipMs < TURRET_ODOMETRY_SLIP_HOLD_MS && lastTurretAimPose != null) {
            return lastTurretAimPose;
        }

        lastTurretAimPose = rawPose.copy();
        return rawPose;
    }
}



