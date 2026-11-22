package org.firstinspires.ftc.teamcode.testing.shots;

import com.bylazar.configurables.annotations.Configurable;

/** Panels settings used ONLY by the stationary shooting/model experiments. Read at Play. */
@Configurable
public final class ShotExperimentSettings2026 {
    private ShotExperimentSettings2026() { }
    public static ShotExperimentPlan2026.ComparisonSet COMPARISON_SET = ShotExperimentPlan2026.ComparisonSet.STARTUP;
    public static ShotExperimentPlan2026.VerificationProfile VERIFICATION_PROFILE = ShotExperimentPlan2026.VerificationProfile.CURRENT_SETTINGS;
    public static RecommendedShotPlans2026.Batch RECOMMENDED_BATCH = RecommendedShotPlans2026.Batch.QUICK_SCREEN;
    public static RecommendedShotPlans2026.ModelBatch MODEL_BATCH = RecommendedShotPlans2026.ModelBatch.STARTUP_AND_COAST;
    public static int RECOMMENDED_REPETITIONS = 1;
    public static boolean RECOMMENDED_COLD = true;
    public static boolean RECOMMENDED_ALREADY_SPINNING = true;
    public static double LOADED_RPM_BRACKET_OFFSET = 100.0;
    public static int MODEL_REPETITIONS = 1;
    public static double MODEL_RPM_LOW = 2800.0;
    public static double MODEL_RPM_CENTER = 3200.0;
    public static double MODEL_RPM_HIGH = 3600.0;
    public static double MODEL_SMALL_STEP_RPM = 200.0;
    public static double MODEL_LARGE_STEP_RPM = 450.0;
    public static double MODEL_STEP_RAMP_SECONDS = 1.0;
    public static double MODEL_PREPARE_SECONDS = 3.0;
    public static double MODEL_OBSERVE_SECONDS = 2.0;
    public static double MODEL_HOLD_SECONDS = 2.0;
    public static double MODEL_BETWEEN_TRIAL_SECONDS = 1.0;
    // Metadata only; set these to the actual physical flywheel for each dataset.
    public static int FLYWHEEL_WEIGHT_COUNT = 4;
    public static double FLYWHEEL_DIAMETER_MM = 72.0;
    public static int COMPARISON_REPETITIONS = 1;
    public static int VERIFICATION_REPETITIONS = 3;
    public static int PROFILE_ORDER_OFFSET = 0;
    public static boolean COMPARE_COLD = true;
    public static boolean COMPARE_ALREADY_SPINNING = true;
    public static boolean VERIFY_COLD = true;
    public static boolean VERIFY_ALREADY_SPINNING = true;
    public static boolean VERIFY_STEP_UP = true;
    public static boolean VERIFY_STEP_DOWN = true;
    public static double TARGET_RPM = 3196.0;
    public static double STEP_DELTA_RPM = 200.0;
    public static double TARGET_RAMP_SECONDS = 1.0;
    public static double COLD_FEED_DELAY_SECONDS = 2.0;
    public static double RUNNING_FEED_DELAY_SECONDS = 2.0;
    public static double RUNNING_PREPARE_SECONDS = 1.5;
    public static double POST_FEED_CAPTURE_SECONDS = 1.0;
    public static double TEST_FEED_SECONDS = 0.0; // Zero inherits each profile's duration; positive overrides all trials.
    public static double STOPPED_RPM_THRESHOLD = 50.0;
    public static double STOPPED_STABLE_SECONDS = 0.10;
    public static double COAST_TIMEOUT_SECONDS = 20.0;
    public static String RUN_LABEL = "stationary_shot_test";
    public static String BATTERY_CONDITION = "UNSPECIFIED"; // FRESH / MID / LOW; actual voltage is always logged.
    public static boolean ENABLE_DETAILED_TEST_LOGGING = true;
    // Optional manually measured pose/aim point. NaN means unknown, not odometry.
    public static double TEST_X_INCHES = Double.NaN;
    public static double TEST_Y_INCHES = Double.NaN;
    public static double TEST_HEADING_DEGREES = Double.NaN;
    public static double AIM_X_INCHES = Double.NaN;
    public static double AIM_Y_INCHES = Double.NaN;
    public static double FEED_MILD_SCALE = 0.85;
    public static double FEED_SLOW_SCALE = 0.70;

    public static boolean CUSTOM_STARTUP_PROFILE = true;
    public static double CUSTOM_STARTUP_UP_PER_S = 6.0;
    public static double CUSTOM_STARTUP_DOWN_PER_S = 6.0;
    public static boolean CUSTOM_ASYMMETRIC_RUNNING = false;
    public static double CUSTOM_RUNNING_UP_PER_S = 1.5;
    public static double CUSTOM_RUNNING_DOWN_PER_S = 1.5;
    public static double CUSTOM_KP = .0009, CUSTOM_KI = 0, CUSTOM_KD = 0;
    public static double CUSTOM_KS = .09, CUSTOM_KV = .000134, CUSTOM_KA = 0;
    public static double CUSTOM_FF_MIN_GAIN = .90, CUSTOM_FF_MAX_GAIN = 1.10;
    public static boolean CUSTOM_BOOST_ENABLED = true;
    public static boolean CUSTOM_BOOST_RAMP = true;
    public static boolean CUSTOM_BOOST_VOLTAGE_SCALING = true;
    public static boolean CUSTOM_BOOST_BYPASSES_SLEW = true;
    public static double CUSTOM_BOOST_DELAY_SECONDS = .04;
    public static double CUSTOM_BOOST_DURATION_SECONDS = .50;
    public static double CUSTOM_BOOST_FLAT_POWER = .75;
    public static double CUSTOM_BOOST_START_POWER = .80;
    public static double CUSTOM_BOOST_END_POWER = .90;
    public static double CUSTOM_BOOST_NOMINAL_V = 12.5;
    public static double CUSTOM_BOOST_TAPER_RPM = 100.0;
    public static double CUSTOM_FEED_SECONDS = 1.2;
    public static double CUSTOM_FEED_INTAKE_POWER = .50;
    public static double CUSTOM_FEED_TRANSFER_POWER = .50;
}
