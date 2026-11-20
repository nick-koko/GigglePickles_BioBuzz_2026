package org.firstinspires.ftc.teamcode.logging;

import android.os.Environment;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GlobalRobotData;
import org.firstinspires.ftc.teamcode.pedroPathing.Pickles2026Teleop;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.subsystems.shot.ShotCalibrationTable2026;
import org.firstinspires.ftc.teamcode.util.TurretController2026;
import java.io.File;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.text.SimpleDateFormat;
import java.util.*;

/** Read-only logging facade. Called on the OpMode thread; files belong to a worker thread. */
@Configurable
public final class RobotLogging2026 {
    public static boolean ENABLE_MATCH_LOG = true;
    public static boolean ENABLE_SHOT_DETAIL_LOG = true;
    public static double MATCH_SAMPLE_INTERVAL_MS = 50.0;
    // Zero records every observed OpMode loop, including pickup, spin-up and recovery.
    public static double DETAIL_SAMPLE_INTERVAL_MS = 0.0;
    public enum StorageMode { MEMORY_UNTIL_STOP, BACKGROUND_STREAMING }
    // Choose before Play. Detailed logging does not override this storage policy.
    public static StorageMode STORAGE_MODE = StorageMode.MEMORY_UNTIL_STOP;
    public static int MEMORY_MAX_ROWS = 40000;
    public static int WRITER_QUEUE_CAPACITY = 2048;
    public static double AUTO_MEMORY_ONLY_SECONDS = 35.0;
    public static double TELEOP_MEMORY_ONLY_SECONDS = 125.0;
    public static boolean ENABLE_LONG_PRACTICE_WRITES = true;
    public static double PRACTICE_CHECKPOINT_INTERVAL_SECONDS = 10.0;
    // Both switches must be true to permit early post-shot writes in a practice run.
    public static boolean PRACTICE_MODE = false;
    public static boolean ENABLE_POST_SHOT_PRACTICE_WRITES = false;
    public static double POST_SHOT_WRITE_DELAY_SECONDS = 1.0;
    public static String PROFILE_LABEL = "baseline";
    public static String RUN_NOTES = "";
    // Set before testing (for example, a short git revision plus "dirty" if applicable).
    public static String BUILD_LABEL = "unspecified";

    public static final String[] DETAIL_EXTRA_HEADER = ("shooter_sample_age_ms,shooter_loop_ms,"
            + "rpm_error,integral_accumulator,derivative_rpm_s,target_acceleration_rpm_s,"
            + "p_term,i_term,d_term,static_ff,velocity_ff,acceleration_ff,total_ff,"
            + "output_before_voltage_comp,voltage_comp_gain,controller_power_before_boost,"
            + "no_boost_output_same_loop,boost_requested_power_increase,boost_applied_power_difference,"
            + "output_before_slew,output_after_slew,slew_limited,saturated,shooter_current_a,"
            + "encoder_delta_motor_rpm,encoder_delta_flywheel_rpm,measured_motor_rpm,"
            + "minimum_power_override,boost_bypasses_slew,kP,kI,kD,kS,kV,kA,slew_per_s,"
            + "i_zone_rpm,integral_min,integral_max,encoder_ticks_per_motor_rev,motor_per_flywheel_ratio,"
            + "voltage_comp_enabled,voltage_comp_nominal_v,voltage_comp_min_gain,voltage_comp_max_gain,"
            + "voltage_comp_filter_tau_s,feed_duration_s,boost_enabled_for_sequence,boost_delay_s,"
            + "boost_duration_s,boost_minimum_power,boost_bypass_for_sequence,"
            + "intake_applied_control_mode,intake_velocity_command_rpm,transfer_velocity_command_rpm,"
            + "intake_power_command,transfer_power_command,intake_controller_power_readback,"
            + "transfer_controller_power_readback,intake_motor_rpm,transfer_motor_rpm,"
            + "intake_shaft_rpm,transfer_shaft_rpm,intake_forward_setting_rpm,transfer_forward_setting_rpm,"
            + "shoot_intake_setting_power,shoot_transfer_setting_power,intake_capacity_stop,"
            + "transfer_capacity_stop,tracking_enabled,simulated_storage_full,manual_motor_test,"
            + "gate_servo_command,gate_open_setting,gate_closed_setting,"
            + "beam_shooter_blocked,beam_middle_high_blocked,beam_middle_low_blocked,beam_entry_blocked,"
            + "beam_shooter_block_edges,beam_shooter_clear_edges,beam_middle_high_block_edges,"
            + "beam_middle_high_clear_edges,beam_middle_low_block_edges,beam_middle_low_clear_edges,"
            + "beam_entry_block_edges,beam_entry_clear_edges,turret_commanded_deg,turret_servo_left,"
            + "turret_servo_right,turret_servo_zero_offset_deg,turret_absolute_raw_deg,"
            + "turret_cached_quad_ticks,turret_calibrated,turret_status,flower_servo_command,"
            + "regression_rpm_per_in,regression_intercept,operator_rpm_trim,shot_table_enabled,"
            + "controller_target_rpm,startup_profile_enabled,asymmetric_slew_enabled,"
            + "startup_slew_up_per_s,startup_slew_down_per_s,running_slew_up_per_s,running_slew_down_per_s,"
            + "startup_entry_max_rpm,startup_exit_tolerance_rpm,startup_exit_max_slope_rpm_s,"
            + "startup_exit_stable_s,startup_timeout_s,active_slew_up_per_s,active_slew_down_per_s,"
            + "measured_acceleration_rpm_s,boost_profile_ramped_for_sequence,boost_start_power_for_sequence,"
            + "boost_end_power_for_sequence,boost_voltage_scaled_for_sequence,boost_nominal_v_for_sequence,"
            + "boost_taper_rpm_for_sequence,boost_floor_requested,controller_sample_override_floor,"
            + "boost_voltage_gain,boost_floor_voltage_scaled,boost_overspeed_multiplier,"
            + "boost_floor_effective,controller_sample_boost_bypass").split(",");
    public static final String[] DETAIL_HEADER = concatenate(RobotLogFrame2026.HEADER, DETAIL_EXTRA_HEADER);

    private AsyncCsvWriter2026 writer;
    private LogWritePolicy2026 writePolicy;
    private StorageMode storageMode;
    private MatchLogRecorder2026 recorder;
    private ShotSequenceController2026 sequence;
    private long startNanos, lastSampleNanos, loop, nextConfigurationNanos;
    private double lastDetailMs = Double.NEGATIVE_INFINITY, previousCaptureMs, maxCaptureMs;
    private String session, opmode, phase, captureError = "";
    private String lastMatchConfiguration = "", lastDetailConfiguration = "";
    private boolean robotAvailable, started, detailPreviously, contextSet;
    private double aimX, aimY;
    private boolean aimRequested, fireRequested, firePermitted;
    private int previousBeams = -1;
    private final int[] blockEdges = new int[4], clearEdges = new int[4];
    private RobotLogFrame2026 latest;

    public void start(String opmode, String phase, ShotSequenceController2026 sequence,
                      boolean robotAvailable) {
        if (started) stop();
        this.opmode = opmode;
        this.phase = phase;
        this.sequence = sequence;
        this.robotAvailable = robotAvailable;
        storageMode = STORAGE_MODE == null ? StorageMode.MEMORY_UNTIL_STOP : STORAGE_MODE;
        writePolicy = new LogWritePolicy2026();
        startNanos = System.nanoTime();
        String date = new SimpleDateFormat("yyyyMMdd_HHmmss_SSS", Locale.US).format(new Date());
        session = date + "_" + opmode.replaceAll("[^A-Za-z0-9_-]", "_")
                + "_" + UUID.randomUUID().toString().substring(0, 8);
        writer = null;
        recorder = null;
        latest = null;
        lastSampleNanos = loop = nextConfigurationNanos = 0;
        lastDetailMs = Double.NEGATIVE_INFINITY;
        previousCaptureMs = maxCaptureMs = 0;
        previousBeams = -1;
        Arrays.fill(blockEdges, 0);
        Arrays.fill(clearEdges, 0);
        captureError = lastMatchConfiguration = lastDetailConfiguration = "";
        contextSet = detailPreviously = false;
        started = true;
    }

    /** TeleOp passes the actual selected/calibrated aim point and input/interlock decisions. */
    public void setShotContext(double x, double y, boolean aim, boolean requested, boolean permitted) {
        contextSet = true;
        aimX = x; aimY = y;
        aimRequested = aim; fireRequested = requested; firePermitted = permitted;
    }

    public void update(Pose pose, Follower follower, String stage, String plan, String rpmSource) {
        if (!started || !captureError.isEmpty()) return;
        long begin = System.nanoTime();
        try {
            RobotLogFrame2026 frame = capture(pose, follower, stage, plan, rpmSource, begin);
            latest = frame;
            if (writer == null && (ENABLE_MATCH_LOG || ENABLE_SHOT_DETAIL_LOG)) {
                File root = new File(Environment.getExternalStorageDirectory(), "FIRST/logs2026");
                writer = new AsyncCsvWriter2026(new File(root, session), RobotLogFrame2026.HEADER,
                        DETAIL_HEADER, storageMode == StorageMode.MEMORY_UNTIL_STOP
                                ? MEMORY_MAX_ROWS : WRITER_QUEUE_CAPACITY,
                        storageMode == StorageMode.MEMORY_UNTIL_STOP);
                recorder = new MatchLogRecorder2026(writer::write);
                if (ENABLE_MATCH_LOG) recorder.event(frame, "SESSION_START", RUN_NOTES);
            }
            if (writer == null) return;
            recorder.update(frame, ENABLE_MATCH_LOG, finiteInterval(MATCH_SAMPLE_INTERVAL_MS, 50));
            if (ENABLE_MATCH_LOG && detailPreviously != ENABLE_SHOT_DETAIL_LOG)
                recorder.event(frame, "DETAIL_LOG", ENABLE_SHOT_DETAIL_LOG ? "ENABLED" : "DISABLED");
            detailPreviously = ENABLE_SHOT_DETAIL_LOG;
            if (!ENABLE_MATCH_LOG) lastMatchConfiguration = "";
            if (!ENABLE_SHOT_DETAIL_LOG) lastDetailConfiguration = "";
            if ((ENABLE_MATCH_LOG || ENABLE_SHOT_DETAIL_LOG)
                    && (begin >= nextConfigurationNanos
                    || (ENABLE_MATCH_LOG && lastMatchConfiguration.isEmpty())
                    || (ENABLE_SHOT_DETAIL_LOG && lastDetailConfiguration.isEmpty()))) {
                String configuration = configuration();
                if (ENABLE_MATCH_LOG && !configuration.equals(lastMatchConfiguration)) {
                    recorder.event(frame, "CONFIG", configuration);
                    lastMatchConfiguration = configuration;
                }
                if (ENABLE_SHOT_DETAIL_LOG && !configuration.equals(lastDetailConfiguration)) {
                    // Metadata has common context but blank detailed measurements. No extra
                    // motor reads, and no stale detail values presented as a fresh sample.
                    Object[] metadata = Arrays.copyOf(frame.values("CONFIG", configuration), DETAIL_HEADER.length);
                    if (writer.write(true, metadata)) lastDetailConfiguration = configuration;
                }
                nextConfigurationNanos = begin + 1_000_000_000L;
            }
            if (ENABLE_SHOT_DETAIL_LOG && frame.timeMs - lastDetailMs
                    >= finiteInterval(DETAIL_SAMPLE_INTERVAL_MS, 0)) {
                Object[] basic = frame.values("DETAIL", "");
                Object[] extra = detailValues(begin);
                Object[] row = Arrays.copyOf(basic, basic.length + extra.length);
                System.arraycopy(extra, 0, row, basic.length, extra.length);
                writer.write(true, row);
                lastDetailMs = frame.timeMs;
            }
            if (storageMode == StorageMode.MEMORY_UNTIL_STOP) {
                double protectedSeconds = "AUTO".equals(phase)
                        ? finiteInterval(AUTO_MEMORY_ONLY_SECONDS, 35)
                        : finiteInterval(TELEOP_MEMORY_ONLY_SECONDS, 125);
                String reason = writePolicy.update(frame.timeMs / 1000, frame.sequenceActive,
                        protectedSeconds, ENABLE_LONG_PRACTICE_WRITES, PRACTICE_CHECKPOINT_INTERVAL_SECONDS,
                        PRACTICE_MODE, ENABLE_POST_SHOT_PRACTICE_WRITES, ENABLE_SHOT_DETAIL_LOG,
                        POST_SHOT_WRITE_DELAY_SECONDS);
                if (!reason.isEmpty() && writer.getQueueSize() > 0) {
                    if (ENABLE_MATCH_LOG) recorder.event(frame, "CHECKPOINT", reason);
                    writer.checkpoint();
                }
            }
        } catch (RuntimeException exception) {
            // A logging failure must never stop the match or command an actuator.
            captureError = exception.toString();
            if (writer != null) writer.close();
        } finally {
            previousCaptureMs = (System.nanoTime() - begin) / 1e6;
            maxCaptureMs = Math.max(maxCaptureMs, previousCaptureMs);
        }
    }

    private RobotLogFrame2026 capture(Pose pose, Follower follower, String stage, String plan,
                                      String rpmSource, long now) {
        RobotLogFrame2026 f = new RobotLogFrame2026();
        f.session = session; f.opmode = opmode; f.phase = phase; f.profile = PROFILE_LABEL;
        f.timeMs = (now - startNanos) / 1e6; f.epochMs = System.currentTimeMillis();
        f.periodMs = lastSampleNanos == 0 ? Double.NaN : (now - lastSampleNanos) / 1e6;
        lastSampleNanos = now; f.loop = ++loop; f.previousCaptureMs = previousCaptureMs;
        f.stage = stage; f.plan = plan; f.rpmSource = rpmSource;
        if (pose != null) { f.x = pose.getX(); f.y = pose.getY(); f.heading = Math.toDegrees(pose.getHeading()); }
        if (follower != null) {
            f.vx = follower.getVelocity().getXComponent();
            f.vy = follower.getVelocity().getYComponent();
            f.omega = Math.toDegrees(follower.getPoseTracker().getAngularVelocity());
            f.busy = follower.isBusy();
            if (follower.getCurrentPath() != null) {
                f.pathIndex = follower.getCurrentPathNumber(); f.pathT = follower.getCurrentTValue();
                f.pathHeading = Math.toDegrees(follower.getClosestPointHeadingGoal());
            }
        }
        if (contextSet) {
            f.targetX = aimX; f.targetY = aimY; f.aimRequested = aimRequested;
            f.fireRequested = fireRequested; f.firePermitted = firePermitted;
        } else if (pose != null) {
            // Reference goal for distance calculations, not a claim that auto turret aiming is active.
            f.targetX = Pickles2026Teleop.SHOOT_TARGET_X;
            f.targetY = pose.getY() > Pickles2026Teleop.SHOOT_TARGET_Y_SPLIT
                    ? Pickles2026Teleop.SHOOT_TARGET_Y_ABOVE_SPLIT : Pickles2026Teleop.SHOOT_TARGET_Y;
        }
        IntakeWithSensorsSubsystem2026 i = IntakeWithSensorsSubsystem2026.INSTANCE;
        ShooterSubsystem2026 s = ShooterSubsystem2026.INSTANCE;
        f.ballCount = i.getBallCount(); f.beams = i.getCachedBeamMask();
        f.occupied = (i.isShooterPositionOccupied() ? 1 : 0) | (i.isMiddleHighPositionOccupied() ? 2 : 0)
                | (i.isMiddleLowPositionOccupied() ? 4 : 0) | (i.isIntakeEntryPositionOccupied() ? 8 : 0);
        f.beamAgeMs = ageMs(now, i.getBeamSampleNanos());
        for (int bit = 0; bit < 4; bit++) if (previousBeams >= 0 && ((previousBeams ^ f.beams) & (1 << bit)) != 0) {
            if ((f.beams & (1 << bit)) != 0) blockEdges[bit]++; else clearEdges[bit]++;
        }
        previousBeams = f.beams;
        f.feeding = i.isFeedingToShooter();
        f.sequenceId = sequence.getSequenceCount(); f.sequenceActive = sequence.isActive();
        f.sequenceElapsed = sequence.getElapsedSeconds();
        f.sequenceStartMs = sequence.getSequenceStartNanos() == 0 ? Double.NaN
                : (sequence.getSequenceStartNanos() - startNanos) / 1e6;
        f.startBallCount = sequence.getStartBallCount(); f.startBeams = sequence.getStartBeamMask();
        f.endReason = sequence.getEndReason();
        if (!contextSet) f.fireRequested = f.sequenceActive;
        f.shooterEnabled = s.isEnabled(); f.targetRpm = s.getTargetFlywheelRpm();
        f.shooterPowerProfile = s.getPowerProfileState();
        f.shooterStartupElapsed = s.getStartupElapsedSeconds();
        f.shooterStartupExitReason = s.getStartupExitReason();
        f.measuredRpm = s.getMeasuredFlywheelRpm(); f.rawV = s.getBatteryVoltageRaw();
        f.filteredV = s.getBatteryVoltageFiltered(); f.boostRequested = sequence.isBoostActive();
        f.boostApplied = s.isMinimumPowerOverrideActive(); f.appliedPower = s.getAppliedPower();
        f.intakeMode = i.getIntakeMode().name(); f.jamReverse = i.isJamReversing();
        if (robotAvailable) {
            f.flower = FlowerIntakeSubsystem2026.INSTANCE.getPosition().name();
            TurretController2026 t = TurretSubsystem2026.INSTANCE.getLoggingController();
            if (t != null) {
                f.turretMode = t.getMode().name();
                f.turretState = t.isServoOnlyFallback()
                        && t.getState() == TurretController2026.State.READY
                        ? "SERVO_ONLY" : t.getState().name();
                if (t.getMode() == TurretController2026.Mode.CHASSIS_ONLY) {
                    f.turretRelative = TurretSubsystem2026.CHASSIS_ONLY_SHOOTER_HEADING_DEGREES;
                } else {
                    f.turretMeasured = t.getMode() == TurretController2026.Mode.TRACKING && t.isCalibrated()
                            ? t.getMeasuredAngle() : Double.NaN;
                    f.turretTarget = t.getTarget(); f.turretVelocity = t.getVelocity();
                    double angle = t.getMode() == TurretController2026.Mode.FIXED ? t.getCommandedAngle() : f.turretMeasured;
                    f.turretRelative = (angle - TurretSubsystem2026.ROBOT_FRONT_TO_TURRET_ZERO_OFFSET_DEGREES)
                            / TurretSubsystem2026.ROBOT_FRONT_RELATIVE_SIGN;
                }
                f.turretReady = t.isAimReady(); f.turretReachable = t.isTargetReachable();
            }
        }
        if (writer != null) {
            f.droppedMatch = writer.getDroppedMatch(); f.droppedDetail = writer.getDroppedDetail();
            f.queueRows = writer.getQueueSize();
        }
        return f;
    }

    private Object[] detailValues(long now) {
        ShooterSubsystem2026 s = ShooterSubsystem2026.INSTANCE;
        IntakeWithSensorsSubsystem2026 i = IntakeWithSensorsSubsystem2026.INSTANCE;
        TurretController2026 t = robotAvailable ? TurretSubsystem2026.INSTANCE.getLoggingController() : null;
        if (t != null && t.getMode() == TurretController2026.Mode.CHASSIS_ONLY) t = null;
        // Velocity getters are sampled ONCE each, only when the detail stream is due.
        double intakeRpm = i.getMeasuredIntakeMotorRpm(), transferRpm = i.getMeasuredTransferMotorRpm();
        int b = i.getCachedBeamMask();
        return new Object[]{ageMs(now, s.getControlSampleNanos()), s.getLoopDtSeconds() * 1000,
                s.getErrorRpm(), s.getIntegralAccumulator(), s.getDerivativeRpmPerSecond(),
                s.getTargetAccelerationRpmPerSecond(), s.getPTerm(), s.getITerm(), s.getDTerm(),
                s.getStaticFeedforward(), s.getVelocityFeedforward(), s.getAccelerationFeedforward(),
                s.getFeedforward(), s.getOutputBeforeVoltageComp(), s.getVoltageCompGain(),
                s.getControllerOutputBeforeBoost(), s.getOutputWithoutBoostThisLoop(),
                s.getBoostRequestedPowerIncrease(), s.getBoostAppliedPowerDifference(),
                s.getOutputBeforeSlew(), s.getOutputAfterSlew(), s.wasSlewLimited(), s.isCommandSaturated(),
                s.getMotorCurrentAmps(), s.getEncoderDeltaMotorRpm(), s.getEncoderDeltaFlywheelRpm(),
                s.getMeasuredMotorRpm(), s.getMinimumPowerOverride(), s.isPowerOverrideBypassingSlew(),
                ShooterSubsystem2026.kP, ShooterSubsystem2026.kI, ShooterSubsystem2026.kD,
                ShooterSubsystem2026.kS, ShooterSubsystem2026.kV, ShooterSubsystem2026.kA,
                ShooterSubsystem2026.SLEW_PER_SECOND, ShooterSubsystem2026.I_ZONE_RPM,
                ShooterSubsystem2026.INTEGRAL_MIN, ShooterSubsystem2026.INTEGRAL_MAX,
                ShooterSubsystem2026.ENCODER_TICKS_PER_MOTOR_REV, ShooterSubsystem2026.MOTOR_RPM_PER_FLYWHEEL_RPM,
                ShooterSubsystem2026.ENABLE_VOLTAGE_COMPENSATION, ShooterSubsystem2026.VOLTAGE_COMP_NOMINAL_V,
                ShooterSubsystem2026.VOLTAGE_COMP_MIN_GAIN, ShooterSubsystem2026.VOLTAGE_COMP_MAX_GAIN,
                ShooterSubsystem2026.VOLTAGE_COMP_FILTER_TAU_SEC, sequence.getActiveFeedSeconds(),
                sequence.isActiveBoostEnabled(), sequence.getActiveBoostDelaySeconds(),
                sequence.getActiveBoostDurationSeconds(), sequence.getActiveBoostMinimumPower(),
                sequence.isActiveBoostBypassingSlew(), i.getAppliedControlMode(),
                "POWER".equals(i.getAppliedControlMode()) ? Double.NaN : i.getAppliedIntakeRpm(),
                "POWER".equals(i.getAppliedControlMode()) ? Double.NaN : i.getAppliedTransferRpm(),
                "POWER".equals(i.getAppliedControlMode()) ? i.getAppliedIntakePower() : Double.NaN,
                "POWER".equals(i.getAppliedControlMode()) ? i.getAppliedTransferPower() : Double.NaN,
                i.getIntakeControllerPowerReadback(), i.getTransferControllerPowerReadback(), intakeRpm,
                transferRpm, intakeRpm / i.getShaftReduction(), transferRpm / i.getShaftReduction(),
                IntakeWithSensorsSubsystem2026.INTAKE_FORWARD_RPM, IntakeWithSensorsSubsystem2026.TRANSFER_FORWARD_RPM,
                IntakeWithSensorsSubsystem2026.SHOOT_INTAKE_POWER, IntakeWithSensorsSubsystem2026.SHOOT_TRANSFER_POWER,
                i.isIntakeStoppedForCapacity(), i.isTransferStoppedForCapacity(),
                IntakeWithSensorsSubsystem2026.ENABLE_BALL_TRACKING, i.isUpperStorageFullSimulated(),
                i.getManualMotorTestMode().name(), i.getGatePosition(), IntakeWithSensorsSubsystem2026.GATE_OPEN_POSITION,
                IntakeWithSensorsSubsystem2026.GATE_CLOSED_POSITION, (b & 1) != 0, (b & 2) != 0,
                (b & 4) != 0, (b & 8) != 0, blockEdges[0], clearEdges[0], blockEdges[1], clearEdges[1],
                blockEdges[2], clearEdges[2], blockEdges[3], clearEdges[3],
                t == null ? Double.NaN : t.getCommandedAngle(), t == null ? Double.NaN : t.getLeftPosition(),
                t == null ? Double.NaN : t.getRightPosition(), t == null ? Double.NaN : t.getServoOffset(),
                t == null ? Double.NaN : t.getRawDegrees(), t == null ? null : t.getCachedEncoderTicks(),
                t != null && t.isCalibrated(), t == null ? "UNAVAILABLE" : t.getStatus(),
                robotAvailable ? FlowerIntakeSubsystem2026.INSTANCE.getServoPosition() : Double.NaN,
                Pickles2026Teleop.REGRESSION_RPM_PER_INCH, Pickles2026Teleop.REGRESSION_RPM_INTERCEPT,
                Pickles2026Teleop.SHOOTER_RPM_TRIM, Pickles2026Teleop.USE_SHOT_TABLE,
                s.getControllerTargetFlywheelRpm(), ShooterSubsystem2026.ENABLE_STARTUP_PROFILE,
                ShooterSubsystem2026.ENABLE_ASYMMETRIC_SLEW,
                ShooterSubsystem2026.STARTUP_SLEW_UP_PER_SECOND, ShooterSubsystem2026.STARTUP_SLEW_DOWN_PER_SECOND,
                ShooterSubsystem2026.RUNNING_SLEW_UP_PER_SECOND, ShooterSubsystem2026.RUNNING_SLEW_DOWN_PER_SECOND,
                ShooterSubsystem2026.STARTUP_ENTRY_MAX_RPM, ShooterSubsystem2026.STARTUP_EXIT_TOLERANCE_RPM,
                ShooterSubsystem2026.STARTUP_EXIT_MAX_SLOPE_RPM_PER_SECOND,
                ShooterSubsystem2026.STARTUP_EXIT_STABLE_SECONDS, ShooterSubsystem2026.STARTUP_MAX_SECONDS,
                s.getActiveSlewUpPerSecond(), s.getActiveSlewDownPerSecond(), s.getMeasuredAccelerationRpmPerSecond(),
                sequence.isActiveRampedBoost(), sequence.getActiveBoostStartPower(), sequence.getActiveBoostEndPower(),
                sequence.isActiveBoostVoltageScaling(), sequence.getActiveBoostNominalV(), sequence.getActiveBoostTaperRpm(),
                sequence.getRequestedBoostFloor(), s.getSampleOverrideFloor(), s.getBoostVoltageGain(),
                s.getBoostScaledFloor(), s.getBoostOverspeedMultiplier(), s.getBoostEffectiveFloor(), s.didBoostBypassSlew()};
    }

    public void addTelemetry(Telemetry telemetry) {
        ShooterSubsystem2026 shooter = ShooterSubsystem2026.INSTANCE;
        telemetry.addData("Shooter profile / up / down", "%s / %.2f / %.2f power/s",
                shooter.getPowerProfileState(), shooter.getActiveSlewUpPerSecond(), shooter.getActiveSlewDownPerSecond());
        telemetry.addData("Shooter startup elapsed / exit", "%.2f s / %s",
                shooter.getStartupElapsedSeconds(), shooter.getStartupExitReason());
        telemetry.addData("Boost floor unscaled / scaled / effective", "%.3f / %.3f / %.3f",
                shooter.getSampleOverrideFloor(), shooter.getBoostScaledFloor(), shooter.getBoostEffectiveFloor());
        String error = !captureError.isEmpty() ? captureError : writer == null ? "" : writer.getError();
        telemetry.addData("Logging", !error.isEmpty() ? "ERROR: " + error
                : "match=" + ENABLE_MATCH_LOG + " detail=" + ENABLE_SHOT_DETAIL_LOG);
        telemetry.addData("Log capture last/max ms", "%.2f / %.2f", previousCaptureMs, maxCaptureMs);
        telemetry.addData("Log storage", storageMode == null ? STORAGE_MODE : storageMode);
        if (writer != null) {
            telemetry.addData("Log buffered / dropped match / detail", "%d / %d / %d",
                    writer.getQueueSize(), writer.getDroppedMatch(), writer.getDroppedDetail());
            telemetry.addData("Log rows written", writer.getWritten());
            telemetry.addData("Log folder", writer.getDirectory().getName());
        }
    }

    /** Must be called before resetting subsystems. Closing drains asynchronously, even if flags are off. */
    public void stop() {
        if (!started) return;
        started = false;
        if (writer != null) {
            if (latest != null && ENABLE_MATCH_LOG) {
                latest.timeMs = (System.nanoTime() - startNanos) / 1e6;
                latest.epochMs = System.currentTimeMillis();
                if (latest.sequenceActive) recorder.event(latest, "SHOT_END", "OPMODE_STOP_ACTIVE_FEED");
                recorder.event(latest, "SESSION_END", "Last cached sample; " + RUN_NOTES);
            }
            writer.close();
        }
    }

    private String configuration() {
        TreeMap<String, String> values = new TreeMap<>();
        List<Class<?>> types = new ArrayList<>(Arrays.asList(RobotLogging2026.class,
                ShooterSubsystem2026.class, ShotSequenceController2026.class, IntakeWithSensorsSubsystem2026.class,
                TurretSubsystem2026.class, FlowerIntakeSubsystem2026.class, Pickles2026Teleop.class,
                ShotCalibrationTable2026.class));
        try { types.add(Class.forName("org.firstinspires.ftc.teamcode.pedroPathing." + opmode)); }
        catch (ClassNotFoundException ignored) { /* Synthetic test session. */ }
        for (Class<?> type : types) for (Field f : type.getFields()) {
            if (!Modifier.isStatic(f.getModifiers()) || !(f.getType().isPrimitive()
                    || f.getType().isEnum() || f.getType() == String.class)) continue;
            try { values.put(type.getSimpleName() + "." + f.getName(), String.valueOf(f.get(null))); }
            catch (IllegalAccessException ignored) { /* Public fields only. */ }
        }
        values.put("alliance", String.valueOf(GlobalRobotData.allianceSide));
        ShotCalibrationTable2026.Sample[] samples = ShotCalibrationTable2026.SAMPLES;
        values.put("ShotCalibrationTable2026.SAMPLES.length", samples == null ? "null" : String.valueOf(samples.length));
        if (samples != null) for (int index = 0; index < samples.length; index++) {
            ShotCalibrationTable2026.Sample sample = samples[index];
            values.put("ShotCalibrationTable2026.SAMPLES[" + index + "]", sample == null ? "null"
                    : sample.robotX + "," + sample.robotY + "," + sample.flywheelRpm
                    + "," + sample.deltaAimX + "," + sample.deltaAimY);
        }
        StringBuilder result = new StringBuilder();
        for (Map.Entry<String, String> entry : values.entrySet())
            result.append(entry.getKey()).append('=').append(entry.getValue()).append('\n');
        return result.toString();
    }

    private static double ageMs(long now, long sample) { return sample == 0 ? Double.NaN : (now - sample) / 1e6; }
    private static double finiteInterval(double value, double fallback) {
        return Double.isFinite(value) ? Math.max(0, value) : fallback;
    }
    private static String[] concatenate(String[] first, String[] second) {
        String[] result = Arrays.copyOf(first, first.length + second.length);
        System.arraycopy(second, 0, result, first.length, second.length);
        return result;
    }
}
