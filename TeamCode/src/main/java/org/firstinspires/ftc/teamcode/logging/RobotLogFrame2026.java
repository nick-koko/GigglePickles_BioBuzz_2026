package org.firstinspires.ftc.teamcode.logging;

/** Immutable-at-enqueue scalar snapshot; all angles/degrees and positions/inches are explicit. */
public final class RobotLogFrame2026 {
    public static final String[] HEADER = ("schema_version,session_id,opmode,phase,profile,"
            + "match_t_ms,epoch_ms,loop_index,loop_period_ms,previous_capture_ms,event,note,"
            + "stage,plan,robot_x_in,robot_y_in,robot_heading_deg,robot_vx_in_s,robot_vy_in_s,"
            + "robot_omega_deg_s,follower_busy,path_index,path_t,path_heading_goal_deg,"
            + "target_x_in,target_y_in,aim_requested,fire_requested,fire_permitted,rpm_source,"
            + "ball_count,beam_mask,occupied_mask,beam_sample_age_ms,feeding,sequence_id,"
            + "sequence_active,sequence_elapsed_s,sequence_start_t_ms,sequence_start_ball_count,"
            + "sequence_start_beam_mask,sequence_end_reason,shooter_enabled,target_rpm,measured_rpm,"
            + "battery_raw_v,battery_filtered_v,boost_requested,boost_applied,applied_power,"
            + "intake_mode,jam_reverse,flower_position,turret_mode,turret_state,"
            + "turret_measured_deg,turret_target_deg,turret_robot_relative_deg,"
            + "turret_velocity_deg_s,turret_aim_ready,turret_reachable,"
            + "dropped_match_rows,dropped_detail_rows,writer_queue_rows").split(",");

    public String session = "", opmode = "", phase = "", profile = "", stage = "", plan = "";
    public double timeMs, periodMs, previousCaptureMs;
    public long epochMs, loop;
    public double x = Double.NaN, y = Double.NaN, heading = Double.NaN;
    public double vx = Double.NaN, vy = Double.NaN, omega = Double.NaN;
    public boolean busy;
    public double pathIndex = Double.NaN, pathT = Double.NaN, pathHeading = Double.NaN;
    public double targetX = Double.NaN, targetY = Double.NaN;
    public boolean aimRequested, fireRequested;
    public Boolean firePermitted;
    public String rpmSource = "UNKNOWN";
    public int ballCount, beams, occupied, sequenceId, startBallCount, startBeams;
    public double beamAgeMs = Double.NaN, sequenceElapsed, sequenceStartMs;
    public boolean feeding, sequenceActive, shooterEnabled, boostRequested, boostApplied;
    public String endReason = "", intakeMode = "", flower = "UNKNOWN";
    public double targetRpm, measuredRpm, rawV, filteredV, appliedPower;
    public boolean jamReverse;
    public String turretMode = "UNAVAILABLE", turretState = "UNAVAILABLE";
    public double turretMeasured = Double.NaN, turretTarget = Double.NaN;
    public double turretRelative = Double.NaN, turretVelocity = Double.NaN;
    public boolean turretReady, turretReachable;
    public long droppedMatch, droppedDetail;
    public int queueRows;

    public Object[] values(String event, String note) {
        return new Object[]{1, session, opmode, phase, profile, timeMs, epochMs, loop, periodMs,
                previousCaptureMs, event, note, stage, plan, x, y, heading, vx, vy, omega,
                busy, pathIndex, pathT, pathHeading, targetX, targetY, aimRequested,
                fireRequested, firePermitted, rpmSource, ballCount, beams, occupied, beamAgeMs,
                feeding, sequenceId, sequenceActive, sequenceElapsed, sequenceStartMs,
                startBallCount, startBeams, endReason, shooterEnabled, targetRpm, measuredRpm,
                rawV, filteredV, boostRequested, boostApplied, appliedPower, intakeMode,
                jamReverse, flower, turretMode, turretState, turretMeasured, turretTarget,
                turretRelative, turretVelocity, turretReady, turretReachable,
                droppedMatch, droppedDetail, queueRows};
    }
}
