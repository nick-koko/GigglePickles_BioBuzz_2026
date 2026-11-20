package org.firstinspires.ftc.teamcode.logging;

import java.util.Objects;

/** Hardware-free event detection. A beam edge is an observation, never a claimed score. */
public final class MatchLogRecorder2026 {
    private final AsyncCsvWriter2026.Sink sink;
    private RobotLogFrame2026 previous;
    private double lastSampleMs = Double.NEGATIVE_INFINITY;
    private boolean enabledPreviously;

    public MatchLogRecorder2026(AsyncCsvWriter2026.Sink sink) { this.sink = sink; }

    public void update(RobotLogFrame2026 frame, boolean enabled, double intervalMs) {
        if (enabled && !enabledPreviously) event(frame, "LOG_RESUME", "Basic stream enabled");
        if (enabled && previous == null && frame.sequenceId > 0)
            event(frame, "SHOT_START", "First observed feed sequence; not a verified launch");
        if (enabled && previous != null) {
            if (frame.sequenceId != previous.sequenceId)
                event(frame, "SHOT_START", "Feed sequence commanded; not a verified launch");
            if (previous.sequenceActive && (!frame.sequenceActive || frame.sequenceId != previous.sequenceId))
                event(frame, "SHOT_END", frame.endReason);
            int changed = frame.beams ^ previous.beams;
            if (changed != 0) event(frame, "BEAM_EDGE", "changed_mask=" + changed);
            if (frame.ballCount != previous.ballCount)
                event(frame, "BALL_COUNT", "Tracked occupancy; cleared when feeding begins");
            if (frame.ballCountRecoveryEvents != previous.ballCountRecoveryEvents)
                event(frame, "BALL_COUNT_RECOVERY", "Sustained blocked-beam stack reconciled; not a verified physical count");
            if (frame.intakeCapacityOverride != previous.intakeCapacityOverride)
                event(frame, "INTAKE_OVERRIDE", frame.intakeCapacityOverride ? "HELD" : "RELEASED_OR_STOPPED");
            if (!frame.stage.equals(previous.stage) || !frame.plan.equals(previous.plan))
                event(frame, "STAGE", frame.stage);
            if (frame.fireRequested != previous.fireRequested
                    || !Objects.equals(frame.firePermitted, previous.firePermitted))
                event(frame, "FIRE_REQUEST", "Request/interlock state; not a verified launch");
            if (!frame.turretState.equals(previous.turretState))
                event(frame, "TURRET_STATE", frame.turretState);
            if (!frame.shooterPowerProfile.equals(previous.shooterPowerProfile)
                    || !frame.shooterStartupExitReason.equals(previous.shooterStartupExitReason))
                event(frame, "SHOOTER_PROFILE", frame.shooterStartupExitReason);
            if (frame.feeding != previous.feeding || frame.boostRequested != previous.boostRequested
                    || frame.shooterEnabled != previous.shooterEnabled
                    || !frame.intakeMode.equals(previous.intakeMode) || frame.jamReverse != previous.jamReverse
                    || !frame.flower.equals(previous.flower)) event(frame, "OUTPUT_STATE", "");
        }
        if (enabled && frame.timeMs - lastSampleMs >= Math.max(0, intervalMs)) {
            event(frame, "SAMPLE", "");
            lastSampleMs = frame.timeMs;
        }
        enabledPreviously = enabled;
        previous = frame;
    }

    public void event(RobotLogFrame2026 frame, String event, String note) {
        sink.write(false, frame.values(event, note));
    }
}
