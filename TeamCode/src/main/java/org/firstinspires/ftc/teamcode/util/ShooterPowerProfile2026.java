package org.firstinspires.ftc.teamcode.util;

/** Hardware-free startup latch and boost shaping. Never decides whether a shot may feed. */
public final class ShooterPowerProfile2026 {
    public enum State { OFF, STARTUP, RUNNING }
    private State state = State.OFF;
    private String exitReason = "";
    private long startNanos, stableSinceNanos;
    private double elapsedSeconds, previousRpm, measuredSlope;

    public void reset() {
        state = State.OFF;
        exitReason = "";
        startNanos = stableSinceNanos = 0;
        elapsedSeconds = previousRpm = measuredSlope = 0;
    }

    public void begin(long now, double rpm, boolean enabled, double entryMaxRpm, boolean feeding) {
        startNanos = now;
        stableSinceNanos = 0;
        elapsedSeconds = measuredSlope = 0;
        previousRpm = rpm;
        state = State.STARTUP;
        exitReason = "";
        if (!enabled) finish("PROFILE_DISABLED");
        else if (feeding) finish("FEED_STARTED");
        else if (rpm >= entryMaxRpm) finish("ALREADY_SPINNING");
    }

    public void update(long now, double rpm, double target, double dt, boolean enabled,
                       boolean feeding, double timeout, double tolerance, double maxSlope,
                       double stableSeconds) {
        measuredSlope = (rpm - previousRpm) / dt;
        previousRpm = rpm;
        if (state != State.STARTUP) return;
        elapsedSeconds = Math.max(0, (now - startNanos) / 1e9);
        if (!enabled) finish("PROFILE_DISABLED");
        else if (feeding) finish("FEED_STARTED");
        else if (elapsedSeconds >= timeout) finish("TIMEOUT");
        else {
            boolean stable = Math.abs(target-rpm) <= tolerance && Math.abs(measuredSlope) <= maxSlope;
            if (!stable) stableSinceNanos = 0;
            else {
                if (stableSinceNanos == 0) stableSinceNanos = now;
                if ((now-stableSinceNanos)/1e9 >= stableSeconds) finish("SETTLED");
            }
        }
    }

    private void finish(String reason) { state = State.RUNNING; exitReason = reason; }
    public State getState() { return state; }
    public String getExitReason() { return exitReason; }
    /** Frozen at handoff; elapsed time is not a permission gate for feeding. */
    public double getElapsedSeconds() { return elapsedSeconds; }
    public double getMeasuredSlope() { return measuredSlope; }

    public static double boostVoltageGain(boolean enabled, double voltage, double minValidVoltage,
                                           double nominalVoltage) {
        // Runtime recovery for a voltage sensor that was present at Init but gives a bad sample.
        return enabled && Double.isFinite(voltage) && voltage >= minValidVoltage
                ? nominalVoltage / voltage : 1.0;
    }

    public static double overspeedMultiplier(double rpm, double target, double taperRpm) {
        return taperRpm > 0 ? clip((target+taperRpm-rpm)/taperRpm) : 1.0;
    }

    public static double rampedFloor(double start, double end, double elapsed, double duration) {
        return start + clip(elapsed/duration)*(end-start);
    }

    public static double clip(double value) { return Math.max(0, Math.min(1, value)); }
}
