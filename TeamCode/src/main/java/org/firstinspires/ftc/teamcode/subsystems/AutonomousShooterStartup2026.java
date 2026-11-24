package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

/** First-volley readiness using fresh controller samples, with a bounded match-time wait. */
@Configurable
public final class AutonomousShooterStartup2026 {
    public static double READY_TOLERANCE_RPM = 75.0;
    public static double READY_STABLE_SECONDS = 0.15;
    public static double MAX_WAIT_SECONDS = 2.0;
    public static double MAX_SAMPLE_AGE_SECONDS = 0.10;

    private long started, stableSince, previousSample;
    private double target, minimumWait, tolerance, stableSeconds, maximumWait, maximumSampleAge;
    private boolean armed, ready, timedOut, completed;

    public void reset() {
        armed = ready = timedOut = completed = false;
        stableSince = previousSample = 0;
    }

    public static void validateConfiguration(double minimumWait) {
        if (!Double.isFinite(minimumWait) || minimumWait < 0
                || !Double.isFinite(READY_TOLERANCE_RPM) || READY_TOLERANCE_RPM <= 0
                || !Double.isFinite(READY_STABLE_SECONDS) || READY_STABLE_SECONDS <= 0
                || !Double.isFinite(MAX_WAIT_SECONDS) || MAX_WAIT_SECONDS < minimumWait
                || MAX_WAIT_SECONDS < READY_STABLE_SECONDS
                || !Double.isFinite(MAX_SAMPLE_AGE_SECONDS) || MAX_SAMPLE_AGE_SECONDS <= 0) {
            throw new IllegalArgumentException("Invalid autonomous shooter readiness settings");
        }
    }

    public void start(long now, double minimumWait) {
        validateConfiguration(minimumWait);
        started = now;
        this.minimumWait = minimumWait;
        tolerance = READY_TOLERANCE_RPM;
        stableSeconds = READY_STABLE_SECONDS;
        maximumWait = MAX_WAIT_SECONDS;
        maximumSampleAge = MAX_SAMPLE_AGE_SECONDS;
        stableSince = previousSample = 0;
        target = Double.NaN;
        ready = timedOut = completed = false;
        armed = true;
    }

    public void update(long now, long sample, double measured, double requested, boolean enabled) {
        if (!armed || completed) return;
        boolean fresh = sample >= started && sample <= now
                && seconds(now - sample) <= maximumSampleAge;
        boolean inBand = fresh && enabled && Double.isFinite(measured)
                && Double.isFinite(requested) && requested > 0
                && Math.abs(measured - requested) <= tolerance;
        if (!inBand || requested != target
                || (previousSample != 0 && (sample < previousSample
                        || seconds(sample - previousSample) > maximumSampleAge))) {
            stableSince = 0;
        }
        if (inBand && sample != previousSample) {
            if (stableSince == 0) stableSince = sample;
        }
        target = requested;
        previousSample = sample;
        ready = inBand && stableSince != 0 && seconds(sample - stableSince) >= stableSeconds
                && seconds(now - started) >= minimumWait;
        timedOut = seconds(now - started) >= maximumWait;
    }

    public boolean isFinished() { return armed && (ready || timedOut); }
    public void complete() { completed = true; }
    public String getResult() { return ready ? "READY" : timedOut ? "TIMEOUT_FEED_ANYWAY" : "WAITING"; }
    public double getElapsedSeconds(long now) { return armed ? seconds(now - started) : 0; }
    private static double seconds(long nanos) { return nanos / 1e9; }
}

