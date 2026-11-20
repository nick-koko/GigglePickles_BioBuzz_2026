package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.util.ShooterPowerProfile2026;

/**
 * Shared timed shooting sequence used by TeleOp and shooter tuning.
 *
 * <p>This class coordinates feeding and the shooter's generic power-override
 * primitive. It does not own normal intake mode or flywheel start/stop.</p>
 */
@Configurable
public final class ShotSequenceController2026 {

    public static double TRIGGER_THRESHOLD = 0.10;
    public static double FEED_SECONDS = 1.2;
    public static boolean ENABLE_TIMED_BOOST = true;
    public static double BOOST_DELAY_SECONDS = 0.18;
    public static double BOOST_DURATION_SECONDS = 0.50;
    public static double BOOST_MINIMUM_POWER = 0.75;
    public static boolean BOOST_BYPASSES_SLEW = true;
    // Opt-in extensions. With these defaults BOOST_MINIMUM_POWER is still the
    // same flat duty-cycle floor, with the same existing delay and duration.
    public static boolean ENABLE_RAMPED_BOOST = false;
    public static double BOOST_START_POWER = 0.75;
    public static double BOOST_END_POWER = 0.75;
    public static boolean ENABLE_BOOST_VOLTAGE_SCALING = false;
    public static double BOOST_NOMINAL_V = 12.5;
    public static double BOOST_OVERSPEED_TAPER_RPM = 0.0;

    private boolean previousShootRequest;
    private boolean requireTriggerRelease;
    private boolean active;
    private boolean boostActive;
    private boolean startedThisUpdate;
    private boolean completedThisUpdate;
    private long sequenceStartNanos;
    private int sequenceCount;
    private double elapsedSeconds;
    private int startBallCount, startBeamMask;
    private String endReason = "";

    // Snapshot configuration at sequence start so a live Panels edit cannot
    // change the timing halfway through a shot.
    private double activeFeedSeconds;
    private boolean activeBoostEnabled;
    private double activeBoostDelaySeconds;
    private double activeBoostDurationSeconds;
    private double activeBoostMinimumPower;
    private boolean activeBoostBypassesSlew;
    private boolean activeRampedBoost, activeBoostVoltageScaling;
    private double activeBoostStartPower, activeBoostEndPower, activeBoostNominalV;
    private double activeBoostTaperRpm, requestedBoostFloor;

    public void reset() {
        validateConfiguration();
        previousShootRequest = false;
        requireTriggerRelease = false;
        active = false;
        boostActive = false;
        startedThisUpdate = false;
        completedThisUpdate = false;
        sequenceStartNanos = 0L;
        sequenceCount = 0;
        elapsedSeconds = 0.0;
        startBallCount = startBeamMask = 0;
        endReason = "";
        stopSequenceOutputs();
    }

    /**
     * Updates the sequence from a level-triggered shoot request. A new sequence
     * starts only on a rising edge, so holding RT cannot repeatedly fire.
     */
    public void update(boolean shootRequested) {
        startedThisUpdate = false;
        completedThisUpdate = false;

        if (!shootRequested) {
            requireTriggerRelease = false;
        }

        boolean risingEdge = shootRequested && !previousShootRequest;
        previousShootRequest = shootRequested;

        if (!active && risingEdge && !requireTriggerRelease) {
            startSequence();
        }

        updateActiveSequence(shootRequested);
    }

    /**
     * Starts one sequence without a gamepad edge. Intended for autonomous.
     */
    public void start() {
        startedThisUpdate = false;
        completedThisUpdate = false;
        if (!active) {
            startSequence();
        }
    }

    /**
     * Advances an autonomous-started sequence without gamepad input.
     */
    public void update() {
        startedThisUpdate = false;
        completedThisUpdate = false;
        updateActiveSequence(false);
    }

    private void updateActiveSequence(boolean shootRequestStillHeld) {
        if (!active) {
            stopSequenceOutputs();
            return;
        }

        elapsedSeconds = (System.nanoTime() - sequenceStartNanos) / 1e9;
        if (elapsedSeconds >= activeFeedSeconds) {
            elapsedSeconds = activeFeedSeconds;
            active = false;
            endReason = "FEED_TIMER_COMPLETE";
            completedThisUpdate = true;
            requireTriggerRelease = shootRequestStillHeld;
            stopSequenceOutputs();
            return;
        }

        IntakeWithSensorsSubsystem2026.INSTANCE.setFeedingToShooter(true);
        ShooterSubsystem2026.INSTANCE.setShotFeeding(true);

        double boostEndSeconds =
                activeBoostDelaySeconds + activeBoostDurationSeconds;
        boostActive =
                activeBoostEnabled
                        && elapsedSeconds >= activeBoostDelaySeconds
                        && elapsedSeconds < boostEndSeconds;
        if (boostActive) {
            requestedBoostFloor = activeRampedBoost
                    ? ShooterPowerProfile2026.rampedFloor(activeBoostStartPower, activeBoostEndPower,
                            elapsedSeconds-activeBoostDelaySeconds, activeBoostDurationSeconds)
                    : activeBoostMinimumPower;
            ShooterSubsystem2026.INSTANCE.setMinimumPowerOverride(
                    requestedBoostFloor, activeBoostBypassesSlew,
                    activeBoostVoltageScaling, activeBoostNominalV, activeBoostTaperRpm
            );
        } else {
            requestedBoostFloor = 0.0;
            ShooterSubsystem2026.INSTANCE.clearMinimumPowerOverride();
        }
    }

    /**
     * Immediately closes the feed gate and clears boost. Flywheel state is not
     * changed. RT must be released before another sequence can begin.
     */
    public void cancel() {
        if (active) endReason = "CANCELLED";
        active = false;
        boostActive = false;
        requireTriggerRelease = true;
        stopSequenceOutputs();
    }

    private void startSequence() {
        validateConfiguration();
        startBallCount = IntakeWithSensorsSubsystem2026.INSTANCE.getBallCount();
        startBeamMask = IntakeWithSensorsSubsystem2026.INSTANCE.getCachedBeamMask();
        endReason = "";
        activeFeedSeconds = Math.max(0.0, FEED_SECONDS);
        activeBoostEnabled = ENABLE_TIMED_BOOST;
        activeBoostDelaySeconds = Math.max(0.0, BOOST_DELAY_SECONDS);
        activeBoostDurationSeconds = Math.max(0.0, BOOST_DURATION_SECONDS);
        activeBoostMinimumPower =
                Range.clip(BOOST_MINIMUM_POWER, 0.0, 1.0);
        activeBoostBypassesSlew = BOOST_BYPASSES_SLEW;
        activeRampedBoost = ENABLE_RAMPED_BOOST;
        activeBoostStartPower = activeRampedBoost ? BOOST_START_POWER : activeBoostMinimumPower;
        activeBoostEndPower = activeRampedBoost ? BOOST_END_POWER : activeBoostMinimumPower;
        activeBoostVoltageScaling = ENABLE_BOOST_VOLTAGE_SCALING;
        activeBoostNominalV = BOOST_NOMINAL_V;
        activeBoostTaperRpm = BOOST_OVERSPEED_TAPER_RPM;

        sequenceStartNanos = System.nanoTime();
        elapsedSeconds = 0.0;
        active = activeFeedSeconds > 0.0;
        boostActive = false;
        startedThisUpdate = active;
        if (active) {
            sequenceCount++;
            ShooterSubsystem2026.INSTANCE.setShotFeeding(true);
        } else {
            completedThisUpdate = true;
            requireTriggerRelease = previousShootRequest;
            stopSequenceOutputs();
        }
    }

    private void stopSequenceOutputs() {
        requestedBoostFloor = 0.0;
        ShooterSubsystem2026.INSTANCE.setShotFeeding(false);
        IntakeWithSensorsSubsystem2026.INSTANCE.setFeedingToShooter(false);
        ShooterSubsystem2026.INSTANCE.clearMinimumPowerOverride();
        boostActive = false;
    }

    public boolean isTriggerPressed(double gamepad1Trigger, double gamepad2Trigger) {
        double threshold = Range.clip(TRIGGER_THRESHOLD, 0.0, 1.0);
        return gamepad1Trigger > threshold || gamepad2Trigger > threshold;
    }

    public boolean isActive() {
        return active;
    }

    public boolean isBoostActive() {
        return boostActive;
    }

    public boolean startedThisUpdate() {
        return startedThisUpdate;
    }

    public boolean completedThisUpdate() {
        return completedThisUpdate;
    }

    public int getSequenceCount() {
        return sequenceCount;
    }

    public int getStartBallCount() { return startBallCount; }
    public int getStartBeamMask() { return startBeamMask; }
    public long getSequenceStartNanos() { return sequenceStartNanos; }
    public String getEndReason() { return endReason; }

    public double getElapsedSeconds() {
        return elapsedSeconds;
    }

    public double getActiveFeedSeconds() {
        return activeFeedSeconds;
    }

    public boolean isActiveBoostEnabled() {
        return activeBoostEnabled;
    }

    public double getActiveBoostDelaySeconds() {
        return activeBoostDelaySeconds;
    }

    public double getActiveBoostDurationSeconds() {
        return activeBoostDurationSeconds;
    }

    public double getActiveBoostMinimumPower() {
        return activeBoostMinimumPower;
    }

    public boolean isActiveBoostBypassingSlew() {
        return activeBoostBypassesSlew;
    }

    public boolean isActiveRampedBoost() { return activeRampedBoost; }
    public double getActiveBoostStartPower() { return activeBoostStartPower; }
    public double getActiveBoostEndPower() { return activeBoostEndPower; }
    public boolean isActiveBoostVoltageScaling() { return activeBoostVoltageScaling; }
    public double getActiveBoostNominalV() { return activeBoostNominalV; }
    public double getActiveBoostTaperRpm() { return activeBoostTaperRpm; }
    /** Pending OpMode request; the shooter may not consume it until the next controller update. */
    public double getRequestedBoostFloor() { return requestedBoostFloor; }

    public static void validateConfiguration() {
        double[] times = {FEED_SECONDS, BOOST_DELAY_SECONDS, BOOST_DURATION_SECONDS, BOOST_OVERSPEED_TAPER_RPM};
        for (double value : times) if (!Double.isFinite(value) || value < 0)
            throw new IllegalArgumentException("Shot feed/boost timing and overspeed taper must be finite and nonnegative");
        double[] powers = {BOOST_MINIMUM_POWER, BOOST_START_POWER, BOOST_END_POWER};
        for (double value : powers) if (!Double.isFinite(value) || value < 0 || value > 1)
            throw new IllegalArgumentException("Shot boost powers must be finite, between zero and one");
        if (!Double.isFinite(BOOST_NOMINAL_V) || BOOST_NOMINAL_V <= 0)
            throw new IllegalArgumentException("BOOST_NOMINAL_V must be finite and positive");
    }
}
