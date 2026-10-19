package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

/**
 * Shared timed shooting sequence used by TeleOp and shooter tuning.
 *
 * <p>This class coordinates feeding and the shooter's generic power-override
 * primitive. It does not own normal intake mode or flywheel start/stop.</p>
 */
@Configurable
public final class ShotSequenceController2026 {

    public static double TRIGGER_THRESHOLD = 0.10;
    public static double FEED_SECONDS = 2.0;
    public static boolean ENABLE_TIMED_BOOST = true;
    public static double BOOST_DELAY_SECONDS = 0.18;
    public static double BOOST_DURATION_SECONDS = 0.50;
    public static double BOOST_MINIMUM_POWER = 0.75;
    public static boolean BOOST_BYPASSES_SLEW = true;

    private boolean previousShootRequest;
    private boolean requireTriggerRelease;
    private boolean active;
    private boolean boostActive;
    private boolean startedThisUpdate;
    private boolean completedThisUpdate;
    private long sequenceStartNanos;
    private int sequenceCount;
    private double elapsedSeconds;

    // Snapshot configuration at sequence start so a live Panels edit cannot
    // change the timing halfway through a shot.
    private double activeFeedSeconds;
    private boolean activeBoostEnabled;
    private double activeBoostDelaySeconds;
    private double activeBoostDurationSeconds;
    private double activeBoostMinimumPower;
    private boolean activeBoostBypassesSlew;

    public void reset() {
        previousShootRequest = false;
        requireTriggerRelease = false;
        active = false;
        boostActive = false;
        startedThisUpdate = false;
        completedThisUpdate = false;
        sequenceStartNanos = 0L;
        sequenceCount = 0;
        elapsedSeconds = 0.0;
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

        if (!active) {
            stopSequenceOutputs();
            return;
        }

        elapsedSeconds = (System.nanoTime() - sequenceStartNanos) / 1e9;
        if (elapsedSeconds >= activeFeedSeconds) {
            elapsedSeconds = activeFeedSeconds;
            active = false;
            completedThisUpdate = true;
            requireTriggerRelease = shootRequested;
            stopSequenceOutputs();
            return;
        }

        IntakeWithSensorsSubsystem2026.INSTANCE.setFeedingToShooter(true);

        double boostEndSeconds =
                activeBoostDelaySeconds + activeBoostDurationSeconds;
        boostActive =
                activeBoostEnabled
                        && elapsedSeconds >= activeBoostDelaySeconds
                        && elapsedSeconds < boostEndSeconds;
        if (boostActive) {
            ShooterSubsystem2026.INSTANCE.setMinimumPowerOverride(
                    activeBoostMinimumPower,
                    activeBoostBypassesSlew
            );
        } else {
            ShooterSubsystem2026.INSTANCE.clearMinimumPowerOverride();
        }
    }

    /**
     * Immediately closes the feed gate and clears boost. Flywheel state is not
     * changed. RT must be released before another sequence can begin.
     */
    public void cancel() {
        active = false;
        boostActive = false;
        requireTriggerRelease = true;
        stopSequenceOutputs();
    }

    private void startSequence() {
        activeFeedSeconds = Math.max(0.0, FEED_SECONDS);
        activeBoostEnabled = ENABLE_TIMED_BOOST;
        activeBoostDelaySeconds = Math.max(0.0, BOOST_DELAY_SECONDS);
        activeBoostDurationSeconds = Math.max(0.0, BOOST_DURATION_SECONDS);
        activeBoostMinimumPower =
                Range.clip(BOOST_MINIMUM_POWER, 0.0, 1.0);
        activeBoostBypassesSlew = BOOST_BYPASSES_SLEW;

        sequenceStartNanos = System.nanoTime();
        elapsedSeconds = 0.0;
        active = activeFeedSeconds > 0.0;
        boostActive = false;
        startedThisUpdate = active;
        if (active) {
            sequenceCount++;
        } else {
            completedThisUpdate = true;
            requireTriggerRelease = previousShootRequest;
            stopSequenceOutputs();
        }
    }

    private void stopSequenceOutputs() {
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
}
