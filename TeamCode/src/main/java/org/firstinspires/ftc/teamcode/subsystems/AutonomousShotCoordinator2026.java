package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.logging.RobotLogging2026;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;

/** Releases path sequencing independently of the shot controller's feed duration. */
public final class AutonomousShotCoordinator2026 {
    private final ShotSequenceController2026 shots;
    private final AutonomousShooterRpm2026 rpm;
    private final RobotLogging2026 logging;
    private Runnable completion;

    public AutonomousShotCoordinator2026(ShotSequenceController2026 shots,
                                        AutonomousShooterRpm2026 rpm, RobotLogging2026 logging) {
        this.shots = shots;
        this.rpm = rpm;
        this.logging = logging;
    }

    public Command shoot(double departureDelayFromFeedStartSeconds) {
        return shoot(departureDelayFromFeedStartSeconds, () -> {});
    }

    public Command shoot(double departureDelayFromFeedStartSeconds, Runnable afterFeed) {
        if (!Double.isFinite(departureDelayFromFeedStartSeconds) || departureDelayFromFeedStartSeconds < 0) {
            throw new IllegalArgumentException("Shot departure delay must be finite and nonnegative");
        }
        return new SequentialGroup(
                new WaitUntil(() -> completion == null && !shots.isActive()),
                new InstantCommand(() -> {
                    rpm.holdVolleyRpm();
                    completion = afterFeed;
                    shots.start();
                    logging.event("SHOT_DEPARTURE_DELAY",
                            "sequence=" + shots.getSequenceCount()
                                    + " delay_s=" + departureDelayFromFeedStartSeconds);
                }),
                new WaitUntil(() -> (System.nanoTime() - shots.getSequenceStartNanos()) / 1e9
                        >= departureDelayFromFeedStartSeconds),
                new InstantCommand(() -> logging.event("SHOT_DEPARTURE_RELEASE",
                        "sequence=" + shots.getSequenceCount()
                                + " elapsed_s=" + (System.nanoTime() - shots.getSequenceStartNanos()) / 1e9))
        );
    }

    /** Called after shots.update() each loop, even after the path has departed. */
    public void update() {
        if (completion == null || shots.isActive()) return;
        Runnable finished = completion;
        completion = null;
        rpm.releaseVolleyRpm();
        if (shots.getEndReason().equals("FEED_TIMER_COMPLETE") || shots.getActiveFeedSeconds() == 0) finished.run();
    }

    public void cancel() {
        completion = null;
        shots.cancel();
        rpm.releaseVolleyRpm();
    }
}

