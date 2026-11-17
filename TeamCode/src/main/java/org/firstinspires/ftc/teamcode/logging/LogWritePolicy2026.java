package org.firstinspires.ftc.teamcode.logging;

/** Hardware-free checkpoint timing. Detailed capture alone never authorizes an early write. */
public final class LogWritePolicy2026 {
    private boolean previousActive;
    private double pendingPostShot = Double.POSITIVE_INFINITY;
    private double lastPracticeWrite = Double.NEGATIVE_INFINITY;

    public String update(double seconds, boolean active, double memoryOnlySeconds,
                         boolean longPracticeWrites, double practiceIntervalSeconds,
                         boolean practiceMode, boolean postShotWrites, boolean detailEnabled,
                         double postShotDelaySeconds) {
        if (active) pendingPostShot = Double.POSITIVE_INFINITY;
        else if (previousActive) pendingPostShot = seconds + nonnegative(postShotDelaySeconds, 1);
        previousActive = active;

        // Strictly beyond the normal match duration. Never flush because memory is full.
        if (longPracticeWrites && seconds > nonnegative(memoryOnlySeconds, 125)
                && seconds - lastPracticeWrite >= Math.max(0.1, nonnegative(practiceIntervalSeconds, 10))) {
            lastPracticeWrite = seconds;
            pendingPostShot = Double.POSITIVE_INFINITY;
            return "LONG_PRACTICE";
        }
        // Two explicit practice switches are required to bypass the normal match window.
        if (practiceMode && postShotWrites && detailEnabled && !active && seconds >= pendingPostShot) {
            pendingPostShot = Double.POSITIVE_INFINITY;
            lastPracticeWrite = seconds;
            return "POST_SHOT_PRACTICE";
        }
        return "";
    }

    private static double nonnegative(double value, double fallback) {
        return Double.isFinite(value) ? Math.max(0, value) : fallback;
    }
}
