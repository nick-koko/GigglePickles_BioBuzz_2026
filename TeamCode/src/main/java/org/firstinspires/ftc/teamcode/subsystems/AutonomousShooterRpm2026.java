package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Pickles2026Teleop;

/** Chooses autonomous RPM without owning paths, feed timing, boost, or shooter stop commands. */
public final class AutonomousShooterRpm2026 {
    public static final double DEFAULT_LOW_RPM = 3050.0;
    public static final double DEFAULT_HIGH_RPM = 3150.0;

    private double fixedRpm;
    private boolean started;
    private boolean previousUseRegression;
    private boolean firstVolleyOverride;

    /** Logical reset only: Init must not start the flywheel. */
    public void reset(double firstFixedRpm) {
        reset(firstFixedRpm, false);
    }

    public void reset(double firstFixedRpm, boolean firstVolleyOverride) {
        fixedRpm = firstFixedRpm;
        started = false;
        previousUseRegression = false;
        this.firstVolleyOverride = firstVolleyOverride;
    }

    /** Replaces an existing fixed-RPM start/change command, retaining its fallback value. */
    public void start(double fixedRpm, boolean useRegression, Pose pose) {
        start(fixedRpm, useRegression, pose, false);
    }

    /** Captures the close preload override for the entire first volley, including spin-up. */
    public void start(double fixedRpm, boolean useRegression, Pose pose, boolean firstVolleyOverride) {
        this.fixedRpm = fixedRpm;
        started = true;
        previousUseRegression = useRegression;
        this.firstVolleyOverride = firstVolleyOverride;
        ShooterSubsystem2026.INSTANCE.spinUp(selectedRpm(useRegression, pose));
    }

    /** Called only after the first feed sequence finishes; subsequent updates may use regression. */
    public void finishFirstVolley() {
        firstVolleyOverride = false;
    }

    public void update(boolean useRegression, Pose pose) {
        // A parking/Stop command must remain stopped. Never re-enable the shooter here.
        if (!started || !ShooterSubsystem2026.INSTANCE.isEnabled()) return;
        // Fixed mode leaves the original one-shot commands alone. Switching back
        // from regression restores the fixed RPM most recently requested by the sequence.
        if (useRegression || previousUseRegression) {
            ShooterSubsystem2026.INSTANCE.spinUp(selectedRpm(useRegression, pose));
        }
        previousUseRegression = useRegression;
    }

    public double selectedRpm(boolean useRegression, Pose pose) {
        double regression = regressionRpm(pose);
        return useRegression && !firstVolleyOverride && Double.isFinite(regression) && regression > 0
                ? regression : fixedRpm;
    }

    private static double regressionRpm(Pose pose) {
        return pose == null ? Double.NaN
                : Pickles2026Teleop.regressionFlywheelRpm(pose.getX(), pose.getY());
    }

    public void addTelemetry(Telemetry telemetry, boolean useRegression, Pose pose) {
        double regression = regressionRpm(pose);
        boolean valid = Double.isFinite(regression) && regression > 0;
        telemetry.addData("Auton RPM mode", !useRegression ? "FIXED"
                : firstVolleyOverride ? "FIRST VOLLEY LOW OVERRIDE"
                : valid ? "TELEOP REGRESSION" : "FIXED FALLBACK (invalid pose/equation)");
        telemetry.addData("Auton fixed / regression RPM", "%.0f / %.0f", fixedRpm, regression);
        telemetry.addData("Auton selected RPM", "%.0f", selectedRpm(useRegression, pose));
        telemetry.addData("Regression distance", "%.2f in", pose == null ? Double.NaN
                : Pickles2026Teleop.regressionDistanceInches(pose.getX(), pose.getY()));
    }
}
