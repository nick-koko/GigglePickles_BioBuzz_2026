package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Pickles2026Teleop;

/** Chooses autonomous RPM without owning paths, feed timing, boost, or shooter stop commands. */
public final class AutonomousShooterRpm2026 {
    public static final double DEFAULT_LOW_RPM = 3100.0;
    public static final double DEFAULT_HIGH_RPM = 3150.0;

    private double fixedRpm;
    private boolean started;
    private boolean previousUseRegression;
    private boolean firstVolleyOverride;
    private boolean volleyHeld, applyAfterVolley;
    private double heldRpm;

    /** Logical reset only: Init must not start the flywheel. */
    public void reset(double firstFixedRpm) {
        reset(firstFixedRpm, false);
    }

    public void reset(double firstFixedRpm, boolean firstVolleyOverride) {
        fixedRpm = firstFixedRpm;
        started = false;
        previousUseRegression = false;
        this.firstVolleyOverride = firstVolleyOverride;
        volleyHeld = applyAfterVolley = false;
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
        if (volleyHeld) {
            applyAfterVolley = true;
        } else {
            ShooterSubsystem2026.INSTANCE.spinUp(selectedRpm(useRegression, pose));
        }
    }

    public void holdVolleyRpm() {
        heldRpm = ShooterSubsystem2026.INSTANCE.getTargetFlywheelRpm();
        volleyHeld = true;
    }

    public void releaseVolleyRpm() { volleyHeld = false; }

    /** Called only after the first feed sequence finishes; subsequent updates may use regression. */
    public void finishFirstVolley() {
        firstVolleyOverride = false;
    }

    public void update(boolean useRegression, Pose pose) {
        // A parking/Stop command must remain stopped. Never re-enable the shooter here.
        if (!started || volleyHeld || !ShooterSubsystem2026.INSTANCE.isEnabled()) return;
        // Fixed mode leaves the original one-shot commands alone. Switching back
        // from regression restores the fixed RPM most recently requested by the sequence.
        if (useRegression || previousUseRegression || applyAfterVolley) {
            ShooterSubsystem2026.INSTANCE.spinUp(selectedRpm(useRegression, pose));
        }
        applyAfterVolley = false;
        previousUseRegression = useRegression;
    }

    public double selectedRpm(boolean useRegression, Pose pose) {
        if (volleyHeld) return heldRpm;
        double regression = regressionRpm(pose);
        return useRegression && !firstVolleyOverride && Double.isFinite(regression) && regression > 0
                ? regression : fixedRpm;
    }

    private static double regressionRpm(Pose pose) {
        return pose == null ? Double.NaN
                : Pickles2026Teleop.regressionFlywheelRpm(pose.getX(), pose.getY());
    }

    public String getMode(boolean useRegression, Pose pose) {
        if (volleyHeld) return "VOLLEY RPM HOLD";
        double regression = regressionRpm(pose);
        boolean valid = Double.isFinite(regression) && regression > 0;
        return !useRegression ? "FIXED"
                : firstVolleyOverride ? "FIRST VOLLEY LOW OVERRIDE"
                : valid ? "TELEOP REGRESSION" : "FIXED FALLBACK (invalid pose/equation)";
    }

    public void addTelemetry(Telemetry telemetry, boolean useRegression, Pose pose) {
        double regression = regressionRpm(pose);
        telemetry.addData("Auton RPM mode", getMode(useRegression, pose));
        telemetry.addData("Auton fixed / regression RPM", "%.0f / %.0f", fixedRpm, regression);
        telemetry.addData("Auton selected RPM", "%.0f", selectedRpm(useRegression, pose));
        telemetry.addData("Regression distance", "%.2f in", pose == null ? Double.NaN
                : Pickles2026Teleop.regressionDistanceInches(pose.getX(), pose.getY()));
    }
}
