package org.firstinspires.ftc.teamcode.subsystems.shot;

import com.bylazar.configurables.annotations.Configurable;

import java.util.Arrays;
import java.util.Comparator;

/** Robot-position calibration: flywheel RPM and field-coordinate aim offsets, with no zones or hood. */
@Configurable
public final class ShotCalibrationTable2026 {
    private ShotCalibrationTable2026() {}

    public static int NEAREST_SAMPLE_COUNT = 3;
    public static double EXACT_SNAP_DISTANCE_INCHES = 0.5;

    public static Sample[] SAMPLES = {
            // Add measured 2026 shots here. Example FORMAT ONLY (not calibrated):
            //            robotX robotY flywheelRPM deltaAimX deltaAimY
            // new Sample(54.0,  12.0,  3200.0,     0.5,      -1.0),
    };

    public static final class Sample {
        public double robotX, robotY, flywheelRpm, deltaAimX, deltaAimY;

        /** X/Y and offsets are field inches; positive X is right, positive Y is up-field. */
        public Sample(double robotX, double robotY, double flywheelRpm,
                      double deltaAimX, double deltaAimY) {
            this.robotX = robotX;
            this.robotY = robotY;
            this.flywheelRpm = flywheelRpm;
            this.deltaAimX = deltaAimX;
            this.deltaAimY = deltaAimY;
        }
    }

    public static final class Solution {
        public final double flywheelRpm, deltaAimX, deltaAimY, nearestSampleDistanceInches;
        public final int samplesUsed;

        private Solution(double rpm, double dx, double dy, double nearestDistance, int count) {
            flywheelRpm = rpm;
            deltaAimX = dx;
            deltaAimY = dy;
            nearestSampleDistanceInches = nearestDistance;
            samplesUsed = count;
        }
    }

    /**
     * Uses the nearest row inside the snap radius, otherwise blends the nearest
     * three rows with inverse-square distance weights. Returns null if no valid
     * rows/pose exist, allowing the caller to use regression instead.
     */
    public static Solution lookup(double robotX, double robotY) {
        Sample[] rows = SAMPLES;
        if (rows == null || rows.length == 0
                || !Double.isFinite(robotX) || !Double.isFinite(robotY)) return null;
        Entry[] entries = new Entry[rows.length];
        int count = 0;
        for (Sample row : rows) {
            if (row == null) continue;
            // Snapshot public tuning fields before validating and blending them.
            double x = row.robotX, y = row.robotY, rpm = row.flywheelRpm;
            double dx = row.deltaAimX, dy = row.deltaAimY;
            if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(rpm) || rpm <= 0
                    || !Double.isFinite(dx) || !Double.isFinite(dy)) continue;
            double distance = Math.hypot(x - robotX, y - robotY);
            if (Double.isFinite(distance)) entries[count++] = new Entry(distance, rpm, dx, dy);
        }
        if (count == 0) return null;
        Arrays.sort(entries, 0, count, Comparator.comparingDouble(e -> e.distance));
        Entry nearest = entries[0];
        double snap = Double.isFinite(EXACT_SNAP_DISTANCE_INCHES)
                ? Math.max(0, EXACT_SNAP_DISTANCE_INCHES) : 0.5;
        if (nearest.distance <= snap) {
            return new Solution(nearest.rpm, nearest.dx, nearest.dy, nearest.distance, 1);
        }
        int used = Math.max(1, Math.min(NEAREST_SAMPLE_COUNT, count));
        double weightSum = 0;
        double[] weights = new double[used];
        for (int i = 0; i < used; i++) {
            // Scaling by nearest distance keeps weights <= 1, avoiding 1/d^2 overflow.
            double ratio = nearest.distance / entries[i].distance;
            weights[i] = ratio * ratio;
            weightSum += weights[i];
        }
        double rpm = 0, dx = 0, dy = 0;
        for (int i = 0; i < used; i++) {
            double weight = weights[i] / weightSum;
            rpm += weight * entries[i].rpm;
            dx += weight * entries[i].dx;
            dy += weight * entries[i].dy;
        }
        return new Solution(rpm, dx, dy, nearest.distance, used);
    }

    private static final class Entry {
        final double distance, rpm, dx, dy;
        Entry(double distance, double rpm, double dx, double dy) {
            this.distance = distance;
            this.rpm = rpm;
            this.dx = dx;
            this.dy = dy;
        }
    }
}
