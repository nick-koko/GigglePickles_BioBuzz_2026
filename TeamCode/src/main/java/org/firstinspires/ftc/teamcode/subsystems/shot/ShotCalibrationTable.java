package org.firstinspires.ftc.teamcode.subsystems.shot;

import com.bylazar.configurables.annotations.Configurable;

import java.util.Arrays;
import java.util.Comparator;

/**
 * 2D shot calibration table. Indexed by robot (x, y) in field inches, returns
 * RPM, hood position, and turret aim point via k-nearest inverse-distance
 * weighting.
 *
 * Seeded with 13 pinned waypoints from on-robot calibration. These rows are
 * intended to represent real, reachable robot-center poses, not theoretical
 * points on a legal boundary line. Per-point aim stays in the table so left,
 * center, and right shots can intentionally bias toward different parts of the
 * goal.
 *
 * The hood values below reflect the currently measured mechanical ceiling of
 * 0.525. If the hardware limit changes, update both these rows and
 * ShooterSubsystem.HOOD_MAX_POS together.
 *
 * Auton does not use this table. Auton opmodes pin their own RPM and hood
 * values per shot position.
 */
@Configurable
public final class ShotCalibrationTable {

    // Lookup tuning. Exposed as public static so FTC Dashboard can adjust them
    // live during a calibration session without recompiling.
    public static int CAL_IDW_K = 3;
    public static double CAL_IDW_POWER = 2.0;
    public static double CAL_EXACT_SNAP_IN = 0.5;
    public static double CAL_EXTRAPOLATED_WARN_IN = 24.0;
    public double offset = 30;

    private static final ShotCalibrationTable DEFAULT = new ShotCalibrationTable(new ShotSample[]{
            //           idx zone   x      y      rpm     hood   aimX   aimY
//            new ShotSample(0, 72.0, 72.0, 3450.0, 0.525, 1.5, 136.0, "A", "A apex"),
//            new ShotSample(1, 36.0, 108.0, 2800.0, 0.22, -0.5, 133.0, "A", "A mid-left"),
//            new ShotSample(2, 108.0, 108.0, 3700.0, 0.525, -1.5, 128.0, "A", "A mid-right hood-max"),
//            new ShotSample(3, 72.0, 108.0, 3350.0, 0.44, 0.5, 132.0, "A", "A mid-center"),
//            new ShotSample(4, 40.0, 134.0, 2700.0, 0.00, 0.0, 134.0, "A", "A upper-left edge"),
//            new ShotSample(5, 50.0, 87.0, 3300.0, 0.525, 1.5, 136.0, "A", "A left interior hood-max"),
//            new ShotSample(6, 72.0, 130.0, 3200.0, 0.44, 0.5, 132.0, "A", "A upper-center"),
//            new ShotSample(7, 86.0, 89.0, 3450.0, 0.525, -0.5, 129.0, "A", "A right interior hood-max"),
//            new ShotSample(8, 110.0, 130.0, 3825.0, 0.525, -0.5, 135.0, "A", "A upper-right hood-max"),
//            new ShotSample(9, 72.0, 24.0, 4150.0, 0.525, 1.5, 129.0, "B", "B apex hood-max"),
//            new ShotSample(10, 54.0, 10.0, 4150.0, 0.525, 3.0, 129.0, "B", "B left hood-max"),
//            new ShotSample(11, 72.0, 12.0, 4200.0, 0.525, 1.5, 129.0, "B", "B center hood-max"),
//            new ShotSample(12, 90.0, 14.0, 4450.0, 0.525, -1.5, 134.0, "B", "B right hood-max"),

            new ShotSample(0, 54.0, 8.4, 3260.0 - 30, 0.45, 0, 144.0, "A", "A apex"),
            new ShotSample(1, 47.6, 12.2, 3260., 0.22, 0, 144.0, "A", "A mid-left"),
            new ShotSample(2, 34.9, 23.6, 3175.0- 30, 0.45, 0, 144.0, "A", "A mid-right hood-max"),
            new ShotSample(3, 37, 26.9, 3175.0- 30, 0.44, 0, 144.0, "A", "A mid-center"),
            new ShotSample(4, 23.2, 23.2, 3250.0, 0.00, 0, 144.0, "A", "A upper-left edge"),
            new ShotSample(5, 13.6, 13.6, 3300.0- 30, 0.45, 0, 144.0, "A", "A left interior hood-max"),
            new ShotSample(6, 13.9, 13.9, 3450.0, 0.44, 0, 144.0, "A", "A upper-center"),
            new ShotSample(7, 10.5, 89.0, 3590, 0.45, -0, 144.0, "A", "A right interior hood-max"),
            new ShotSample(8, 10.7, 130.0, 3650, 0.45, 0, 144.0, "A", "A upper-right hood-max"),
            new ShotSample(9, 20.5, 24.0, 3550, 0.525, 1., 129.0, "B", "B apex hood-max"),
            new ShotSample(10, 54.0, 10.0, 4000, 0.525, 1.5, 129.0, "B", "B left hood-max")

//            new ShotSample(0, 72.0, 72.0, 3300.0, 0.35, 1.,  136.0, "A", "A apex"),
//            new ShotSample(1, 36.0, 108.0, 2650.0, 0.22, -0.5, 133.0, "A", "A mid-left"),
//            new ShotSample(2, 108.0, 108.0, 3500.0, 0.4, -1., 128.0, "A", "A mid-right hood-max"),
//            new ShotSample(3, 72.0, 108.0, 3150.0, 0.3, 0.5, 132.0, "A", "A mid-center"),
//            new ShotSample(4, 40.0, 134.0, 2550.0, 0.00, 0.0, 134.0, "A", "A upper-left edge"),
//            new ShotSample(5, 50.0, 87.0, 3100.0, 0.3, 1., 136.0, "A", "A left interior hood-max"),
//            new ShotSample(6, 72.0, 130.0, 3000.0, 0.25, 0.5, 132.0, "A", "A upper-center"),
//            new ShotSample(7, 86.0, 89.0, 3250.0, 0.35, -0.5, 129.0, "A", "A right interior hood-max"),
//            new ShotSample(8, 110.0, 130.0, 3685.0, 0.4, -0.5, 135.0, "A", "A upper-right hood-max"),
//            new ShotSample(9, 72.0, 24.0, 4080.0, 0.525, 1., 129.0, "B", "B apex hood-max"),
//            new ShotSample(10, 54.0, 10.0, 4080.0, 0.525, 1.5, 129.0, "B", "B left hood-max"),
//            new ShotSample(11, 72.0, 12.0, 4130.0, 0.525, 1., 129.0, "B", "B center hood-max"),
//            new ShotSample(12, 90.0, 14.0, 4380.0, 0.525, -1., 134.0, "B", "B right hood-max"),
//            new ShotSample(13, 92.19, 86.89, 4420.0, 0.525, -0.4, 129.4, "B", "B right hood-max"),
    });

    public static ShotCalibrationTable active() {
        return DEFAULT;
    }

    private final ShotSample[] samples;

    private ShotCalibrationTable(ShotSample[] samples) {
        // Deliberate constructor argument: this is baked-in, one profile. If
        // red/blue profiles are ever needed, add a registry around active().
        this.samples = samples;
    }

    public ShotSample[] samples() {
        return samples;
    }

    public ShotSample sampleAt(int index) {
        if (index < 0 || index >= samples.length) {
            throw new IndexOutOfBoundsException(
                    "calibration point index " + index + " out of range [0, " + samples.length + ")");
        }
        return samples[index];
    }

    public int size() {
        return samples.length;
    }

    public ShotSolution lookup(double botX, double botY) {
        int k = Math.max(1, Math.min(CAL_IDW_K, samples.length));
        double power = CAL_IDW_POWER;

        // Gather (distance, index) pairs sorted ascending.
        Entry[] entries = new Entry[samples.length];
        for (int i = 0; i < samples.length; i++) {
            entries[i] = new Entry(i, samples[i].distanceTo(botX, botY));
        }
        Arrays.sort(entries, Comparator.comparingDouble(e -> e.distance));

        double nearestDistance = entries[0].distance;
        boolean extrapolated = nearestDistance > CAL_EXTRAPOLATED_WARN_IN;

        if (nearestDistance < CAL_EXACT_SNAP_IN) {
            ShotSample s = samples[entries[0].index];
            return new ShotSolution(
                    s.rpm, s.hoodPos, s.aimX, s.aimY,
                    new int[]{s.index},
                    new double[]{1.0},
                    extrapolated,
                    nearestDistance
            );
        }

        int[] idx = new int[k];
        double[] w = new double[k];
        double wSum = 0.0;
        for (int i = 0; i < k; i++) {
            idx[i] = entries[i].index;
            double d = Math.max(entries[i].distance, 1e-6);
            double weight = 1.0 / Math.pow(d, power);
            w[i] = weight;
            wSum += weight;
        }
        double rpm = 0.0, hood = 0.0, aimX = 0.0, aimY = 0.0;
        for (int i = 0; i < k; i++) {
            double norm = w[i] / wSum;
            w[i] = norm;
            ShotSample s = samples[idx[i]];
            rpm += norm * s.rpm;
            hood += norm * s.hoodPos;
            aimX += norm * s.aimX;
            aimY += norm * s.aimY;
        }

        return new ShotSolution(rpm, hood, aimX, aimY, idx, w, extrapolated, nearestDistance);
    }

    private static final class Entry {
        final int index;
        final double distance;
        Entry(int index, double distance) {
            this.index = index;
            this.distance = distance;
        }
    }
}
