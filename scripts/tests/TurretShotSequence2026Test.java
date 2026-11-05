import org.firstinspires.ftc.teamcode.subsystems.IntakeWithSensorsSubsystem2026;
import org.firstinspires.ftc.teamcode.subsystems.ShotSequenceController2026;

/** Runs against the compiled production shot controller and its uninitialized subsystems. */
public final class TurretShotSequence2026Test {
    public static void main(String[] args) {
        ShotSequenceController2026 shots = new ShotSequenceController2026();
        shots.reset();
        // Same gating used by teleop during recovery, with RT held throughout.
        for (int i = 0; i < 20; i++) {
            shots.cancel();
            shots.update(true);
            check("calibration cannot start feeding", !shots.isActive());
            check("gate feed request remains off",
                    !IntakeWithSensorsSubsystem2026.INSTANCE.isFeedingToShooter());
        }
        // Calibration has completed. Holding the old request must not fire.
        shots.update(true);
        check("no queued shot on calibration completion", !shots.isActive());
        shots.update(false);
        shots.update(true);
        check("release and repress starts normal feed", shots.isActive());
        check("normal feed reaches intake", IntakeWithSensorsSubsystem2026.INSTANCE.isFeedingToShooter());
        shots.cancel();
        check("cancel closes feed request", !IntakeWithSensorsSubsystem2026.INSTANCE.isFeedingToShooter());
        // Without a turret, there is no cancellation gate and the first RT still works.
        shots.reset();
        shots.update(true);
        check("chassis-only first RT unchanged", shots.isActive());
        shots.cancel();
        System.out.println("Passed 45 turret/shot sequence checks.");
    }

    private static void check(String label, boolean condition) {
        if (!condition) throw new AssertionError(label);
    }
}
