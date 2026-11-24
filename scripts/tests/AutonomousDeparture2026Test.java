import java.lang.reflect.Field;
import com.pedropathing.geometry.Pose;
import dev.nextftc.core.commands.Command;
import org.firstinspires.ftc.teamcode.logging.RobotLogging2026;
import org.firstinspires.ftc.teamcode.subsystems.*;

/** Exercise the production command entry point and asynchronous feed completion. */
public final class AutonomousDeparture2026Test {
    private static int checks;
    private static final ShotSequenceController2026 shots = new ShotSequenceController2026();
    private static final AutonomousShooterRpm2026 rpm = new AutonomousShooterRpm2026();
    private static final AutonomousShotCoordinator2026 coordinator =
            new AutonomousShotCoordinator2026(shots, rpm, new RobotLogging2026());
    private static final ShooterSubsystem2026 shooter = ShooterSubsystem2026.INSTANCE;

    public static void main(String[] args) throws Exception {
        shots.reset();
        rpm.reset(3100, true);
        rpm.start(3100, false, null, true);
        int[] completions = {0};
        Command first = coordinator.shoot(.8, () -> {
            rpm.finishFirstVolley();
            completions[0]++;
        });
        first.start();
        pump(first);
        check("first volley starts normally", shots.isActive() && shots.getSequenceCount() == 1);
        elapsed(.79); pump(first);
        check("departure not yet released", !first.isDone());
        elapsed(.81); pump(first);
        check("departure releases while feeding continues", first.isDone() && shots.isActive());
        check("feed output retained", IntakeWithSensorsSubsystem2026.INSTANCE.isFeedingToShooter());
        check("completion callback waits", completions[0] == 0);
        rpm.update(true, new Pose(0, 100, 0));
        check("moving pose cannot change volley RPM", shooter.getTargetFlywheelRpm() == 3100);
        rpm.start(3300, false, null);
        check("later fixed command is deferred", shooter.getTargetFlywheelRpm() == 3100);
        Command second = coordinator.shoot(.8);
        second.start(); pump(second);
        check("next shot waits for active feed", shots.getSequenceCount() == 1);
        elapsed(1.21); shots.update(); coordinator.update();
        check("completion runs once", completions[0] == 1);
        coordinator.update();
        check("completion never repeats", completions[0] == 1);
        rpm.update(false, null);
        check("deferred fixed RPM applies after feeding", shooter.getTargetFlywheelRpm() == 3300);
        pump(second);
        check("second shot starts after completion", shots.getSequenceCount() == 2 && shots.isActive());
        coordinator.cancel();
        check("Stop closes feed", !shots.isActive()
                && !IntakeWithSensorsSubsystem2026.INSTANCE.isFeedingToShooter());

        rpm.start(3100, false, null, true);
        Command longWait = coordinator.shoot(1.5, () -> completions[0]++);
        longWait.start(); pump(longWait);
        elapsed(1.3); pump(longWait);
        check("feed can complete before departure", !shots.isActive() && !longWait.isDone());
        check("callback follows feed completion", completions[0] == 2);
        elapsed(1.6); pump(longWait);
        check("longer per-shot delay honored", longWait.isDone());

        Command cancelled = coordinator.shoot(.8, () -> completions[0]++);
        cancelled.start(); pump(cancelled);
        coordinator.cancel(); coordinator.update();
        check("cancel never runs successful completion callback", completions[0] == 2);
        Pose beforeMove = new Pose(54, 10, 0);
        Pose afterMove = new Pose(54, 30, 0);
        rpm.start(3100, true, beforeMove);
        double held = shooter.getTargetFlywheelRpm();
        rpm.holdVolleyRpm(); rpm.update(true, afterMove);
        check("regression held through movement", shooter.getTargetFlywheelRpm() == held);
        rpm.releaseVolleyRpm(); rpm.update(true, afterMove);
        check("regression resumes at current pose", shooter.getTargetFlywheelRpm() != held
                && shooter.getTargetFlywheelRpm() == rpm.selectedRpm(true, afterMove));
        shooter.stop(); rpm.update(true, new Pose(0, 100, 0));
        check("stopped shooter stays stopped", !shooter.isEnabled());

        for (double bad : new double[]{-1, Double.NaN, Double.POSITIVE_INFINITY}) {
            boolean failed = false;
            try { coordinator.shoot(bad); } catch (IllegalArgumentException expected) { failed = true; }
            check("invalid departure rejected", failed);
        }
        System.out.println("Passed " + checks + " autonomous departure/feed integration checks.");
    }

    private static void pump(Command command) {
        for (int i=0; i<12 && !command.isDone(); i++) {
            shots.update();
            coordinator.update();
            command.update();
        }
    }

    private static void elapsed(double seconds) throws Exception {
        Field start = ShotSequenceController2026.class.getDeclaredField("sequenceStartNanos");
        start.setAccessible(true);
        start.setLong(shots, System.nanoTime() - (long)(seconds * 1e9));
    }

    private static void check(String name, boolean pass) {
        checks++;
        if (!pass) throw new AssertionError(name);
    }
}

