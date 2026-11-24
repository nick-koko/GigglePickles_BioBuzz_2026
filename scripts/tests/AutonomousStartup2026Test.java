import org.firstinspires.ftc.teamcode.subsystems.AutonomousShooterStartup2026;
import org.firstinspires.ftc.teamcode.subsystems.AutonomousShooterRpm2026;
import org.firstinspires.ftc.teamcode.pedroPathing.CloseShootingAlone;
import org.firstinspires.ftc.teamcode.pedroPathing.CloseWithPartner;
import org.firstinspires.ftc.teamcode.pedroPathing.FarWithPartner;

public final class AutonomousStartup2026Test {
    private static int checks;
    private static final long START = 1_000_000_000L;
    private static final AutonomousShooterStartup2026 gate = new AutonomousShooterStartup2026();

    public static void main(String[] args) {
        check("close alone preload", CloseShootingAlone.SHOOTER_FLYWHEEL_RPM_Low == 3100);
        check("close partner preload", CloseWithPartner.SHOOTER_FLYWHEEL_RPM_Low == 3100);
        check("far preload", FarWithPartner.SHOOTER_PRELOAD_RPM == 3100);
        check("later fixed targets", AutonomousShooterRpm2026.DEFAULT_HIGH_RPM == 3150);
        gate.start(START, 0);
        sample(800, 3100);
        sample(875, 3100);
        check("hold not yet complete", !gate.isFinished());
        sample(950, 3100);
        check("150 ms of fresh in-band samples", gate.isFinished() && gate.getResult().equals("READY"));
        sample(975, 3000);
        check("RPM departure during path clears readiness", !gate.isFinished());
        sample(1000, 3100);
        sample(1075, 3100);
        sample(1150, 3100);
        check("recovers after another continuous hold", gate.isFinished());
        gate.complete();
        sample(2500, 0);
        check("recorded outcome survives feeding RPM sag", gate.getResult().equals("READY"));
        gate.start(START, 0);
        sample(800, 3100);
        gate.update(START + 950_000_000L, START + 800_000_000L, 3100, 3100, true);
        check("stale sample cannot satisfy hold", !gate.isFinished());
        sample(960, 3100);
        sample(1035, 3100);
        check("gap requires new hold", !gate.isFinished());
        sample(1110, 3100);
        check("fresh hold after gap", gate.isFinished());
        gate.start(START, 1.2);
        for (int ms=800;ms<=950;ms+=50) sample(ms, 3100);
        check("optional minimum delays ready result", !gate.isFinished());
        for (int ms=1000;ms<=1200;ms+=50) sample(ms, 3100);
        check("minimum measured from shooter start", gate.isFinished());
        gate.start(START, 0);
        for (int ms=800;ms<=950;ms+=50) sample(ms, 3100);
        gate.update(START+975_000_000L, START+975_000_000L, 3150, 3150, true);
        check("changed target restarts hold despite in-band RPM", !gate.isFinished());
        gate.update(START+1_000_000_000L, START+1_000_000_000L, Double.NaN, 3150, true);
        check("bad reading clears readiness", !gate.isFinished());
        sample(1999, 0);
        check("timeout not early", !gate.isFinished());
        sample(2000, 0);
        check("bounded timeout allows match to proceed", gate.isFinished()
                && gate.getResult().equals("TIMEOUT_FEED_ANYWAY"));
        gate.reset();
        check("Init reset clears previous outcome", !gate.isFinished());
        boolean failed = false;
        try { AutonomousShooterStartup2026.validateConfiguration(3); }
        catch (IllegalArgumentException expected) { failed = true; }
        check("invalid minimum fails at Init", failed);
        System.out.println("Passed " + checks + " autonomous opening readiness checks.");
    }

    private static void sample(int ms, double rpm) {
        long now = START + ms * 1_000_000L;
        gate.update(now, now, rpm, 3100, true);
    }

    private static void check(String name, boolean pass) {
        checks++;
        if (!pass) throw new AssertionError(name);
    }
}

