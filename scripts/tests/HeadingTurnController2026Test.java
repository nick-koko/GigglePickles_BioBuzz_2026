import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.HeadingTurnController2026;

/** Standalone JVM checks; compile with the cached Pedro core 2.1.2 JAR. */
public final class HeadingTurnController2026Test {
    private static int checks;

    public static void main(String[] args) throws Exception {
        for (double degrees : new double[]{-90, -45, -10, -2, -1, 0, 1, 2, 10, 45, 90}) {
            double raw = custom(degrees, 0, 0.0165, 0);
            double expected = Math.abs(degrees) <= 1 ? 0
                    : Math.copySign(Math.min(0.8, Math.abs(degrees * 0.0165) + 0.052), degrees);
            close("legacy default at " + degrees, expected, finish(raw, degrees, 0, 0, 0.052, 0.8));
        }
        close("minimum corrects small error", 0.052, finish(custom(2, 0, .0165, 0), 2, 0, .052, 0, .8));
        close("minimum leaves large correction alone", .165, finish(custom(10, 0, .0165, 0), 10, 0, .052, 0, .8));
        close("kS is additive", .085, finish(custom(2, 0, .0165, 0), 2, 0, 0, .052, .8));
        close("kS then floor", .1, finish(custom(2, 0, .0165, 0), 2, 0, .1, .01, .8));
        close("negative floor", -.052, finish(custom(-2, 0, .0165, 0), -2, 0, .052, 0, .8));
        close("zero max disables turn", 0, finish(.5, 30, 0, .1, .1, 0));
        close("negative cap", -.2, finish(-.5, -30, 0, .1, .1, .2));
        close("zero controller stays zero", 0, finish(0, 10, 0, .1, .1, .8));

        double braking = custom(10, 100, .0165, .002);
        close("D brakes before crossing", -.035, braking);
        close("compensation preserves braking", braking, finish(braking, 10, 100, .2, .2, .8));
        close("D brakes through tolerance", -.1, finish(custom(.5, 50, .0165, .002), .5, 50, .2, .2, .8));
        close("settled suppresses noise", 0, finish(custom(.5, 2, .0165, .002), .5, 2, .2, .2, .8));
        close("D zero preserves tolerance at speed", 0, finish(custom(.5, 100, .0165, 0), .5, 100, 0, .052, .8));
        close("wrapped custom error", -.033, custom(358, 0, .0165, 0));

        FollowerConstants constants = new FollowerConstants()
                .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, .11, 0))
                .useSecondaryHeadingPIDF(false);
        HeadingTurnController2026 pedro = new HeadingTurnController2026(constants);
        Pose pose = new Pose(0, 0, Math.toRadians(90));
        close("Pedro engagement primes only", 0, pedro.calculatePedro(pose, Math.toRadians(80)));
        Thread.sleep(10);
        close("Pedro clockwise sign and no derivative kick", -1.5 * Math.toRadians(10),
                pedro.calculatePedro(pose, Math.toRadians(80)));
        pedro.reset();
        close("Pedro re-engagement primes only", 0, pedro.calculatePedro(pose, Math.toRadians(100)));
        Thread.sleep(10);
        close("Pedro counterclockwise sign", 1.5 * Math.toRadians(10),
                pedro.calculatePedro(pose, Math.toRadians(100)));
        constants.coefficientsHeadingPIDF.setCoefficients(2, 0, 0, 0);
        close("Pedro shares the configured coefficient object", 2 * Math.toRadians(10),
                pedro.calculatePedro(pose, Math.toRadians(100)));
        pedro.reset();
        pedro.calculatePedro(new Pose(0, 0, 0), Math.toRadians(179));
        close("Pedro angle-branch change reprimes", 0,
                pedro.calculatePedro(new Pose(0, 0, 0), Math.toRadians(-179)));
        constants.useSecondaryHeadingPIDF = true;
        constants.coefficientsSecondaryHeadingPIDF = new PIDFCoefficients(3, 0, 0, 0);
        pedro.reset();
        pedro.calculatePedro(pose, Math.toRadians(92));
        close("Pedro secondary heading gains", 3 * Math.toRadians(2),
                pedro.calculatePedro(pose, Math.toRadians(92)));
        System.out.println("Passed " + checks + " heading controller checks.");
    }

    private static double custom(double errorDegrees, double rateDegrees, double p, double d) {
        return HeadingTurnController2026.calculateCustom(
                Math.toRadians(errorDegrees), Math.toRadians(rateDegrees), p, d, 1);
    }

    private static double finish(double raw, double errorDegrees, double rateDegrees,
                                 double minimum, double ks, double maximum) {
        return HeadingTurnController2026.applyCompensationAndLimits(raw,
                Math.toRadians(errorDegrees), Math.toRadians(rateDegrees), minimum, ks, maximum, 1, 5);
    }

    private static void close(String label, double expected, double actual) {
        checks++;
        if (!Double.isFinite(actual) || Math.abs(expected - actual) > 1e-8) {
            throw new AssertionError(label + ": expected " + expected + ", got " + actual);
        }
    }
}
