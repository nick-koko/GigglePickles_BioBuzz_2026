import com.pedropathing.geometry.Pose;
import java.lang.reflect.Method;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.firstinspires.ftc.teamcode.pedroPathing.CloseShootingAlone;
import org.firstinspires.ftc.teamcode.pedroPathing.CloseWithPartner;
import org.firstinspires.ftc.teamcode.pedroPathing.FarWithPartner;
import org.firstinspires.ftc.teamcode.pedroPathing.Pickles2026Teleop;
import org.firstinspires.ftc.teamcode.subsystems.AutonomousShooterRpm2026;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem2026;

/** Desktop checks against production RPM selection and shooter state, without hardware Init. */
public final class AutonomousRegression2026Test {
    private static int checks;

    public static void main(String[] args) throws Exception {
        if (args.length > 0 && args[0].equals("--report")) {
            comparison();
            return;
        }
        check("close alone defaults to fixed", !CloseShootingAlone.USE_DISTANCE_BASED_RPM);
        check("far partner defaults to fixed", !FarWithPartner.USE_DISTANCE_BASED_RPM);
        check("close partner defaults to fixed", !CloseWithPartner.USE_DISTANCE_BASED_RPM);
        check("close alone defaults to preload override", CloseShootingAlone.USE_FIRST_SHOT_LOW_RPM_OVERRIDE);
        check("close partner defaults to preload override", CloseWithPartner.USE_FIRST_SHOT_LOW_RPM_OVERRIDE);
        close("close alone preload is Low", 3050, CloseShootingAlone.SHOOTER_FLYWHEEL_RPM_Low);
        close("close partner preload is Low", 3050, CloseWithPartner.SHOOTER_FLYWHEEL_RPM_Low);
        close("close alone later shots are High", 3150, CloseShootingAlone.SHOOTER_FLYWHEEL_RPM_High);
        close("close partner later shots are High", 3150, CloseWithPartner.SHOOTER_FLYWHEEL_RPM_High);
        close("far partner starts and stays High in fixed mode", 3150, FarWithPartner.SHOOTER_FLYWHEEL_RPM);
        Pose first = waypoint("CloseShootingAlone", "closeShoot1");
        Pose far = waypoint("FarWithPartner", "farShootingFromStart");
        Pose later = waypoint("CloseWithPartner", "closeShoot2");
        close("planned first-close regression", 3206.21107791148, regression(first));
        close("planned far regression", 3102.06430337072, regression(far));
        close("planned later-close regression", 3188.94004867212, regression(later));
        close("Y split keeps lower target at exactly 72", 12,
                Pickles2026Teleop.regressionDistanceInches(58, 72));
        close("above split uses upper target", 16,
                Pickles2026Teleop.regressionDistanceInches(58, 100));

        checkRoutine(CloseShootingAlone.SHOOTER_FLYWHEEL_RPM_Low,
                CloseShootingAlone.SHOOTER_FLYWHEEL_RPM_High, first, far, later);
        checkRoutine(CloseWithPartner.SHOOTER_FLYWHEEL_RPM_Low,
                CloseWithPartner.SHOOTER_FLYWHEEL_RPM_High, first, far, later);
        checkRoutine(FarWithPartner.SHOOTER_FLYWHEEL_RPM,
                FarWithPartner.SHOOTER_FLYWHEEL_RPM, far, far, later);
        checkPreloadOverride(first, far);

        // The shared entry point must match TeleOp's regression path, including live edits.
        double slope = Pickles2026Teleop.REGRESSION_RPM_PER_INCH;
        double intercept = Pickles2026Teleop.REGRESSION_RPM_INTERCEPT;
        double targetX = Pickles2026Teleop.SHOOT_TARGET_X;
        double upperY = Pickles2026Teleop.SHOOT_TARGET_Y_ABOVE_SPLIT;
        boolean useTable = Pickles2026Teleop.USE_SHOT_TABLE;
        double trim = Pickles2026Teleop.SHOOTER_RPM_TRIM;
        try {
            Pickles2026Teleop.REGRESSION_RPM_PER_INCH = 20;
            Pickles2026Teleop.REGRESSION_RPM_INTERCEPT = 2500;
            Pickles2026Teleop.SHOOT_TARGET_X = 60;
            Pickles2026Teleop.SHOOT_TARGET_Y_ABOVE_SPLIT = 90;
            Pickles2026Teleop.USE_SHOT_TABLE = false;
            Pickles2026Teleop.SHOOTER_RPM_TRIM = 0;
            double expected = 2500 + 20 * Math.hypot(60 - far.getX(), 90 - far.getY());
            close("live target and coefficients are shared", expected, regression(far));
            Method calculate = Pickles2026Teleop.class.getDeclaredMethod("calculateShotSettings", Pose.class);
            calculate.setAccessible(true);
            Object settings = calculate.invoke(null, far);
            java.lang.reflect.Field rpm = settings.getClass().getDeclaredField("rpm");
            rpm.setAccessible(true);
            close("autonomous matches actual TeleOp regression", rpm.getDouble(settings), regression(far));
            Pickles2026Teleop.USE_SHOT_TABLE = true;
            Pickles2026Teleop.SHOOTER_RPM_TRIM = 500;
            close("table toggle and operator trim do not leak into auton", expected, regression(far));
        } finally {
            Pickles2026Teleop.REGRESSION_RPM_PER_INCH = slope;
            Pickles2026Teleop.REGRESSION_RPM_INTERCEPT = intercept;
            Pickles2026Teleop.SHOOT_TARGET_X = targetX;
            Pickles2026Teleop.SHOOT_TARGET_Y_ABOVE_SPLIT = upperY;
            Pickles2026Teleop.USE_SHOT_TABLE = useTable;
            Pickles2026Teleop.SHOOTER_RPM_TRIM = trim;
            ShooterSubsystem2026.INSTANCE.stop();
        }
        System.out.println("Passed " + checks + " autonomous regression checks.");
    }

    private static void checkRoutine(double firstFixed, double laterFixed,
                                     Pose first, Pose far, Pose later) {
        ShooterSubsystem2026 shooter = ShooterSubsystem2026.INSTANCE;
        shooter.stop();
        AutonomousShooterRpm2026 controller = new AutonomousShooterRpm2026();
        controller.reset(firstFixed);
        controller.update(true, first);
        check("Init/reset cannot start flywheel", !shooter.isEnabled());
        controller.start(firstFixed, false, first);
        close("disabled flag preserves first fixed RPM", firstFixed, shooter.getTargetFlywheelRpm());
        controller.update(false, far);
        close("disabled flag preserves RPM while moving", firstFixed, shooter.getTargetFlywheelRpm());
        controller.start(laterFixed, false, far);
        close("later fixed phase is retained", laterFixed, shooter.getTargetFlywheelRpm());
        controller.update(true, far);
        close("enabling flag switches to far regression", regression(far), shooter.getTargetFlywheelRpm());
        controller.update(true, later);
        close("regression follows actual pose on return close", regression(later), shooter.getTargetFlywheelRpm());
        controller.update(false, later);
        close("disabling restores original later-phase RPM", laterFixed, shooter.getTargetFlywheelRpm());
        controller.start(laterFixed, true, new Pose(Double.NaN, 0, 0));
        close("invalid pose uses current phase's fixed fallback", laterFixed, shooter.getTargetFlywheelRpm());
        controller.update(true, null);
        close("missing pose uses fixed fallback", laterFixed, shooter.getTargetFlywheelRpm());
        shooter.stop();
        controller.update(true, first);
        check("regression never restarts a parked/stopped shooter", !shooter.isEnabled());
    }

    private static double regression(Pose pose) {
        return Pickles2026Teleop.regressionFlywheelRpm(pose.getX(), pose.getY());
    }

    private static void checkPreloadOverride(Pose first, Pose far) {
        ShooterSubsystem2026 shooter = ShooterSubsystem2026.INSTANCE;
        AutonomousShooterRpm2026 controller = new AutonomousShooterRpm2026();
        for (boolean distance : new boolean[]{false, true}) {
            for (boolean override : new boolean[]{false, true}) {
                shooter.stop();
                controller.reset(3050, override);
                check("preload override reset remains passive", !shooter.isEnabled());
                double expectedFirst = distance && !override ? regression(first) : 3050;
                close("Init preview respects both flags", expectedFirst, controller.selectedRpm(distance, first));
                controller.start(3050, distance, first, override);
                close("first volley respects both flags", expectedFirst, shooter.getTargetFlywheelRpm());
                for (int i = 0; i < 12; i++) {
                    controller.update(distance, first);
                    close("periodic updates cannot bypass first-volley override",
                            expectedFirst, shooter.getTargetFlywheelRpm());
                }
                controller.finishFirstVolley();
                controller.update(distance, first);
                close("regression resumes after entire first volley", distance ? regression(first) : 3050,
                        shooter.getTargetFlywheelRpm());
                controller.start(3150, distance, far);
                controller.update(distance, far);
                close("later volleys use regression or High", distance ? regression(far) : 3150,
                        shooter.getTargetFlywheelRpm());
            }
        }
        controller.start(3050, true, first, true);
        controller.update(true, far);
        close("pose changes during preload cannot alter Low RPM", 3050, shooter.getTargetFlywheelRpm());
        controller.update(false, first);
        controller.update(true, first);
        close("regression toggle cannot bypass preload override", 3050, shooter.getTargetFlywheelRpm());
        shooter.stop();
        controller.finishFirstVolley();
        controller.update(true, first);
        check("ending override cannot restart a stopped shooter", !shooter.isEnabled());
    }

    /** Pull waypoint coordinates from the actual opmode sources for the comparison. */
    private static Pose waypoint(String opmode, String name) throws Exception {
        String source = new String(Files.readAllBytes(Paths.get("TeamCode/src/main/java/"
                + "org/firstinspires/ftc/teamcode/pedroPathing/" + opmode + ".java")),
                java.nio.charset.StandardCharsets.UTF_8);
        Matcher matcher = Pattern.compile("private final Pose " + Pattern.quote(name)
                + "\\s*=\\s*poseFactory\\.of\\(\\s*([-0-9.]+),\\s*([-0-9.]+),").matcher(source);
        if (!matcher.find()) throw new AssertionError("Missing waypoint " + opmode + "." + name);
        return new Pose(Double.parseDouble(matcher.group(1)), Double.parseDouble(matcher.group(2)), 0);
    }

    private static void comparison() throws Exception {
        System.out.println("| Program | Shot | Planned robot X, Y (in) | Distance (in) | Fixed RPM | Raw regression RPM | Raw change | Regression + preload override |");
        System.out.println("|---|---|---:|---:|---:|---:|---:|---:|");
        row("Close alone", "Preloads", "CloseShootingAlone", "closeShoot1", CloseShootingAlone.SHOOTER_FLYWHEEL_RPM_Low,
                CloseShootingAlone.USE_FIRST_SHOT_LOW_RPM_OVERRIDE);
        row("Close alone", "Far flower shots / optional far garden shot", "CloseShootingAlone", "farShooting", CloseShootingAlone.SHOOTER_FLYWHEEL_RPM_High);
        row("Close alone", "Garden balls from close (default)", "CloseShootingAlone", "closeShoot2", CloseShootingAlone.SHOOTER_FLYWHEEL_RPM_High);
        row("Far with partner", "Far preloads + far flower", "FarWithPartner", "farShootingFromStart", FarWithPartner.SHOOTER_FLYWHEEL_RPM);
        row("Far with partner", "Extended plan: garden + optional near-flower shot", "FarWithPartner", "closeShoot2", FarWithPartner.SHOOTER_FLYWHEEL_RPM);
        row("Close with partner", "Preloads", "CloseWithPartner", "closeShoot1", CloseWithPartner.SHOOTER_FLYWHEEL_RPM_Low,
                CloseWithPartner.USE_FIRST_SHOT_LOW_RPM_OVERRIDE);
        row("Close with partner", "Help-far branch: far flower", "CloseWithPartner", "farShooting", CloseWithPartner.SHOOTER_FLYWHEEL_RPM_High);
        row("Close with partner", "Garden + optional near-flower shot", "CloseWithPartner", "closeShoot2", CloseWithPartner.SHOOTER_FLYWHEEL_RPM_High);
    }

    private static void row(String program, String shot, String opmode, String name, double fixed) throws Exception {
        row(program, shot, opmode, name, fixed, false);
    }

    private static void row(String program, String shot, String opmode, String name, double fixed,
                            boolean overridePreload) throws Exception {
        Pose pose = waypoint(opmode, name);
        double distance = Pickles2026Teleop.regressionDistanceInches(pose.getX(), pose.getY());
        double rpm = regression(pose);
        System.out.printf(Locale.US, "| %s | %s | %.4f, %.4f | %.2f | %.0f | %.1f | %+.1f (%+.2f%%) | %.1f |%n",
                program, shot, pose.getX(), pose.getY(), distance, fixed, rpm, rpm - fixed,
                100 * (rpm - fixed) / fixed, overridePreload ? fixed : rpm);
    }

    private static void close(String label, double expected, double actual) {
        check(label + " (expected " + expected + ", got " + actual + ")",
                Double.isFinite(actual) && Math.abs(expected - actual) < 1e-7);
    }
    private static void check(String label, boolean condition) {
        checks++;
        if (!condition) throw new AssertionError(label);
    }
}
