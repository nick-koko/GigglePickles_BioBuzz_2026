import com.pedropathing.geometry.Pose;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import org.firstinspires.ftc.teamcode.pedroPathing.Pickles2026Teleop;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem2026;
import org.firstinspires.ftc.teamcode.subsystems.shot.ShotCalibrationTable2026;
import org.firstinspires.ftc.teamcode.subsystems.shot.ShotCalibrationTable2026.Sample;
import org.firstinspires.ftc.teamcode.subsystems.shot.ShotCalibrationTable2026.Solution;

/** Desktop checks using compiled production classes; no hardware is initialized. */
public final class ShotCalibration2026Test {
    private static int checks;
    private static Method calculate, command;

    public static void main(String[] args) throws Exception {
        calculate = Pickles2026Teleop.class.getDeclaredMethod("calculateShotSettings", Pose.class);
        calculate.setAccessible(true);
        command = Pickles2026Teleop.class.getDeclaredMethod("updateShooter",
                double.class, boolean.class, boolean.class);
        command.setAccessible(true);
        defaults();
        interpolation();
        invalidRows();
        targetSelection();
        actualShooterCommand();
        System.out.println("Passed " + checks + " shot calibration and RPM routing checks.");
    }

    private static void defaults() throws Exception {
        check("regression is default", !Pickles2026Teleop.USE_SHOT_TABLE);
        check("no fabricated 2026 calibration data", ShotCalibrationTable2026.SAMPLES.length == 0);
        Object shot = shot(58, 30);
        close("regression uses nominal distance", 3110.7, value(shot, "rpm"));
        check("regression source", text(shot, "source").equals("REGRESSION"));
        Pickles2026Teleop.USE_SHOT_TABLE = true;
        shot = shot(58, 30);
        close("empty table falls back to regression", 3110.7, value(shot, "rpm"));
        check("fallback reported", text(shot, "source").contains("no valid table rows"));
    }

    private static void interpolation() {
        ShotCalibrationTable2026.SAMPLES = new Sample[]{
                new Sample(0, 0, 3000, 2, -4), new Sample(10, 0, 4000, 4, -2)};
        Solution s = ShotCalibrationTable2026.lookup(0, 0);
        close("exact calibration RPM", 3000, s.flywheelRpm);
        close("exact calibration X delta", 2, s.deltaAimX);
        close("exact calibration Y delta", -4, s.deltaAimY);
        check("exact match uses one row", s.samplesUsed == 1);
        close("near-row snap", 3000, ShotCalibrationTable2026.lookup(.25, 0).flywheelRpm);
        s = ShotCalibrationTable2026.lookup(5, 0);
        close("midpoint RPM", 3500, s.flywheelRpm);
        close("midpoint X delta", 3, s.deltaAimX);
        close("midpoint Y delta", -3, s.deltaAimY);
        check("two rows blend when fewer than three exist", s.samplesUsed == 2);
        s = ShotCalibrationTable2026.lookup(2.5, 0);
        close("inverse-square weights", 3100, s.flywheelRpm);
        close("same weights apply to aim X", 2.2, s.deltaAimX);
        close("same weights apply to aim Y", -3.8, s.deltaAimY);

        ShotCalibrationTable2026.SAMPLES = new Sample[]{
                new Sample(0, 0, 3000, 0, 0), new Sample(10, 0, 3500, 1, 0),
                new Sample(0, 10, 4000, 2, 0), new Sample(100, 100, 6000, 100, 100)};
        s = ShotCalibrationTable2026.lookup(5, 5);
        close("only nearest three blend", 3500, s.flywheelRpm);
        close("far outlier excluded from aim", 1, s.deltaAimX);
        check("nearest three count", s.samplesUsed == 3);
        s = ShotCalibrationTable2026.lookup(300, 300);
        check("distant queries stay within sampled RPM range", s.flywheelRpm >= 3000 && s.flywheelRpm <= 6000);
        ShotCalibrationTable2026.NEAREST_SAMPLE_COUNT = 0;
        check("invalid neighbor count becomes nearest one", ShotCalibrationTable2026.lookup(5, 5).samplesUsed == 1);
        ShotCalibrationTable2026.NEAREST_SAMPLE_COUNT = 3;
        ShotCalibrationTable2026.EXACT_SNAP_DISTANCE_INCHES = 0;
        check("zero-distance lookup still safe without snap radius", ShotCalibrationTable2026.lookup(0, 0).flywheelRpm == 3000);
        ShotCalibrationTable2026.EXACT_SNAP_DISTANCE_INCHES = .5;
    }

    private static void invalidRows() throws Exception {
        ShotCalibrationTable2026.SAMPLES = new Sample[]{null, new Sample(Double.NaN, 0, 3000, 0, 0),
                new Sample(0, 0, 0, 0, 0), new Sample(0, 0, -1, 0, 0),
                new Sample(0, 0, Double.POSITIVE_INFINITY, 0, 0),
                new Sample(0, 0, 3000, Double.NaN, 0), new Sample(0, 0, 3500, 2, 3)};
        close("invalid rows skipped", 3500, ShotCalibrationTable2026.lookup(0, 0).flywheelRpm);
        check("invalid query returns no table solution", ShotCalibrationTable2026.lookup(Double.NaN, 0) == null);
        ShotCalibrationTable2026.SAMPLES = null;
        check("null table returns no solution", ShotCalibrationTable2026.lookup(0, 0) == null);
        Object shot = shot(58, 30);
        close("null table uses regression", 3110.7, value(shot, "rpm"));
    }

    private static void targetSelection() throws Exception {
        ShotCalibrationTable2026.SAMPLES = new Sample[]{new Sample(58, 30, 3450, 2, -3)};
        Pickles2026Teleop.USE_SHOT_TABLE = true;
        Object lower = shot(58, 30), upper = shot(58, 100);
        close("lower nominal target X", 58, value(lower, "nominalX"));
        close("lower nominal target Y", 60, value(lower, "nominalY"));
        close("lower corrected target X", 60, value(lower, "aimX"));
        close("lower corrected target Y", 57, value(lower, "aimY"));
        close("upper nominal target Y", 84, value(upper, "nominalY"));
        close("upper corrected target X", 60, value(upper, "aimX"));
        close("upper corrected target Y", 81, value(upper, "aimY"));
        close("range distance excludes aim offsets", 30, value(lower, "distanceInches"));
        close("table uses flywheel RPM directly", 3450, value(lower, "rpm"));
        check("table source reported", text(lower, "source").equals("TABLE"));
        Pickles2026Teleop.SHOOTER_RPM_TRIM = 25;
        close("table RPM trim is applied once", 3475, value(shot(58, 30), "rpm"));
        Pickles2026Teleop.USE_SHOT_TABLE = false;
        lower = shot(58, 30);
        close("disabled table has zero aim correction X", 58, value(lower, "aimX"));
        close("disabled table has zero aim correction Y", 60, value(lower, "aimY"));
        close("regression RPM trim is applied once", 3135.7, value(lower, "rpm"));
        Pickles2026Teleop.SHOOTER_RPM_TRIM = 0;
        close("split boundary uses original lower-target convention", 60, value(shot(58, 72), "nominalY"));
        double slope = Pickles2026Teleop.REGRESSION_RPM_PER_INCH;
        Pickles2026Teleop.REGRESSION_RPM_PER_INCH = Double.NaN;
        lower = shot(58, 30);
        close("invalid regression uses explicit fixed fallback", 3175, value(lower, "rpm"));
        check("invalid regression fallback reported", text(lower, "source").contains("FIXED FALLBACK"));
        Pickles2026Teleop.REGRESSION_RPM_PER_INCH = slope;
    }

    private static void actualShooterCommand() throws Exception {
        ShooterSubsystem2026 shooter = ShooterSubsystem2026.INSTANCE;
        shooter.stop();
        double rpm = value(shot(58, 30), "rpm");
        command.invoke(null, rpm, false, false);
        check("RPM calculation does not enable an idle shooter", !shooter.isEnabled());
        command.invoke(null, rpm, true, false);
        close("LB commands regression RPM instead of former fixed RPM", 3110.7, shooter.getTargetFlywheelRpm());
        rpm = value(shot(58, 20), "rpm");
        command.invoke(null, rpm, false, false);
        close("enabled shooter follows changing distance after LB release", 3380.7, shooter.getTargetFlywheelRpm());
        Pickles2026Teleop.USE_SHOT_TABLE = true;
        command.invoke(null, value(shot(58, 30), "rpm"), true, false);
        close("selected table value reaches production shooter", 3450, shooter.getTargetFlywheelRpm());
        command.invoke(null, 3450.0, false, true);
        check("RB can still stop shooter", !shooter.isEnabled());
        command.invoke(null, 3450.0, false, true);
        close("RB starts shooter at selected RPM", 3450, shooter.getTargetFlywheelRpm());
        shooter.stop();
        Pickles2026Teleop.AUTO_AIM_STARTS_SHOOTER = false;
        command.invoke(null, 3450.0, true, false);
        check("auto-aim start option is respected", !shooter.isEnabled());
    }

    private static Object shot(double x, double y) throws Exception {
        return calculate.invoke(null, new Pose(x, y, 0));
    }
    private static Field field(Object object, String name) throws Exception {
        Field field = object.getClass().getDeclaredField(name);
        field.setAccessible(true);
        return field;
    }
    private static double value(Object object, String name) throws Exception {
        return field(object, name).getDouble(object);
    }
    private static String text(Object object, String name) throws Exception {
        return (String) field(object, name).get(object);
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
