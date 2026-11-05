import org.firstinspires.ftc.teamcode.util.TurretController2026;
import org.firstinspires.ftc.teamcode.util.TurretController2026.Config;
import org.firstinspires.ftc.teamcode.util.TurretController2026.Mode;
import org.firstinspires.ftc.teamcode.util.TurretController2026.State;

/** Standalone JVM tests with passive sensors and a servo-command recorder. */
public final class TurretController2026Test {
    private static int checks;

    public static void main(String[] args) {
        disabledAndFixed();
        calibrationAndPreload();
        handoffWhileMoving();
        invalidHandoffs();
        initResetAndStop();
        aimingAndLimits();
        calibrationFailures();
        System.out.println("Passed " + checks + " turret controller checks.");
    }

    private static void disabledAndFixed() {
        Config c = new Config();
        TurretController2026 off = new TurretController2026(Mode.CHASSIS_ONLY, c, null, 0);
        off.start(0, false);
        off.update(1);
        check("old robot needs no turret hardware", off.getState() == State.DISABLED);
        check("old robot shooting unchanged", off.canShoot(0));

        c.fixedAngle = 30;
        FakeHardware h = new FakeHardware(c);
        h.encodersForbidden = true;
        TurretController2026 fixed = new TurretController2026(Mode.FIXED, c, h, 0);
        fixed.update(1);
        check("fixed Init does not write", h.writes == 0);
        fixed.start(2, false);
        check("fixed Play commands both servos", h.writes == 1);
        close("fixed left preload", .5 - 31.0 / 355, h.left);
        close("fixed right preload", .5 - 29.0 / 355, h.right);
        check("fixed hold settles before feed", !fixed.canShoot(3));
        check("fixed hold permits feed after settle", fixed.canShoot(4));
        close("fixed chassis aim accounts for turret bearing", 30, fixed.getFixedRobotRelativeHeading());
        c.fixedAngle = 60;
        fixed.update(4.02);
        check("live fixed angle changes servo hold", h.left < .5 - 31.0 / 355);
        check("live fixed adjustment blocks feeding", !fixed.canShoot(4.02));
    }

    private static void calibrationAndPreload() {
        Config c = new Config();
        close("new gear ratio is derived", 20.48, c.countsPerDegree());
        FakeHardware h = new FakeHardware(c);
        TurretController2026 t = new TurretController2026(Mode.TRACKING, c, h, 0);
        t.update(.5);
        t.update(1);
        check("practice Init is passive", h.writes == 0);
        check("practice cannot feed before Play", !t.canShoot(1));
        t.start(1, false);
        close("startup retains left preload", .5 - 1.0 / 355, h.left);
        close("startup retains right preload", .5 + 1.0 / 355, h.right);
        check("startup cannot feed while centering", !t.canShoot(1));
        t.update(1.1);
        t.update(1.2);
        t.update(3.1);
        check("practice calibration completes", t.isCalibrated());
        check("practice enables feed after calibration", t.canShoot(3.1));
        close("nominal servo zero", 0, t.getServoOffset());

        Config shifted = new Config();
        FakeHardware hs = new FakeHardware(shifted);
        hs.angle = 6;
        TurretController2026 ts = calibrated(shifted, hs);
        close("known centering learns servo offset", 6, ts.getServoOffset());
        close("analog sets physical angle", 6, ts.getMeasuredAngle());
        check("calibration does not reset counts", hs.ticks() != 0);
        close("absolute sector uses physical range, not circular distance", 50,
                TurretController2026.resolveAbsolute(90, 0, 1.8, 1, 0, -130, 130));
        close("absolute negative direction supported", -50,
                TurretController2026.resolveAbsolute(90, 0, 1.8, -1, 0, -130, 130));
        close("center phase unwrap", -0.2,
                TurretController2026.resolveAbsolute(359.64, 0, 1.8, 1, 0, -130, 130));

        Config moving = new Config();
        FakeHardware hm = new FakeHardware(moving);
        hm.angle = 60;
        TurretController2026 tm = new TurretController2026(Mode.TRACKING, moving, hm, 0);
        tm.start(0, false);
        tm.update(.1);
        hm.angle = 0;
        tm.update(2.0);
        check("moving center is not accepted", !tm.isCalibrated());
        tm.update(2.02);
        tm.update(2.04);
        tm.update(2.06);
        check("settled center is accepted", tm.isCalibrated());
        close("sweep samples do not bias center calibration", 0, tm.getServoOffset());
    }

    private static void handoffWhileMoving() {
        Config c = new Config();
        FakeHardware h = new FakeHardware(c);
        h.angle = 6;
        TurretController2026 auto = calibrated(c, h);
        h.angle = 40;
        // Snapshot samples fresh counts even though periodic still thinks the turret is at 6 degrees.
        TurretController2026.Snapshot snapshot = auto.snapshot();
        check("valid autonomous snapshot", snapshot != null);
        near("snapshot uses fresh measurement", 40, snapshot.measuredAngle, .05);
        h.angle = 45;
        TurretController2026 tele = new TurretController2026(Mode.TRACKING, new Config(), h, 3);
        int before = h.writes;
        check("restore accepts coasting turret", tele.restore(snapshot));
        near("coasting included before teleop Init", 45, tele.getMeasuredAngle(), .05);
        h.angle = 50;
        tele.update(3.1);
        near("coasting included during Init", 50, tele.getMeasuredAngle(), .05);
        check("restored Init still cannot write", h.writes == before);
        check("restored Init still cannot feed", !tele.canShoot(3.1));
        close("motion is not learned as servo zero", 6, tele.getServoOffset());
        close("last left command preserved in Init", snapshot.leftPosition, tele.getLeftPosition());
        close("last right command preserved in Init", snapshot.rightPosition, tele.getRightPosition());
        tele.start(3.2, false);
        check("valid handoff skips recentering", tele.getState() == State.READY);
        check("valid handoff ready at Play", tele.canShoot(3.2));
        tele.update(3.22);
        check("restored controller begins writing after Play", h.writes > before);
        near("restored target starts at actual angle", 50, tele.getTarget(), .05);
        check("restored telemetry source", tele.isRestoredFromAutonomous());
    }

    private static void invalidHandoffs() {
        Config c = new Config();
        FakeHardware h = new FakeHardware(c);
        TurretController2026 auto = calibrated(c, h);
        h.angle = 40;
        TurretController2026.Snapshot snapshot = auto.snapshot();
        h.origin = -h.ticks(); // Hub counter reset, mechanism unchanged.
        TurretController2026 tele = new TurretController2026(Mode.TRACKING, new Config(), h, 3);
        int before = h.writes;
        check("reset count rejected using passive analog check", !tele.restore(snapshot));
        tele.update(3.1);
        check("invalid handoff still passive in Init", h.writes == before);
        tele.start(3.2, false);
        check("invalid handoff automatically recovers on Play", tele.getState() == State.CENTERING);

        h.origin = 0;
        h.angle = 0;
        Config changed = new Config();
        changed.encoderRatio = 4.8;
        TurretController2026 wrongRatio = new TurretController2026(Mode.TRACKING, changed, h, 4);
        check("changed calibration cannot reuse handoff", !wrongRatio.restore(snapshot));
        TurretController2026 noAuto = new TurretController2026(Mode.TRACKING, c, h, 4);
        check("missing handoff is normal recovery case", !noAuto.restore(null));
    }

    private static void initResetAndStop() {
        Config c = new Config();
        FakeHardware h = new FakeHardware(c);
        TurretController2026 auto = calibrated(c, h);
        h.angle = 40;
        TurretController2026.Snapshot snapshot = auto.snapshot();
        TurretController2026 tele = new TurretController2026(Mode.TRACKING, c, h, 3);
        check("reference initially valid", tele.restore(snapshot));
        h.origin = -h.ticks();
        int before = h.writes;
        tele.update(3.1);
        check("reset during Init invalidates calibration", !tele.isCalibrated());
        check("reset during Init never moves turret", h.writes == before);
        tele.start(3.2, false);
        check("reset during Init recovered after Play", tele.getState() == State.CENTERING);
        before = h.writes;
        tele.stop();
        tele.update(3.3);
        tele.update(7);
        check("Stop blocks all later servo writes", h.writes == before);
        check("Stop blocks feed", !tele.canShoot(7));
    }

    private static void aimingAndLimits() {
        Config c = new Config();
        FakeHardware h = new FakeHardware(c);
        TurretController2026 t = calibrated(c, h);
        t.aim(90, 30);
        close("field bearing minus robot heading", 60, t.getTarget());
        t.aim(-170, 170);
        close("field wrap is shortest direction", 20, t.getTarget());
        t.aim(180, 0);
        check("unreachable direction identified", !t.isTargetReachable());
        check("unreachable direction respects limits", Math.abs(t.getTarget()) <= 130);
        t.setTarget(130);
        t.update(2.12);
        check("command rate and trim prevent full sweep in one loop", h.left > .4);
        for (int i = 1; i <= 50; i++) t.update(2.12 + .02 * i);
        check("servo endpoint clipping", h.left >= .5 - 130.0 / 355);

        Config rear = new Config();
        rear.frontOffset = 180;
        FakeHardware hr = new FakeHardware(rear);
        TurretController2026 tr = calibrated(rear, hr);
        tr.aim(180, 0);
        close("rear zero mapping", 0, tr.getTarget());
        TurretController2026 fixedRear = new TurretController2026(Mode.FIXED, rear, hr, 0);
        close("rear fixed chassis aim offset", -180, fixedRear.getFixedRobotRelativeHeading());
    }

    private static void calibrationFailures() {
        Config c = new Config();
        FakeHardware h = new FakeHardware(c);
        h.angle = 70;
        TurretController2026 t = calibrated(c, h);
        check("wrong center phase faults instead of guessing", t.getState() == State.FAULT);
        check("fault blocks feeding", !t.canShoot(3));
        check("fault cannot publish a handoff", t.snapshot() == null);

        h = new FakeHardware(c);
        h.badVoltage = true;
        t = new TurretController2026(Mode.TRACKING, c, h, 0);
        t.start(0, false);
        t.update(7);
        check("missing analog input times out", t.getState() == State.FAULT);

        Config invalid = new Config();
        invalid.encoderRatio = 0;
        t = new TurretController2026(Mode.TRACKING, invalid, h, 0);
        t.start(0, false);
        check("invalid geometry faults before output", t.getState() == State.FAULT);

        Config live = new Config();
        h = new FakeHardware(live);
        t = new TurretController2026(Mode.TRACKING, live, h, 0);
        live.rateLimit = Double.NaN;
        t.start(1, false);
        check("bad live setting at Play stays faulted", t.getState() == State.FAULT);
        check("bad live setting at Play cannot write", h.writes == 0);
    }

    private static TurretController2026 calibrated(Config c, FakeHardware h) {
        TurretController2026 t = new TurretController2026(Mode.TRACKING, c, h, 0);
        t.start(0, true);
        t.update(.1);
        t.update(.2);
        t.update(2.1);
        return t;
    }

    private static final class FakeHardware implements TurretController2026.Hardware {
        final Config config;
        double angle, left, right;
        int origin, writes;
        boolean encodersForbidden, badVoltage;
        FakeHardware(Config config) { this.config = config; }
        int ticks() { return origin + (int) Math.round(angle * config.countsPerDegree() / config.quadSign); }
        public int encoderTicks() {
            if (encodersForbidden) throw new AssertionError("FIXED mode read quadrature encoder");
            return ticks();
        }
        public double absoluteVoltage() {
            if (encodersForbidden) throw new AssertionError("FIXED mode read analog encoder");
            if (badVoltage) return Double.NaN;
            double raw = (angle - config.absoluteOffset) * config.encoderRatio / config.absoluteSign;
            raw = (raw % 360 + 360) % 360;
            return raw / 360 * config.absoluteMaxVoltage;
        }
        public void writeServos(double l, double r) { left = l; right = r; writes++; }
    }

    private static void close(String label, double expected, double actual) { near(label, expected, actual, 1e-8); }
    private static void near(String label, double expected, double actual, double tolerance) {
        check(label + " (expected " + expected + ", got " + actual + ")",
                Double.isFinite(actual) && Math.abs(expected - actual) <= tolerance);
    }
    private static void check(String label, boolean passed) {
        checks++;
        if (!passed) throw new AssertionError(label);
    }
}
