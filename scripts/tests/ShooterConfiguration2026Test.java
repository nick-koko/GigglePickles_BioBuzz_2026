import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.Random;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem2026;
import org.firstinspires.ftc.teamcode.subsystems.ShotSequenceController2026;
import org.firstinspires.ftc.teamcode.util.ShooterPowerProfile2026;

/** Production controller checks with prescribed samples and a motor command recorder. */
public final class ShooterConfiguration2026Test {
    private static int checks;
    private static final ShooterSubsystem2026 shooter = ShooterSubsystem2026.INSTANCE;
    private static Method control;
    private static long now = 1_000_000_000L;

    public static void main(String[] args) throws Exception {
        control = ShooterSubsystem2026.class.getDeclaredMethod("updateController", long.class, double.class, double.class);
        control.setAccessible(true);
        double[] power = {0};
        DcMotorEx motor = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[]{DcMotorEx.class}, (proxy, method, values) -> {
                    if (method.getName().equals("setPower")) { power[0]=(double)values[0]; return null; }
                    if (method.getName().equals("getPower")) return power[0];
                    throw new AssertionError("Unexpected hardware call: " + method.getName());
                });
        put("shooterMotor", motor);
        defaults();
        legacyOutputParity();
        check("recorded hardware command matches cached output", power[0] == shooter.getAppliedPower());
        asymmetricSlew();
        startupHandoffAndReenable();
        shapedBoost();
        sequenceSnapshotAndFlatDefault();
        invalidConfiguration();
        System.out.println("Passed " + checks + " shooter configuration checks (including 2000 legacy-output comparisons).");
    }

    private static void defaults() {
        check("tested startup and asymmetric defaults enabled", ShooterSubsystem2026.ENABLE_STARTUP_PROFILE
                && ShooterSubsystem2026.ENABLE_ASYMMETRIC_SLEW);
        check("running slew is 1.5 and startup slew is 6", ShooterSubsystem2026.SLEW_PER_SECOND==1.5
                && ShooterSubsystem2026.RUNNING_SLEW_UP_PER_SECOND==1.5
                && ShooterSubsystem2026.RUNNING_SLEW_DOWN_PER_SECOND==1.5
                && ShooterSubsystem2026.STARTUP_SLEW_UP_PER_SECOND==6
                && ShooterSubsystem2026.STARTUP_SLEW_DOWN_PER_SECOND==6);
        check("tested compensation defaults", ShooterSubsystem2026.VOLTAGE_COMP_MIN_GAIN==.90
                && ShooterSubsystem2026.VOLTAGE_COMP_MAX_GAIN==1.10);
        check("candidate boost shaping enabled", ShotSequenceController2026.ENABLE_RAMPED_BOOST
                && ShotSequenceController2026.ENABLE_BOOST_VOLTAGE_SCALING
                && ShotSequenceController2026.BOOST_START_POWER==.80
                && ShotSequenceController2026.BOOST_END_POWER==.90
                && ShotSequenceController2026.BOOST_NOMINAL_V==12.5
                && ShotSequenceController2026.BOOST_OVERSPEED_TAPER_RPM==100);
        check("candidate feed/boost timing", ShotSequenceController2026.FEED_SECONDS==1.0
                && ShotSequenceController2026.BOOST_DELAY_SECONDS==.04
                && ShotSequenceController2026.BOOST_DURATION_SECONDS==.5
                && ShotSequenceController2026.BOOST_MINIMUM_POWER==.75
                && ShotSequenceController2026.BOOST_BYPASSES_SLEW);
    }

    private static void legacyOutputParity() throws Exception {
        ShooterSubsystem2026.ENABLE_STARTUP_PROFILE=false;
        ShooterSubsystem2026.ENABLE_ASYMMETRIC_SLEW=false;
        ShooterSubsystem2026.VOLTAGE_COMP_MIN_GAIN=1;
        ShooterSubsystem2026.VOLTAGE_COMP_MAX_GAIN=1.05;
        Random r = new Random(20261119);
        double previous = 0;
        shooter.stop();
        // Independent oracle: the meet's P+FF, capped voltage compensation,
        // optional flat boost and symmetric slew; all shipped I/D/kA are zero.
        for (int i=0;i<2000;i++) {
            if (i%250==0) { shooter.stop(); previous=0; }
            double dt=.015+r.nextDouble()*.055, volts=11+r.nextDouble()*3;
            double target=2800+r.nextDouble()*850, rpm=r.nextDouble()*4200;
            boolean boost=i%90>=15 && i%90<31;
            boolean bypass=i%180<90;
            shooter.spinUp(target);
            if (boost) shooter.setMinimumPowerOverride(.75,bypass); else shooter.clearMinimumPowerOverride();
            double ff=(.09+.000134*target)*clip(12.5/volts,1,1.05);
            double requested=clip(.0009*(target-rpm)+ff,0,1);
            double without=clip(requested,previous-1.5*dt,previous+1.5*dt);
            double wanted=boost?Math.max(requested,.75):requested;
            double expected=boost&&bypass?wanted:clip(wanted,previous-1.5*dt,previous+1.5*dt);
            sample(rpm,volts,dt);
            near("legacy applied power step "+i,shooter.getAppliedPower(),expected,1e-14);
            near("legacy request",shooter.getOutputBeforeSlew(),wanted,1e-14);
            near("same-loop no-boost diagnostic",shooter.getOutputWithoutBoostThisLoop(),without,1e-14);
            previous=expected;
        }
    }

    private static void asymmetricSlew() throws Exception {
        ShooterSubsystem2026.ENABLE_ASYMMETRIC_SLEW=true;
        ShooterSubsystem2026.RUNNING_SLEW_UP_PER_SECOND=2;
        ShooterSubsystem2026.RUNNING_SLEW_DOWN_PER_SECOND=6;
        shooter.stop(); shooter.spinUp(3200);
        sample(0,12.5,.05);
        near("asymmetric up",shooter.getAppliedPower(),.1,1e-12);
        put("lastAppliedPower",.8);
        sample(4500,12.5,.05);
        near("asymmetric down",shooter.getAppliedPower(),.5,1e-12);
        near("asymmetric no boost uses same limits",shooter.getOutputWithoutBoostThisLoop(),.5,1e-12);
        ShooterSubsystem2026.ENABLE_ASYMMETRIC_SLEW=false;
        ShooterSubsystem2026.RUNNING_SLEW_UP_PER_SECOND=1.5;
        ShooterSubsystem2026.RUNNING_SLEW_DOWN_PER_SECOND=1.5;
    }

    private static void startupHandoffAndReenable() throws Exception {
        ShooterSubsystem2026.ENABLE_STARTUP_PROFILE=true;
        ShooterSubsystem2026.STARTUP_SLEW_UP_PER_SECOND=6;
        ShooterSubsystem2026.STARTUP_SLEW_DOWN_PER_SECOND=6;
        shooter.stop(); shooter.spinUp(3200);
        sample(0,13.8,.033);
        check("cold enable arms startup",shooter.getPowerProfileState().equals("STARTUP"));
        near("startup upward rate",shooter.getAppliedPower(),6*.033,1e-12);
        shooter.spinUp(3300); sample(1000,13.8,.033);
        check("target update preserves startup clock",shooter.getStartupElapsedSeconds()>0);
        shooter.setShotFeeding(true); sample(2000,13.8,.033);
        check("feed ends startup",shooter.getPowerProfileState().equals("RUNNING")
                && shooter.getStartupExitReason().equals("FEED_STARTED"));
        shooter.setShotFeeding(false); sample(50,13.8,.033);
        check("later sag does not rearm startup",shooter.getPowerProfileState().equals("RUNNING"));
        shooter.stop(); shooter.spinUp(3200); sample(500,12.5,.033);
        check("fresh read protects warm reenable despite cleared cached RPM",shooter.getStartupExitReason().equals("ALREADY_SPINNING"));
        near("warm reenable uses running limit",shooter.getAppliedPower(),1.5*.033,1e-12);
        shooter.stop(); shooter.spinUp(3200); shooter.setShotFeeding(true); sample(0,12.5,.033);
        check("feed before first controller sample prevents startup",shooter.getStartupExitReason().equals("FEED_STARTED"));
        shooter.stop(); shooter.spinUp(3200); sample(0,12.5,.033);
        now+=2_100_000_000L; sample(500,12.5,.033);
        check("startup timeout never gates a feed",shooter.getStartupExitReason().equals("TIMEOUT"));
        shooter.stop(); shooter.spinUp(3200); sample(0,12.5,.033);
        for (int i=0;i<10;i++) { shooter.spinUp(3200); sample(3200,12.5,.033); }
        check("stable handoff",shooter.getStartupExitReason().equals("SETTLED"));
        double before=shooter.getAppliedPower(); sample(3500,12.5,.033);
        check("handoff preserves applied-power slew history",Math.abs(before-shooter.getAppliedPower())<=1.5*.033+1e-12);
        shooter.stop(); check("stop resets profile",shooter.getPowerProfileState().equals("OFF"));
        ShooterSubsystem2026.ENABLE_STARTUP_PROFILE=false;
        ShooterSubsystem2026.STARTUP_SLEW_UP_PER_SECOND=ShooterSubsystem2026.STARTUP_SLEW_DOWN_PER_SECOND=1.5;
    }

    private static void shapedBoost() throws Exception {
        shooter.stop(); shooter.spinUp(3200);
        for (double v:new double[]{13.8,12.5,11.4}) {
            shooter.setMinimumPowerOverride(.75,true,true,12.5,0);
            sample(3200,v,.033);
            near("scaled boost preserves nominal voltage",shooter.getAppliedPower()*v,.75*12.5,1e-12);
        }
        shooter.setMinimumPowerOverride(.95,true,true,12.5,0); sample(3200,11.4,.033);
        near("scaled boost clips at full power",shooter.getAppliedPower(),1,1e-12);
        shooter.setMinimumPowerOverride(.75,true,false,12.5,100); sample(3300,12.5,.033);
        near("overspeed removes added floor only",shooter.getAppliedPower(),shooter.getControllerOutputBeforeBoost(),1e-12);
        shooter.setMinimumPowerOverride(.75,true); sample(3300,12.5,.033);
        near("legacy API clears shaping flags",shooter.getAppliedPower(),.75,1e-12);
        near("half overspeed multiplier",ShooterPowerProfile2026.overspeedMultiplier(3250,3200,100),.5,1e-12);
        shooter.clearMinimumPowerOverride(); sample(3200,12.5,.033);
        check("clear removes cached boost floor",shooter.getSampleOverrideFloor()==0 && !shooter.didBoostBypassSlew());
    }

    private static void sequenceSnapshotAndFlatDefault() throws Exception {
        // Explicit legacy settings exercise flat-floor support after candidate adoption.
        ShotSequenceController2026.ENABLE_RAMPED_BOOST=false;
        ShotSequenceController2026.ENABLE_BOOST_VOLTAGE_SCALING=false;
        ShotSequenceController2026.BOOST_DELAY_SECONDS=.18;
        ShotSequenceController2026.BOOST_OVERSPEED_TAPER_RPM=0;
        ShotSequenceController2026 seq=new ShotSequenceController2026();
        seq.reset(); seq.start(); setElapsed(seq,.4); seq.update();
        near("default sequence floor",seq.getRequestedBoostFloor(),.75,1e-12);
        ShotSequenceController2026.ENABLE_RAMPED_BOOST=true;
        ShotSequenceController2026.BOOST_START_POWER=.2;
        ShotSequenceController2026.BOOST_END_POWER=.8;
        ShotSequenceController2026.ENABLE_BOOST_VOLTAGE_SCALING=true;
        setElapsed(seq,.4); seq.update();
        near("live edits do not change active flat volley",seq.getRequestedBoostFloor(),.75,1e-12);
        check("active voltage choice is snapshotted",!seq.isActiveBoostVoltageScaling());
        seq.cancel(); seq.start(); setElapsed(seq,.43); seq.update();
        near("ramped floor midpoint",seq.getRequestedBoostFloor(),.5,.002);
        check("new volley uses edited voltage choice",seq.isActiveBoostVoltageScaling());
        seq.cancel(); check("cancel clears requested boost",seq.getRequestedBoostFloor()==0);
        ShotSequenceController2026.ENABLE_RAMPED_BOOST=false;
        ShotSequenceController2026.BOOST_START_POWER=ShotSequenceController2026.BOOST_END_POWER=.75;
        ShotSequenceController2026.ENABLE_BOOST_VOLTAGE_SCALING=false;
        seq.reset(); seq.start(); setElapsed(seq,2); seq.update();
        check("feed timer remains independent of RPM",!seq.isActive() && seq.getEndReason().equals("FEED_TIMER_COMPLETE"));
    }

    private static void invalidConfiguration() {
        ShooterSubsystem2026.STARTUP_SLEW_UP_PER_SECOND=Double.NaN;
        boolean failed=false;
        try { ShooterSubsystem2026.validateConfiguration(); } catch (IllegalArgumentException e) { failed=true; }
        check("invalid startup config fails clearly",failed);
        ShooterSubsystem2026.STARTUP_SLEW_UP_PER_SECOND=1.5;
        ShotSequenceController2026.BOOST_END_POWER=1.2; failed=false;
        try { new ShotSequenceController2026().reset(); } catch (IllegalArgumentException e) { failed=true; }
        check("sequence reset validates during Init",failed);
        ShotSequenceController2026.BOOST_END_POWER=.75;
    }

    private static void setElapsed(ShotSequenceController2026 seq,double elapsed) throws Exception {
        Field f=seq.getClass().getDeclaredField("sequenceStartNanos"); f.setAccessible(true);
        f.setLong(seq,System.nanoTime()-(long)(elapsed*1e9));
    }
    private static void sample(double rpm,double volts,double dt) throws Exception {
        now+=(long)(dt*1e9);
        put("lastBatteryVoltageRaw",volts); put("lastBatteryVoltageFiltered",volts);
        put("lastMeasuredFlywheelRpm",rpm);
        control.invoke(shooter,now,rpm,dt);
    }
    private static void put(String name,Object value) throws Exception {
        Field f=ShooterSubsystem2026.class.getDeclaredField(name); f.setAccessible(true); f.set(shooter,value);
    }
    private static double clip(double v,double lo,double hi) { return Math.max(lo,Math.min(hi,v)); }
    private static void near(String label,double a,double b,double tolerance) { check(label+": "+a+" vs "+b,Math.abs(a-b)<=tolerance); }
    private static void check(String label,boolean condition) { checks++; if(!condition)throw new AssertionError(label); }
}
