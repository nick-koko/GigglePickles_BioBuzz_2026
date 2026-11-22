package org.firstinspires.ftc.teamcode.testing.shots;

import java.lang.reflect.Field;
import java.util.*;
import org.firstinspires.ftc.teamcode.subsystems.*;

/** Immutable, explicitly named comparisons; never a blind Cartesian product. */
public final class ShotExperimentPlan2026 {
    public enum ComparisonSet { STARTUP, VOLTAGE, BOOST, FEED_POWER, CUSTOM_VS_CURRENT }
    public enum VerificationProfile { CURRENT_SETTINGS, MODELED_CANDIDATE, CUSTOM }
    public enum Condition { COLD, ALREADY_SPINNING, STEP_UP, STEP_DOWN }
    public static final class Profile {
        public final String id;
        final Map<Field,Object> overrides;
        private Profile(String id,Map<Field,Object> values) {
            this.id=id;overrides=Collections.unmodifiableMap(new LinkedHashMap<>(values));
        }
        Profile with(String id,Class<?> type,String name,Object value) {
            Map<Field,Object> values=new LinkedHashMap<>(overrides);
            values.put(ScopedShotSettings2026.field(type,name),value);
            return new Profile(id,values);
        }
        Profile shooter(String id,String name,Object value) { return with(id,ShooterSubsystem2026.class,name,value); }
        Profile shot(String id,String name,Object value) { return with(id,ShotSequenceController2026.class,name,value); }
        Profile intake(String id,String name,Object value) { return with(id,IntakeWithSensorsSubsystem2026.class,name,value); }
    }
    public static final class Trial {
        public final int id,repetition;
        public final Profile profile;
        public final Condition condition;
        public final double targetRpm,initialTargetRpm,rampSeconds,feedDelaySeconds;
        public final boolean feedEnabled,recordCoast;
        Trial(int id,int repetition,Profile profile,Condition condition) {
            this(id,repetition,profile,condition,ShotExperimentSettings2026.TARGET_RPM,
                    ShotExperimentSettings2026.STEP_DELTA_RPM,ShotExperimentSettings2026.TARGET_RAMP_SECONDS,
                    condition==Condition.COLD?ShotExperimentSettings2026.COLD_FEED_DELAY_SECONDS:
                        ShotExperimentSettings2026.RUNNING_FEED_DELAY_SECONDS,true,false);
        }
        Trial(int id,int repetition,Profile profile,Condition condition,double target,double step,double ramp,
              double delay,boolean feedEnabled,boolean recordCoast) {
            this.id=id;this.repetition=repetition;this.profile=profile;this.condition=condition;
            targetRpm=target;this.feedEnabled=feedEnabled;this.recordCoast=recordCoast;
            initialTargetRpm=condition==Condition.COLD?0:condition==Condition.STEP_UP?
                    targetRpm-step:condition==Condition.STEP_DOWN?targetRpm+step:targetRpm;
            rampSeconds=condition==Condition.STEP_UP||condition==Condition.STEP_DOWN?ramp:0;
            feedDelaySeconds=delay;
            if(!Double.isFinite(target)||target<=0||target>6000||!Double.isFinite(step)||step<0
                    ||!Double.isFinite(ramp)||ramp<0||!Double.isFinite(delay)||delay<rampSeconds
                    ||(feedEnabled&&recordCoast))throw new IllegalArgumentException("Invalid shot/model trial target, timing or coast request");
            if (initialTargetRpm<0 || initialTargetRpm>6000 || (condition!=Condition.COLD && initialTargetRpm==0))
                throw new IllegalArgumentException("Shot test initial RPM must be positive and <=6000 for a spinning condition");
        }
    }
    public final List<Trial> trials;
    public final double prepareSeconds,recoverySeconds,stoppedRpm,stoppedStableSeconds,coastTimeoutSeconds,unloadedHoldSeconds;
    public final double unloadedBetweenSeconds;
    public final String suite,runLabel,batteryCondition;
    public final boolean detailedLogging;
    private ShotExperimentPlan2026(List<Trial> trials,boolean comparison) {
        this(trials,comparison?"COMPARE_"+ShotExperimentSettings2026.COMPARISON_SET:
                "VERIFY_"+ShotExperimentSettings2026.VERIFICATION_PROFILE,
                ShotExperimentSettings2026.RUNNING_PREPARE_SECONDS,ShotExperimentSettings2026.POST_FEED_CAPTURE_SECONDS,0);
    }
    ShotExperimentPlan2026(List<Trial> trials,String suite,double prepare,double recovery,double unloadedHold) {
        this.trials=Collections.unmodifiableList(new ArrayList<>(trials));
        prepareSeconds=prepare;recoverySeconds=recovery;unloadedHoldSeconds=unloadedHold;
        unloadedBetweenSeconds=ShotExperimentSettings2026.MODEL_BETWEEN_TRIAL_SECONDS;
        stoppedRpm=ShotExperimentSettings2026.STOPPED_RPM_THRESHOLD;
        stoppedStableSeconds=ShotExperimentSettings2026.STOPPED_STABLE_SECONDS;
        coastTimeoutSeconds=ShotExperimentSettings2026.COAST_TIMEOUT_SECONDS;
        this.suite=suite;
        runLabel=ShotExperimentSettings2026.RUN_LABEL;batteryCondition=ShotExperimentSettings2026.BATTERY_CONDITION;
        detailedLogging=ShotExperimentSettings2026.ENABLE_DETAILED_TEST_LOGGING;
    }

    public static ShotExperimentPlan2026 build(boolean comparison,ScopedShotSettings2026 baseline) {
        validateSettings();
        List<Profile> profiles=comparison?comparisons(baseline):Collections.singletonList(verification());
        if(ShotExperimentSettings2026.TEST_FEED_SECONDS>0) {
            List<Profile> timed=new ArrayList<>();
            for(Profile p:profiles)timed.add(p.shot(p.id,"FEED_SECONDS",ShotExperimentSettings2026.TEST_FEED_SECONDS));
            profiles=timed;
        }
        List<Condition> conditions=new ArrayList<>();
        if (comparison?ShotExperimentSettings2026.COMPARE_COLD:ShotExperimentSettings2026.VERIFY_COLD)conditions.add(Condition.COLD);
        if (comparison?ShotExperimentSettings2026.COMPARE_ALREADY_SPINNING:ShotExperimentSettings2026.VERIFY_ALREADY_SPINNING)conditions.add(Condition.ALREADY_SPINNING);
        if (!comparison && ShotExperimentSettings2026.VERIFY_STEP_UP)conditions.add(Condition.STEP_UP);
        if (!comparison && ShotExperimentSettings2026.VERIFY_STEP_DOWN)conditions.add(Condition.STEP_DOWN);
        if (conditions.isEmpty())throw new IllegalArgumentException("Enable at least one shot test starting condition");
        int repetitions=comparison?ShotExperimentSettings2026.COMPARISON_REPETITIONS:ShotExperimentSettings2026.VERIFICATION_REPETITIONS;
        List<Trial> trials=new ArrayList<>();
        for (int repeat=0;repeat<repetitions;repeat++) {
            for (int p=0;p<profiles.size();p++) {
                Profile profile=profiles.get(Math.floorMod(p+repeat+ShotExperimentSettings2026.PROFILE_ORDER_OFFSET,profiles.size()));
                for (int c=0;c<conditions.size();c++) {
                    Condition condition=conditions.get(comparison?c:(c+repeat)%conditions.size());
                    trials.add(new Trial(trials.size()+1,repeat+1,profile,condition));
                }
            }
        }
        try {
            for (Profile profile:profiles)baseline.apply(profile); // No motor commands; Init validation only.
        } finally { baseline.resetToBaseline(); }
        return new ShotExperimentPlan2026(trials,comparison);
    }

    static Profile base() { return new Profile("CURRENT",Collections.emptyMap()); }
    private static Profile startup(Profile b,String name,double up,double down) {
        return b.shooter(name,"ENABLE_STARTUP_PROFILE",true).shooter(name,"STARTUP_SLEW_UP_PER_SECOND",up)
                .shooter(name,"STARTUP_SLEW_DOWN_PER_SECOND",down);
    }
    private static Profile voltage(Profile b,String name) {
        return b.shooter(name,"ENABLE_VOLTAGE_COMPENSATION",true)
                .shooter(name,"VOLTAGE_COMP_MIN_GAIN",.90).shooter(name,"VOLTAGE_COMP_MAX_GAIN",1.10);
    }
    private static Profile modelCandidate() {
        return RecommendedShotPlans2026.referenceProfile();
    }
    private static Profile verification() {
        switch (ShotExperimentSettings2026.VERIFICATION_PROFILE) {
            case CURRENT_SETTINGS:return base();
            case MODELED_CANDIDATE:return modelCandidate();
            case CUSTOM:return custom();
            default:throw new IllegalArgumentException("Unknown verification profile");
        }
    }
    private static List<Profile> comparisons(ScopedShotSettings2026 b) {
        Profile base=base();
        switch (ShotExperimentSettings2026.COMPARISON_SET) {
            case STARTUP:return Arrays.asList(base,startup(base,"START_DOWN6",1.5,6),
                    startup(base,"START6_6",6,6),startup(base,"START10_6",10,6));
            case VOLTAGE:return Arrays.asList(base,voltage(base,"FF_VOLTAGE"),
                    base.shot("BOOST_VOLTAGE","ENABLE_TIMED_BOOST",true).shot("BOOST_VOLTAGE","ENABLE_BOOST_VOLTAGE_SCALING",true),
                    voltage(base,"FF_AND_BOOST_VOLTAGE").shot("FF_AND_BOOST_VOLTAGE","ENABLE_TIMED_BOOST",true)
                            .shot("FF_AND_BOOST_VOLTAGE","ENABLE_BOOST_VOLTAGE_SCALING",true));
            case BOOST:return Arrays.asList(base,base.shot("NO_BOOST","ENABLE_TIMED_BOOST",false),
                    base.shot("EARLY_FLAT","ENABLE_TIMED_BOOST",true).shot("EARLY_FLAT","ENABLE_RAMPED_BOOST",false)
                            .shot("EARLY_FLAT","BOOST_DELAY_SECONDS",.04),
                    base.shot("EARLY_RAMP_TAPER","ENABLE_TIMED_BOOST",true)
                            .shot("EARLY_RAMP_TAPER","ENABLE_RAMPED_BOOST",true)
                            .shot("EARLY_RAMP_TAPER","BOOST_DELAY_SECONDS",.04)
                            .shot("EARLY_RAMP_TAPER","BOOST_START_POWER",.80)
                            .shot("EARLY_RAMP_TAPER","BOOST_END_POWER",.90)
                            .shot("EARLY_RAMP_TAPER","BOOST_OVERSPEED_TAPER_RPM",100.0));
            case FEED_POWER:
                double intake=b.originalDouble(IntakeWithSensorsSubsystem2026.class,"SHOOT_INTAKE_POWER");
                double transfer=b.originalDouble(IntakeWithSensorsSubsystem2026.class,"SHOOT_TRANSFER_POWER");
                double mild=ShotExperimentSettings2026.FEED_MILD_SCALE,slow=ShotExperimentSettings2026.FEED_SLOW_SCALE;
                return Arrays.asList(base,base.intake("TRANSFER_MILD","SHOOT_TRANSFER_POWER",transfer*mild),
                        base.intake("INTAKE_MILD","SHOOT_INTAKE_POWER",intake*mild),
                        base.intake("BOTH_MILD","SHOOT_INTAKE_POWER",intake*mild).intake("BOTH_MILD","SHOOT_TRANSFER_POWER",transfer*mild),
                        base.intake("BOTH_SLOW","SHOOT_INTAKE_POWER",intake*slow).intake("BOTH_SLOW","SHOOT_TRANSFER_POWER",transfer*slow));
            case CUSTOM_VS_CURRENT:return Arrays.asList(base,custom());
            default:throw new IllegalArgumentException("Unknown comparison set");
        }
    }
    private static Profile custom() {
        String id="CUSTOM";Profile p=base();
        String[] gains={"kP","kI","kD","kS","kV","kA"};
        double[] values={ShotExperimentSettings2026.CUSTOM_KP,ShotExperimentSettings2026.CUSTOM_KI,ShotExperimentSettings2026.CUSTOM_KD,
                ShotExperimentSettings2026.CUSTOM_KS,ShotExperimentSettings2026.CUSTOM_KV,ShotExperimentSettings2026.CUSTOM_KA};
        for(int i=0;i<gains.length;i++) {
            if(!Double.isFinite(values[i])||values[i]<0)throw new IllegalArgumentException("Custom PIDF gains must be finite and nonnegative");
            p=p.shooter(id,gains[i],values[i]);
        }
        return p.shooter(id,"ENABLE_STARTUP_PROFILE",ShotExperimentSettings2026.CUSTOM_STARTUP_PROFILE)
                .shooter(id,"STARTUP_SLEW_UP_PER_SECOND",ShotExperimentSettings2026.CUSTOM_STARTUP_UP_PER_S)
                .shooter(id,"STARTUP_SLEW_DOWN_PER_SECOND",ShotExperimentSettings2026.CUSTOM_STARTUP_DOWN_PER_S)
                .shooter(id,"ENABLE_ASYMMETRIC_SLEW",ShotExperimentSettings2026.CUSTOM_ASYMMETRIC_RUNNING)
                .shooter(id,"RUNNING_SLEW_UP_PER_SECOND",ShotExperimentSettings2026.CUSTOM_RUNNING_UP_PER_S)
                .shooter(id,"RUNNING_SLEW_DOWN_PER_SECOND",ShotExperimentSettings2026.CUSTOM_RUNNING_DOWN_PER_S)
                .shooter(id,"VOLTAGE_COMP_MIN_GAIN",ShotExperimentSettings2026.CUSTOM_FF_MIN_GAIN)
                .shooter(id,"VOLTAGE_COMP_MAX_GAIN",ShotExperimentSettings2026.CUSTOM_FF_MAX_GAIN)
                .shot(id,"ENABLE_TIMED_BOOST",ShotExperimentSettings2026.CUSTOM_BOOST_ENABLED)
                .shot(id,"ENABLE_RAMPED_BOOST",ShotExperimentSettings2026.CUSTOM_BOOST_RAMP)
                .shot(id,"ENABLE_BOOST_VOLTAGE_SCALING",ShotExperimentSettings2026.CUSTOM_BOOST_VOLTAGE_SCALING)
                .shot(id,"BOOST_BYPASSES_SLEW",ShotExperimentSettings2026.CUSTOM_BOOST_BYPASSES_SLEW)
                .shot(id,"BOOST_DELAY_SECONDS",ShotExperimentSettings2026.CUSTOM_BOOST_DELAY_SECONDS)
                .shot(id,"BOOST_DURATION_SECONDS",ShotExperimentSettings2026.CUSTOM_BOOST_DURATION_SECONDS)
                .shot(id,"BOOST_MINIMUM_POWER",ShotExperimentSettings2026.CUSTOM_BOOST_FLAT_POWER)
                .shot(id,"BOOST_START_POWER",ShotExperimentSettings2026.CUSTOM_BOOST_START_POWER)
                .shot(id,"BOOST_END_POWER",ShotExperimentSettings2026.CUSTOM_BOOST_END_POWER)
                .shot(id,"BOOST_NOMINAL_V",ShotExperimentSettings2026.CUSTOM_BOOST_NOMINAL_V)
                .shot(id,"BOOST_OVERSPEED_TAPER_RPM",ShotExperimentSettings2026.CUSTOM_BOOST_TAPER_RPM)
                .shot(id,"FEED_SECONDS",ShotExperimentSettings2026.CUSTOM_FEED_SECONDS)
                .intake(id,"SHOOT_INTAKE_POWER",ShotExperimentSettings2026.CUSTOM_FEED_INTAKE_POWER)
                .intake(id,"SHOOT_TRANSFER_POWER",ShotExperimentSettings2026.CUSTOM_FEED_TRANSFER_POWER);
    }
    static void validateSettings() {
        if (ShotExperimentSettings2026.COMPARISON_SET==null || ShotExperimentSettings2026.VERIFICATION_PROFILE==null)
            throw new IllegalArgumentException("Select shot experiment profiles");
        if(ShotExperimentSettings2026.FLYWHEEL_WEIGHT_COUNT<0||!Double.isFinite(ShotExperimentSettings2026.FLYWHEEL_DIAMETER_MM)
                ||ShotExperimentSettings2026.FLYWHEEL_DIAMETER_MM<=0)
            throw new IllegalArgumentException("Flywheel metadata needs a nonnegative weight count and positive diameter");
        if (ShotExperimentSettings2026.COMPARISON_REPETITIONS<1 || ShotExperimentSettings2026.COMPARISON_REPETITIONS>20
                || ShotExperimentSettings2026.VERIFICATION_REPETITIONS<1 || ShotExperimentSettings2026.VERIFICATION_REPETITIONS>20)
            throw new IllegalArgumentException("Shot test repetitions must be 1..20");
        double[] nonnegative={ShotExperimentSettings2026.STEP_DELTA_RPM,ShotExperimentSettings2026.TARGET_RAMP_SECONDS,
                ShotExperimentSettings2026.COLD_FEED_DELAY_SECONDS,ShotExperimentSettings2026.RUNNING_FEED_DELAY_SECONDS,
                ShotExperimentSettings2026.RUNNING_PREPARE_SECONDS,ShotExperimentSettings2026.POST_FEED_CAPTURE_SECONDS,
                ShotExperimentSettings2026.STOPPED_STABLE_SECONDS,ShotExperimentSettings2026.TEST_FEED_SECONDS};
        for(double v:nonnegative)if(!Double.isFinite(v)||v<0)throw new IllegalArgumentException("Shot test timings/step must be finite and nonnegative");
        if(!Double.isFinite(ShotExperimentSettings2026.TARGET_RPM)||ShotExperimentSettings2026.TARGET_RPM<=0||ShotExperimentSettings2026.TARGET_RPM>6000
                || !Double.isFinite(ShotExperimentSettings2026.STOPPED_RPM_THRESHOLD)||ShotExperimentSettings2026.STOPPED_RPM_THRESHOLD<0||ShotExperimentSettings2026.STOPPED_RPM_THRESHOLD>200
                || !Double.isFinite(ShotExperimentSettings2026.COAST_TIMEOUT_SECONDS)||ShotExperimentSettings2026.COAST_TIMEOUT_SECONDS<=0)
            throw new IllegalArgumentException("Invalid shot test target, stopped-RPM threshold or coast timeout");
        if(ShotExperimentSettings2026.RUNNING_FEED_DELAY_SECONDS<ShotExperimentSettings2026.TARGET_RAMP_SECONDS)
            throw new IllegalArgumentException("Running feed delay must be at least the target ramp duration");
        for(double scale:new double[]{ShotExperimentSettings2026.FEED_MILD_SCALE,ShotExperimentSettings2026.FEED_SLOW_SCALE})
            if(!Double.isFinite(scale)||scale<=0||scale>1)throw new IllegalArgumentException("Feed comparison scales must be >0 and <=1");
        if(ShotExperimentSettings2026.RUN_LABEL==null||ShotExperimentSettings2026.BATTERY_CONDITION==null)
            throw new IllegalArgumentException("Shot test run/battery labels must be specified");
        double[] pose={ShotExperimentSettings2026.TEST_X_INCHES,ShotExperimentSettings2026.TEST_Y_INCHES,ShotExperimentSettings2026.TEST_HEADING_DEGREES};
        int known=0;for(double v:pose) {
            if(Double.isFinite(v))known++;
            else if(!Double.isNaN(v))throw new IllegalArgumentException("Manual test pose must be finite or NaN (unknown)");
        }
        if(known!=0&&known!=3)throw new IllegalArgumentException("Specify all three manual test pose values, or leave all unknown");
        if(Double.isFinite(ShotExperimentSettings2026.AIM_X_INCHES)!=Double.isFinite(ShotExperimentSettings2026.AIM_Y_INCHES))
            throw new IllegalArgumentException("Specify both manual aim coordinates, or leave both unknown");
        if(Double.isInfinite(ShotExperimentSettings2026.AIM_X_INCHES)||Double.isInfinite(ShotExperimentSettings2026.AIM_Y_INCHES))
            throw new IllegalArgumentException("Manual aim coordinates must be finite or NaN (unknown)");
    }

    public String manifest() {
        StringBuilder s=new StringBuilder(suite).append('\n');
        s.append("prepare_s=").append(prepareSeconds).append(" recovery_s=").append(recoverySeconds)
                .append(" unloaded_hold_s=").append(unloadedHoldSeconds).append(" unloaded_between_s=").append(unloadedBetweenSeconds)
                .append(" stopped_rpm=").append(stoppedRpm)
                .append(" stopped_stable_s=").append(stoppedStableSeconds).append(" coast_timeout_s=").append(coastTimeoutSeconds)
                .append(" run_label=").append(runLabel).append(" battery_label=").append(batteryCondition).append('\n');
        Set<Profile> profiles=new LinkedHashSet<>();
        for(Trial t:trials) {
            profiles.add(t.profile);
            s.append(t.id).append(" ").append(t.profile.id).append(" ").append(t.condition)
                    .append(" initial=").append(t.initialTargetRpm).append(" final=").append(t.targetRpm)
                    .append(" ramp_s=").append(t.rampSeconds).append(" delay_s=").append(t.feedDelaySeconds)
                    .append(" feed=").append(t.feedEnabled).append(" coast_capture=").append(t.recordCoast).append('\n');
        }
        for(Profile p:profiles) {
            s.append("PROFILE ").append(p.id).append('\n');
            for(Map.Entry<Field,Object> e:p.overrides.entrySet())s.append(e.getKey().getDeclaringClass().getSimpleName())
                    .append('.').append(e.getKey().getName()).append('=').append(e.getValue()).append('\n');
        }
        return s.toString();
    }
}
