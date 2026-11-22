package org.firstinspires.ftc.teamcode.testing.shots;

import java.util.*;
import org.firstinspires.ftc.teamcode.testing.shots.ShotExperimentPlan2026.*;

/** Small hypothesis-driven batches. Values here apply only inside an experiment scope. */
public final class RecommendedShotPlans2026 {
    private RecommendedShotPlans2026() { }
    public enum Batch { QUICK_SCREEN, STARTUP_NEIGHBORS, BOOST_LEVEL, BOOST_TIMING, BOOST_TAPER,
        RUNNING_NEIGHBORS, LARGE_TRANSITIONS, FEED_NEIGHBORS, LOADED_RPM_BRACKET, P_GAIN, FEEDFORWARD_GAIN }
    public enum ModelBatch { STARTUP_AND_COAST, STEP_RESPONSE }

    /** Explicit reference, rather than accidentally inheriting future production tuning changes. */
    public static Profile referenceProfile() {
        String id="MODEL_CANDIDATE";
        Profile p=ShotExperimentPlan2026.base();
        String[] names={"kP","kI","kD","kS","kV","kA","SLEW_PER_SECOND",
                "RUNNING_SLEW_UP_PER_SECOND","RUNNING_SLEW_DOWN_PER_SECOND",
                "STARTUP_SLEW_UP_PER_SECOND","STARTUP_SLEW_DOWN_PER_SECOND","STARTUP_ENTRY_MAX_RPM",
                "STARTUP_EXIT_TOLERANCE_RPM","STARTUP_EXIT_MAX_SLOPE_RPM_PER_SECOND",
                "STARTUP_EXIT_STABLE_SECONDS","STARTUP_MAX_SECONDS","VOLTAGE_COMP_NOMINAL_V",
                "VOLTAGE_COMP_FILTER_TAU_SEC","VOLTAGE_COMP_MIN_GAIN","VOLTAGE_COMP_MAX_GAIN"};
        double[] values={.0009,0,0,.09,.000134,0,1.5,1.5,1.5,6,6,200,50,200,.15,2,12.5,.5,.90,1.10};
        for(int i=0;i<names.length;i++)p=p.shooter(id,names[i],values[i]);
        p=p.shooter(id,"ENABLE_ASYMMETRIC_SLEW",true).shooter(id,"ENABLE_STARTUP_PROFILE",true)
                .shooter(id,"ENABLE_VOLTAGE_COMPENSATION",true);
        String[] shotNames={"FEED_SECONDS","BOOST_DELAY_SECONDS","BOOST_DURATION_SECONDS","BOOST_MINIMUM_POWER",
                "BOOST_START_POWER","BOOST_END_POWER","BOOST_NOMINAL_V","BOOST_OVERSPEED_TAPER_RPM"};
        double[] shotValues={1.2,.04,.50,.75,.80,.90,12.5,100};
        for(int i=0;i<shotNames.length;i++)p=p.shot(id,shotNames[i],shotValues[i]);
        return p.shot(id,"ENABLE_TIMED_BOOST",true).shot(id,"ENABLE_RAMPED_BOOST",true)
                .shot(id,"ENABLE_BOOST_VOLTAGE_SCALING",true).shot(id,"BOOST_BYPASSES_SLEW",true)
                .intake(id,"SHOOT_INTAKE_POWER",.50).intake(id,"SHOOT_TRANSFER_POWER",.50);
    }

    public static ShotExperimentPlan2026 recommended(ScopedShotSettings2026 scope) {
        ShotExperimentPlan2026.validateSettings();
        if(ShotExperimentSettings2026.RECOMMENDED_BATCH==null)throw new IllegalArgumentException("Select recommended batch");
        repeats(ShotExperimentSettings2026.RECOMMENDED_REPETITIONS);
        positive("Loaded RPM bracket offset",ShotExperimentSettings2026.LOADED_RPM_BRACKET_OFFSET);
        Profile p=referenceProfile();List<Profile> profiles;
        List<Condition> conditions=new ArrayList<>();
        if(ShotExperimentSettings2026.RECOMMENDED_COLD)conditions.add(Condition.COLD);
        if(ShotExperimentSettings2026.RECOMMENDED_ALREADY_SPINNING)conditions.add(Condition.ALREADY_SPINNING);
        double step=ShotExperimentSettings2026.STEP_DELTA_RPM;
        switch(ShotExperimentSettings2026.RECOMMENDED_BATCH) {
            case QUICK_SCREEN:profiles=Arrays.asList(ShotExperimentPlan2026.base(),p,
                    boost(p,"BOOST_75_85",.75,.85),p.shot("DELAY_08","BOOST_DELAY_SECONDS",.08));break;
            case STARTUP_NEIGHBORS:profiles=Arrays.asList(p,startup(p,"START3_6",3,6),
                    startup(p,"START6_3",6,3),startup(p,"START10_6",10,6));break;
            case BOOST_LEVEL:profiles=Arrays.asList(boost(p,"BOOST_75_85",.75,.85),p,boost(p,"BOOST_85_95",.85,.95));break;
            case BOOST_TIMING:profiles=Arrays.asList(p.shot("DELAY_00","BOOST_DELAY_SECONDS",0.0),p,
                    p.shot("DELAY_08","BOOST_DELAY_SECONDS",.08));break;
            case BOOST_TAPER:profiles=Arrays.asList(p.shot("TAPER_0","BOOST_OVERSPEED_TAPER_RPM",0.0),p,
                    p.shot("TAPER_200","BOOST_OVERSPEED_TAPER_RPM",200.0));break;
            case RUNNING_NEIGHBORS:profiles=Arrays.asList(p,running(p,"RUN1_5_3",1.5,3),running(p,"RUN3_3",3,3));
                conditions=Arrays.asList(Condition.ALREADY_SPINNING,Condition.STEP_UP,Condition.STEP_DOWN);break;
            case LARGE_TRANSITIONS:profiles=Arrays.asList(p,running(p,"RUN1_5_3",1.5,3));
                conditions=Arrays.asList(Condition.STEP_UP,Condition.STEP_DOWN);
                step=ShotExperimentSettings2026.MODEL_LARGE_STEP_RPM;break;
            case FEED_NEIGHBORS:profiles=Arrays.asList(p,p.intake("TRANSFER_425","SHOOT_TRANSFER_POWER",.425),
                    p.intake("INTAKE_425","SHOOT_INTAKE_POWER",.425),
                    p.intake("BOTH_425","SHOOT_INTAKE_POWER",.425).intake("BOTH_425","SHOOT_TRANSFER_POWER",.425),
                    p.intake("BOTH_350","SHOOT_INTAKE_POWER",.35).intake("BOTH_350","SHOOT_TRANSFER_POWER",.35));break;
            case LOADED_RPM_BRACKET:profiles=Collections.singletonList(p);break;
            case P_GAIN:profiles=Arrays.asList(p.shooter("P_0007","kP",.0007),p,p.shooter("P_0011","kP",.0011));break;
            case FEEDFORWARD_GAIN:profiles=Arrays.asList(p.shooter("KV_MINUS5","kV",.0001273),p,
                    p.shooter("KV_PLUS5","kV",.0001407));break;
            default:throw new IllegalArgumentException("Unknown recommended batch");
        }
        if(conditions.isEmpty())throw new IllegalArgumentException("Enable a recommended starting condition");
        List<Trial> trials=new ArrayList<>();
        double center=ShotExperimentSettings2026.TARGET_RPM,offset=ShotExperimentSettings2026.LOADED_RPM_BRACKET_OFFSET;
        double[] targets=ShotExperimentSettings2026.RECOMMENDED_BATCH==Batch.LOADED_RPM_BRACKET?
                new double[]{center-offset,center,center+offset}:new double[]{center};
        for(int r=0;r<ShotExperimentSettings2026.RECOMMENDED_REPETITIONS;r++) {
            for(int n=0;n<profiles.size()*targets.length;n++) {
                int order=Math.floorMod(n+r+ShotExperimentSettings2026.PROFILE_ORDER_OFFSET,profiles.size()*targets.length);
                Profile profile=profiles.get(order/targets.length);
                if(ShotExperimentSettings2026.TEST_FEED_SECONDS>0)
                    profile=profile.shot(profile.id,"FEED_SECONDS",ShotExperimentSettings2026.TEST_FEED_SECONDS);
                for(Condition c:conditions)trials.add(new Trial(trials.size()+1,r+1,profile,c,targets[order%targets.length],step,
                        ShotExperimentSettings2026.TARGET_RAMP_SECONDS,c==Condition.COLD?
                        ShotExperimentSettings2026.COLD_FEED_DELAY_SECONDS:ShotExperimentSettings2026.RUNNING_FEED_DELAY_SECONDS,true,false));
            }
        }
        validateProfiles(scope,trials);
        return new ShotExperimentPlan2026(trials,"RECOMMENDED_"+ShotExperimentSettings2026.RECOMMENDED_BATCH,
                ShotExperimentSettings2026.RUNNING_PREPARE_SECONDS,ShotExperimentSettings2026.POST_FEED_CAPTURE_SECONDS,0);
    }

    public static ShotExperimentPlan2026 model(ScopedShotSettings2026 scope) {
        ShotExperimentPlan2026.validateSettings();
        if(ShotExperimentSettings2026.MODEL_BATCH==null)throw new IllegalArgumentException("Select model batch");
        repeats(ShotExperimentSettings2026.MODEL_REPETITIONS);
        for(double value:new double[]{ShotExperimentSettings2026.MODEL_RPM_LOW,ShotExperimentSettings2026.MODEL_RPM_CENTER,
                ShotExperimentSettings2026.MODEL_RPM_HIGH,ShotExperimentSettings2026.MODEL_SMALL_STEP_RPM,
                ShotExperimentSettings2026.MODEL_LARGE_STEP_RPM})positive("Model RPM/step",value);
        for(double value:new double[]{ShotExperimentSettings2026.MODEL_PREPARE_SECONDS,ShotExperimentSettings2026.MODEL_OBSERVE_SECONDS,
                ShotExperimentSettings2026.MODEL_HOLD_SECONDS,ShotExperimentSettings2026.MODEL_STEP_RAMP_SECONDS,
                ShotExperimentSettings2026.MODEL_BETWEEN_TRIAL_SECONDS})
            if(!Double.isFinite(value)||value<0)throw new IllegalArgumentException("Model timings must be finite and nonnegative");
        if(ShotExperimentSettings2026.MODEL_OBSERVE_SECONDS<ShotExperimentSettings2026.MODEL_STEP_RAMP_SECONDS)
            throw new IllegalArgumentException("Model observation must include the entire target ramp");
        Profile p=referenceProfile().shot("UNLOADED_REFERENCE","ENABLE_TIMED_BOOST",false);
        List<Trial> trials=new ArrayList<>();
        boolean coast=ShotExperimentSettings2026.MODEL_BATCH==ModelBatch.STARTUP_AND_COAST;
        double[] targets={ShotExperimentSettings2026.MODEL_RPM_LOW,ShotExperimentSettings2026.MODEL_RPM_CENTER,ShotExperimentSettings2026.MODEL_RPM_HIGH};
        for(int r=0;r<ShotExperimentSettings2026.MODEL_REPETITIONS;r++) {
            for(int n=0;n<(coast?3:8);n++) {
                int i=Math.floorMod(n+r+ShotExperimentSettings2026.PROFILE_ORDER_OFFSET,coast?3:8);
                Condition c=coast?Condition.COLD:i%2==0?Condition.STEP_UP:Condition.STEP_DOWN;
                double step=i%4<2?ShotExperimentSettings2026.MODEL_SMALL_STEP_RPM:ShotExperimentSettings2026.MODEL_LARGE_STEP_RPM;
                double ramp=coast||i<4?0:ShotExperimentSettings2026.MODEL_STEP_RAMP_SECONDS;
                trials.add(new Trial(trials.size()+1,r+1,p,c,coast?targets[i]:ShotExperimentSettings2026.MODEL_RPM_CENTER,
                        step,ramp,ShotExperimentSettings2026.MODEL_OBSERVE_SECONDS,false,coast));
            }
        }
        validateProfiles(scope,trials);
        return new ShotExperimentPlan2026(trials,"UNLOADED_"+ShotExperimentSettings2026.MODEL_BATCH,
                ShotExperimentSettings2026.MODEL_PREPARE_SECONDS,0,ShotExperimentSettings2026.MODEL_HOLD_SECONDS);
    }
    private static void validateProfiles(ScopedShotSettings2026 scope,List<Trial> trials) {
        try {for(Trial t:trials)scope.apply(t.profile);}
        finally {scope.resetToBaseline();}
    }
    private static void positive(String label,double value) {
        if(!Double.isFinite(value)||value<=0)throw new IllegalArgumentException(label+" must be positive and finite");
    }
    private static void repeats(int count) {
        if(count<1||count>20)throw new IllegalArgumentException("Test repetitions must be 1..20");
    }
    private static Profile boost(Profile p,String id,double start,double end) {
        return p.shot(id,"BOOST_START_POWER",start).shot(id,"BOOST_END_POWER",end);
    }
    private static Profile startup(Profile p,String id,double up,double down) {
        return p.shooter(id,"STARTUP_SLEW_UP_PER_SECOND",up).shooter(id,"STARTUP_SLEW_DOWN_PER_SECOND",down);
    }
    private static Profile running(Profile p,String id,double up,double down) {
        return p.shooter(id,"RUNNING_SLEW_UP_PER_SECOND",up).shooter(id,"RUNNING_SLEW_DOWN_PER_SECOND",down);
    }
}
