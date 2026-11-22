import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.*;
import org.firstinspires.ftc.teamcode.testing.shots.*;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.logging.RobotLogging2026;

/** Deterministic tests of production experiment plans, state machine and settings ownership. */
public final class ShotExperiment2026Test {
    private static int checks;
    private static final Class<?>[] NORMAL={ShooterSubsystem2026.class,ShotSequenceController2026.class,
            IntakeWithSensorsSubsystem2026.class,LEDControlSubsystem2026.class,RobotLogging2026.class};
    private static Map<Field,Object> normalStart,testStart;

    public static void main(String[] args) throws Exception {
        normalStart=snapshot(NORMAL);testStart=snapshot(ShotExperimentSettings2026.class);
        try {
            planMatrixAndRestoration();feedPowerProfiles();immutableAndInvalidPlans();
            coldTimingAndFreshReady();coastTimeoutAndPause();runningRampsAndNoRpmGate();skipAndCompletion();
            recommendedBatches();modelPlansAndValidation();unloadedSequencing();
            check("all normal configuration restored at end",normalStart.equals(snapshot(NORMAL)));
        } finally { restore(testStart);restore(normalStart); }
        System.out.println("Passed "+checks+" automated shot experiment checks.");
    }
    private static void defaults() throws Exception { restore(testStart);restore(normalStart); }
    private static ShotExperimentPlan2026 build(boolean comparison) {
        try(ScopedShotSettings2026 scope=new ScopedShotSettings2026()) {
            ShotExperimentPlan2026 plan=ShotExperimentPlan2026.build(comparison,scope);
            check("building plan restores all normal parameters before returning",normalStart.equals(snapshot(NORMAL)));
            return plan;
        }
    }
    private static void planMatrixAndRestoration() throws Exception {
        defaults();
        for(ShotExperimentPlan2026.ComparisonSet set:ShotExperimentPlan2026.ComparisonSet.values()) {
            ShotExperimentSettings2026.COMPARISON_SET=set;
            ShotExperimentPlan2026 plan=build(true);
            int profiles=set==ShotExperimentPlan2026.ComparisonSet.FEED_POWER?5:set==ShotExperimentPlan2026.ComparisonSet.CUSTOM_VS_CURRENT?2:4;
            check("paired comparison conditions "+set,plan.trials.size()==profiles*2);
            for(int i=0;i<plan.trials.size();i+=2) {
                check("same profile cold and running",plan.trials.get(i).profile==plan.trials.get(i+1).profile);
                check("cold first",plan.trials.get(i).condition==ShotExperimentPlan2026.Condition.COLD);
                check("warm second",plan.trials.get(i+1).condition==ShotExperimentPlan2026.Condition.ALREADY_SPINNING);
            }
        }
        defaults();ShotExperimentPlan2026 verify=build(false);
        check("verification defaults to twelve trials",verify.trials.size()==12);
        Map<ShotExperimentPlan2026.Condition,Integer> conditions=new EnumMap<>(ShotExperimentPlan2026.Condition.class);
        for(int i=0;i<verify.trials.size();i++) {
            ShotExperimentPlan2026.Trial t=verify.trials.get(i);
            check("monotonic unique trial id",t.id==i+1);
            check("same final RPM across all conditions",t.targetRpm==3196);
            conditions.merge(t.condition,1,Integer::sum);
            if(t.condition==ShotExperimentPlan2026.Condition.STEP_UP)check("up starts 200 below",t.initialTargetRpm==2996);
            if(t.condition==ShotExperimentPlan2026.Condition.STEP_DOWN)check("down starts 200 above",t.initialTargetRpm==3396);
        }
        for(int n:conditions.values())check("three repeats each condition",n==3);
        ShotExperimentSettings2026.COMPARISON_REPETITIONS=2;
        ShotExperimentSettings2026.PROFILE_ORDER_OFFSET=1;
        ShotExperimentPlan2026 rotated=build(true);
        check("profile order offset",rotated.trials.get(0).profile.id.equals("START_DOWN6"));
        check("profile order rotates next repetition",rotated.trials.get(8).profile.id.equals("START6_6"));
        defaults();
        ShotExperimentPlan2026 plan=build(true);
        try(ScopedShotSettings2026 scope=new ScopedShotSettings2026()) {
            scope.apply(plan.trials.get(6).profile);
            check("fast test profile applied",ShooterSubsystem2026.ENABLE_STARTUP_PROFILE && ShooterSubsystem2026.STARTUP_SLEW_UP_PER_SECOND==10);
            scope.apply(plan.trials.get(0).profile);
            check("baseline after candidate really resets all fields",normalStart.equals(snapshot(NORMAL)));
            LEDControlSubsystem2026.STAGE_PULSE_PRESET=LEDControlSubsystem2026.LedPreset.WHITE;
            RobotLogging2026.PROFILE_LABEL="test_temp";
        }
        check("LED and logger restored as well",normalStart.equals(snapshot(NORMAL)));
    }
    private static void feedPowerProfiles() throws Exception {
        defaults();ShotExperimentSettings2026.COMPARISON_SET=ShotExperimentPlan2026.ComparisonSet.FEED_POWER;
        ShotExperimentPlan2026 p=build(true);
        try(ScopedShotSettings2026 s=new ScopedShotSettings2026()) {
            s.apply(p.trials.get(2).profile);
            near("transfer-only lower feeding power",IntakeWithSensorsSubsystem2026.SHOOT_TRANSFER_POWER,.425);
            near("intake feed stays baseline",IntakeWithSensorsSubsystem2026.SHOOT_INTAKE_POWER,.5);
            s.apply(p.trials.get(4).profile);
            near("intake-only lower feeding power",IntakeWithSensorsSubsystem2026.SHOOT_INTAKE_POWER,.425);
            near("transfer reset from prior candidate",IntakeWithSensorsSubsystem2026.SHOOT_TRANSFER_POWER,.5);
            s.apply(p.trials.get(8).profile);
            near("slow both intake",IntakeWithSensorsSubsystem2026.SHOOT_INTAKE_POWER,.35);
            near("slow both transfer",IntakeWithSensorsSubsystem2026.SHOOT_TRANSFER_POWER,.35);
            check("collection RPMs untouched",IntakeWithSensorsSubsystem2026.INTAKE_FORWARD_RPM==4000
                    && IntakeWithSensorsSubsystem2026.TRANSFER_FORWARD_RPM==700);
        }
        check("feed settings restored",normalStart.equals(snapshot(NORMAL)));
        ShotExperimentSettings2026.TEST_FEED_SECONDS=1.5;
        ShotExperimentPlan2026 extended=build(true);
        try(ScopedShotSettings2026 s=new ScopedShotSettings2026()) {
            for(ShotExperimentPlan2026.Trial t:extended.trials) {
                s.apply(t.profile);near("test-only duration override",ShotSequenceController2026.FEED_SECONDS,1.5);
            }
        }
        check("duration override never persists into normal settings",normalStart.equals(snapshot(NORMAL)));
    }
    private static void immutableAndInvalidPlans() throws Exception {
        defaults();ShotExperimentSettings2026.VERIFICATION_PROFILE=ShotExperimentPlan2026.VerificationProfile.CUSTOM;
        ShotExperimentSettings2026.CUSTOM_KP=.0011;
        ShotExperimentPlan2026 p=build(false);
        ShotExperimentSettings2026.CUSTOM_KP=.003;
        try(ScopedShotSettings2026 s=new ScopedShotSettings2026()) {
            s.apply(p.trials.get(0).profile);near("custom profile frozen at Play",ShooterSubsystem2026.kP,.0011);
        }
        ShotExperimentSettings2026.CUSTOM_FEED_TRANSFER_POWER=1.2;
        boolean failed=false;
        try(ScopedShotSettings2026 s=new ScopedShotSettings2026()) {ShotExperimentPlan2026.build(false,s);}
        catch(IllegalArgumentException e) {failed=true;}
        check("invalid candidate fails validation",failed);
        check("failed validation also restores all normal settings",normalStart.equals(snapshot(NORMAL)));
        defaults();ShotExperimentSettings2026.TARGET_RAMP_SECONDS=3;
        failed=false;try {build(true);}catch(IllegalArgumentException e){failed=true;}
        check("cannot schedule feed before ramp completes",failed);
        defaults();ShotExperimentSettings2026.VERIFY_COLD=false;ShotExperimentSettings2026.VERIFY_ALREADY_SPINNING=false;
        ShotExperimentSettings2026.VERIFY_STEP_UP=false;ShotExperimentSettings2026.VERIFY_STEP_DOWN=false;
        failed=false;try {build(false);}catch(IllegalArgumentException e){failed=true;}
        check("empty condition selection fails Init",failed);
        defaults();
    }
    private static ShotExperimentPlan2026 single(ShotExperimentPlan2026.Condition condition,int repeats) throws Exception {
        defaults();ShotExperimentSettings2026.VERIFICATION_REPETITIONS=repeats;
        ShotExperimentSettings2026.VERIFY_COLD=condition==ShotExperimentPlan2026.Condition.COLD;
        ShotExperimentSettings2026.VERIFY_ALREADY_SPINNING=condition==ShotExperimentPlan2026.Condition.ALREADY_SPINNING;
        ShotExperimentSettings2026.VERIFY_STEP_UP=condition==ShotExperimentPlan2026.Condition.STEP_UP;
        ShotExperimentSettings2026.VERIFY_STEP_DOWN=condition==ShotExperimentPlan2026.Condition.STEP_DOWN;
        ShotExperimentSettings2026.RUNNING_PREPARE_SECONDS=.1;
        ShotExperimentSettings2026.STOPPED_STABLE_SECONDS=.05;
        ShotExperimentSettings2026.COAST_TIMEOUT_SECONDS=1;
        ShotExperimentSettings2026.POST_FEED_CAPTURE_SECONDS=.1;
        return build(false);
    }
    private static void coldTimingAndFreshReady() throws Exception {
        ShotExperimentPlan2026 p=single(ShotExperimentPlan2026.Condition.COLD,2);
        Fake f=new Fake();f.rpm=1000;
        ShotExperimentController2026 c=new ShotExperimentController2026(p,f,ns(0),true);
        tick(c,f,0,true,false,false,false);
        check("Y held at Play is not ready",c.getAttemptId()==0);
        tick(c,f,.01,false,false,false,false);tick(c,f,.02,true,false,false,false);
        check("ready starts cold coast",c.getPhase()==ShotExperimentController2026.Phase.COASTING_TO_STOP && f.stops==1);
        tick(c,f,.1,true,false,false,false);check("cannot claim cold while RPM remains high",f.starts==0);
        f.rpm=0;tick(c,f,.2,true,false,false,false);tick(c,f,.25,true,false,false,false);
        check("stable stopped sample begins spinup",c.getPhase()==ShotExperimentController2026.Phase.WAITING_TO_FEED);
        near("cold initial measured RPM recorded",c.getInitialMeasuredRpm(),0);
        f.rpm=1200;tick(c,f,2.24,true,false,false,false);check("fixed cold delay not early",f.feeds==0);
        tick(c,f,2.25,true,false,false,false);check("fixed cold delay does not wait for target RPM",f.feeds==1 && f.feedCues==1);
        tick(c,f,2.36,true,false,false,false);tick(c,f,2.47,true,false,false,false);
        check("completed trial waits for reload",c.canReload() && c.getCompletedCount()==1);
        check("last trial metadata retained through reload",c.trialForLog().id==1 && c.trial().id==2);
        tick(c,f,3,true,false,false,false);check("held Y cannot queue second feed",c.getAttemptId()==1);
        tick(c,f,3.01,false,false,false,false);tick(c,f,3.02,true,false,false,false);
        check("fresh Y starts second attempt",c.getAttemptId()==2 && f.confirmations==2);
    }
    private static void coastTimeoutAndPause() throws Exception {
        ShotExperimentPlan2026 p=single(ShotExperimentPlan2026.Condition.COLD,1);Fake f=new Fake();f.rpm=2000;
        ShotExperimentController2026 c=new ShotExperimentController2026(p,f,ns(0));
        tick(c,f,0,true,false,false,false);tick(c,f,1.01,true,false,false,false);
        check("coast timeout is bounded",c.getPhase()==ShotExperimentController2026.Phase.PAUSED && c.getReason().equals("COAST_TIMEOUT"));
        check("coast timeout never feeds",f.feeds==0);
        tick(c,f,1.02,true,true,false,false);tick(c,f,1.03,true,false,false,false);
        check("holding ready after X cannot retry",c.getAttemptId()==1);
        f.rpm=0;tick(c,f,1.04,false,false,false,false);tick(c,f,1.05,true,false,false,false);
        check("fresh retry gets new attempt ID",c.getAttemptId()==2);
        tick(c,f,1.06,false,false,false,true);
        check("manual reverse aborts an active timed trial",c.getPhase()==ShotExperimentController2026.Phase.PAUSED && c.getReason().equals("MANUAL_REVERSE"));
    }
    private static void runningRampsAndNoRpmGate() throws Exception {
        for(ShotExperimentPlan2026.Condition condition:new ShotExperimentPlan2026.Condition[]{
                ShotExperimentPlan2026.Condition.STEP_UP,ShotExperimentPlan2026.Condition.STEP_DOWN}) {
            ShotExperimentPlan2026 p=single(condition,1);Fake f=new Fake();f.rpm=2700;
            ShotExperimentController2026 c=new ShotExperimentController2026(p,f,ns(0));
            tick(c,f,0,true,false,false,false);
            check("warm condition does not stop motor",f.stops==0);
            near("initial requested RPM",f.target,condition==ShotExperimentPlan2026.Condition.STEP_UP?2996:3396);
            tick(c,f,.1,false,false,false,false);near("actual transition RPM retained",c.getInitialMeasuredRpm(),2700);
            tick(c,f,.6,false,false,false,false);near("ramp midpoint",f.target,condition==ShotExperimentPlan2026.Condition.STEP_UP?3096:3296);
            tick(c,f,1.1,false,false,false,false);near("ramp reaches common final target",f.target,3196);
            check("ramp itself does not start feeding",f.feeds==0);
            tick(c,f,2.1,false,false,false,false);check("warm feed delay remains fixed despite RPM error",f.feeds==1);
            tick(c,f,2.11,false,true,false,false);
            check("X cancels active feed",!f.feeding && c.getPhase()==ShotExperimentController2026.Phase.PAUSED);
        }
    }
    private static void skipAndCompletion() throws Exception {
        ShotExperimentPlan2026 p=single(ShotExperimentPlan2026.Condition.ALREADY_SPINNING,2);Fake f=new Fake();
        ShotExperimentController2026 c=new ShotExperimentController2026(p,f,ns(0));
        tick(c,f,0,false,false,true,false);
        check("skip waits for next operator confirmation",c.trial().id==2 && c.getSkippedCount()==1 && f.feeds==0);
        check("skip records correct planned trial ID",c.getLastResultTrialId()==1 && c.getLastResult().equals("SKIPPED"));
        tick(c,f,.01,true,false,false,false);tick(c,f,.11,false,false,false,false);
        tick(c,f,2.11,false,false,false,false);tick(c,f,2.22,false,false,false,false);tick(c,f,2.33,false,false,false,false);
        check("last recovery completes plan",c.getPhase()==ShotExperimentController2026.Phase.COMPLETE && c.getCompletedCount()==1);
        check("completion stops every actuator",f.stopAll>0 && !f.feeding && !f.enabled);
        int attempts=c.getAttemptId();tick(c,f,3,true,false,false,false);
        check("complete cannot silently restart",c.getAttemptId()==attempts && f.feeds==1);
    }
    private static ShotExperimentPlan2026 recommended() {
        try(ScopedShotSettings2026 s=new ScopedShotSettings2026()) {
            ShotExperimentPlan2026 p=RecommendedShotPlans2026.recommended(s);
            check("recommended Init restores configuration",normalStart.equals(snapshot(NORMAL)));return p;
        }
    }
    private static ShotExperimentPlan2026 model() {
        try(ScopedShotSettings2026 s=new ScopedShotSettings2026()) {
            ShotExperimentPlan2026 p=RecommendedShotPlans2026.model(s);
            check("model Init restores configuration",normalStart.equals(snapshot(NORMAL)));return p;
        }
    }
    private static void recommendedBatches() throws Exception {
        defaults();
        int[] counts={8,8,6,6,6,9,4,10,6,6,6};
        for(RecommendedShotPlans2026.Batch batch:RecommendedShotPlans2026.Batch.values()) {
            ShotExperimentSettings2026.RECOMMENDED_BATCH=batch;
            ShotExperimentPlan2026 p=recommended();
            check("batch count "+batch,p.trials.size()==counts[batch.ordinal()]);
            for(int i=0;i<p.trials.size();i++) {
                ShotExperimentPlan2026.Trial t=p.trials.get(i);
                check("loaded batch feeds with no coast capture",t.feedEnabled&&!t.recordCoast);
                check("batch trial ID monotonic",t.id==i+1);
                if(batch==RecommendedShotPlans2026.Batch.LARGE_TRANSITIONS)
                    near("large transitions keep final shot RPM",Math.abs(t.initialTargetRpm-t.targetRpm),450);
            }
            check("manifest contains profile settings",p.manifest().contains("BOOST_NOMINAL_V=12.5"));
            check("manifest contains trial targets",p.manifest().contains("final=3196.0"));
        }
        defaults();ShotExperimentPlan2026 quick=recommended();
        check("quick screen starts current",quick.trials.get(0).profile.id.equals("CURRENT"));
        check("quick candidate paired cold/warm",quick.trials.get(2).condition==ShotExperimentPlan2026.Condition.COLD
                &&quick.trials.get(3).condition==ShotExperimentPlan2026.Condition.ALREADY_SPINNING);
        try(ScopedShotSettings2026 s=new ScopedShotSettings2026()) {
            s.apply(quick.trials.get(2).profile);
            near("reference startup up",ShooterSubsystem2026.STARTUP_SLEW_UP_PER_SECOND,6);
            near("reference startup down",ShooterSubsystem2026.STARTUP_SLEW_DOWN_PER_SECOND,6);
            near("reference running unchanged rate",ShooterSubsystem2026.RUNNING_SLEW_DOWN_PER_SECOND,1.5);
            near("reference FF high-voltage reduction",ShooterSubsystem2026.VOLTAGE_COMP_MIN_GAIN,.9);
            check("reference voltage flags",ShooterSubsystem2026.ENABLE_VOLTAGE_COMPENSATION&&ShotSequenceController2026.ENABLE_BOOST_VOLTAGE_SCALING);
            near("reference boost delay",ShotSequenceController2026.BOOST_DELAY_SECONDS,.04);
            near("reference boost start",ShotSequenceController2026.BOOST_START_POWER,.8);
            near("reference boost end",ShotSequenceController2026.BOOST_END_POWER,.9);
            Map<Field,Object> ref=snapshot(NORMAL);
            s.apply(quick.trials.get(4).profile);
            check("boost neighbor changes only its two endpoints",differences(ref,snapshot(NORMAL)).equals(new HashSet<>(Arrays.asList(
                    ShotSequenceController2026.class.getField("BOOST_START_POWER"),ShotSequenceController2026.class.getField("BOOST_END_POWER")))));
            s.apply(quick.trials.get(6).profile);
            check("timing neighbor changes only delay",differences(ref,snapshot(NORMAL)).equals(Collections.singleton(
                    ShotSequenceController2026.class.getField("BOOST_DELAY_SECONDS"))));
            ShooterSubsystem2026.kP=.1; // Explicit reference must override later production settings.
            s.apply(quick.trials.get(2).profile);near("reference pinned",ShooterSubsystem2026.kP,.0009);
        }
        ShotExperimentSettings2026.RECOMMENDED_BATCH=RecommendedShotPlans2026.Batch.LOADED_RPM_BRACKET;
        ShotExperimentPlan2026 bracket=recommended();
        near("loaded bracket low",bracket.trials.get(0).targetRpm,3096);
        near("loaded bracket center",bracket.trials.get(2).targetRpm,3196);
        near("loaded bracket high",bracket.trials.get(4).targetRpm,3296);
        ShotExperimentSettings2026.RECOMMENDED_BATCH=RecommendedShotPlans2026.Batch.FEED_NEIGHBORS;
        ShotExperimentSettings2026.TEST_FEED_SECONDS=1.6;
        ShotExperimentPlan2026 feeds=recommended();
        try(ScopedShotSettings2026 s=new ScopedShotSettings2026()) {
            s.apply(feeds.trials.get(8).profile);
            near("recommended slow feed intake",IntakeWithSensorsSubsystem2026.SHOOT_INTAKE_POWER,.35);
            near("recommended slow feed transfer",IntakeWithSensorsSubsystem2026.SHOOT_TRANSFER_POWER,.35);
            near("common loaded duration override",ShotSequenceController2026.FEED_SECONDS,1.6);
            near("collection transfer untouched",IntakeWithSensorsSubsystem2026.TRANSFER_FORWARD_RPM,700);
        }
        defaults();ShotExperimentSettings2026.RECOMMENDED_REPETITIONS=2;ShotExperimentSettings2026.PROFILE_ORDER_OFFSET=1;
        ShotExperimentPlan2026 rotated=recommended();
        check("recommended rotation per repetition",rotated.trials.size()==16&&rotated.trials.get(0).profile.id.equals("MODEL_CANDIDATE")
                &&rotated.trials.get(8).profile.id.equals("BOOST_75_85"));
        defaults();
    }
    private static Set<Field> differences(Map<Field,Object> a,Map<Field,Object> b) {
        Set<Field> diff=new HashSet<>();for(Field f:a.keySet())if(!Objects.equals(a.get(f),b.get(f)))diff.add(f);return diff;
    }
    private static void modelPlansAndValidation() throws Exception {
        defaults();ShotExperimentPlan2026 p=model();
        check("model three cold targets",p.trials.size()==3);
        for(int i=0;i<3;i++) {
            ShotExperimentPlan2026.Trial t=p.trials.get(i);
            check("unloaded cold plus coast",!t.feedEnabled&&t.recordCoast&&t.condition==ShotExperimentPlan2026.Condition.COLD);
            near("model target sweep",t.targetRpm,2800+400*i);
        }
        ShotExperimentSettings2026.MODEL_BATCH=RecommendedShotPlans2026.ModelBatch.STEP_RESPONSE;
        p=model();check("model eight step/ramp conditions",p.trials.size()==8);
        Set<String> cases=new HashSet<>();
        for(ShotExperimentPlan2026.Trial t:p.trials) {
            check("unloaded step remains spinning",!t.feedEnabled&&!t.recordCoast);
            near("common model final target",t.targetRpm,3200);
            cases.add((t.initialTargetRpm-t.targetRpm)+"/"+t.rampSeconds);
        }
        check("all plus/minus small/large and step/ramp combinations",cases.size()==8);
        check("frozen model config includes no feed flag",p.manifest().contains("feed=false"));
        ShotExperimentSettings2026.MODEL_RPM_CENTER=100;
        boolean failed=false;try{model();}catch(IllegalArgumentException e){failed=true;}
        check("invalid negative starting RPM rejected",failed);
        check("failed model leaves normal unchanged",normalStart.equals(snapshot(NORMAL)));
        defaults();ShotExperimentSettings2026.RECOMMENDED_COLD=false;ShotExperimentSettings2026.RECOMMENDED_ALREADY_SPINNING=false;
        failed=false;try{recommended();}catch(IllegalArgumentException e){failed=true;}
        check("empty recommended conditions rejected",failed);
        defaults();ShotExperimentSettings2026.MODEL_OBSERVE_SECONDS=.5;
        failed=false;try{model();}catch(IllegalArgumentException e){failed=true;}
        check("model observation cannot truncate ramp",failed);defaults();
    }
    private static void unloadedSequencing() throws Exception {
        defaults();ShotExperimentSettings2026.MODEL_OBSERVE_SECONDS=.1;ShotExperimentSettings2026.MODEL_STEP_RAMP_SECONDS=0;
        ShotExperimentSettings2026.MODEL_HOLD_SECONDS=.1;ShotExperimentSettings2026.STOPPED_STABLE_SECONDS=.05;
        ShotExperimentSettings2026.COAST_TIMEOUT_SECONDS=1;
        ShotExperimentPlan2026 p=model();Fake f=new Fake();
        ShotExperimentController2026 c=new ShotExperimentController2026(p,f,ns(0));
        tick(c,f,0,true,false,false,false);tick(c,f,.01,false,false,false,false);tick(c,f,.06,false,false,false,false);
        near("unloaded enables first target",f.target,2800);
        f.rpm=2800;tick(c,f,.16,false,false,false,false);
        check("unloaded hold replaces feed",c.getPhase()==ShotExperimentController2026.Phase.UNLOADED_HOLD);
        check("no confirmation, feed or white cue",f.confirmations==0&&f.feeds==0&&f.feedCues==0);
        tick(c,f,.26,false,false,false,false);
        check("unloaded coast turns motor off",c.getPhase()==ShotExperimentController2026.Phase.COAST_CAPTURE&&!f.enabled);
        tick(c,f,.5,false,false,false,false);check("coast waits for measured stop",c.getCompletedCount()==0);
        f.rpm=0;tick(c,f,.6,true,false,false,false);f.rpm=100;tick(c,f,.63,true,false,false,false);
        f.rpm=0;tick(c,f,.7,true,false,false,false);tick(c,f,.75,true,false,false,false);
        check("coast stable-zero completes",c.getCompletedCount()==1&&c.canReload());
        tick(c,f,.8,true,false,false,false);check("holding Y cannot restart unloaded",c.getAttemptId()==1);
        tick(c,f,.81,false,false,false,false);tick(c,f,.82,true,false,false,false);
        tick(c,f,.83,false,false,false,false);tick(c,f,.88,false,false,false,false);
        f.rpm=3200;tick(c,f,.98,false,false,false,false);tick(c,f,1.08,false,false,false,false);
        tick(c,f,2.09,false,false,false,false);
        check("coast failure pauses not completes",c.getReason().equals("COAST_CAPTURE_TIMEOUT")&&c.getCompletedCount()==1);
        check("unloaded never shoots during timeout",f.feeds==0&&f.confirmations==0);
        defaults();ShotExperimentSettings2026.MODEL_BATCH=RecommendedShotPlans2026.ModelBatch.STEP_RESPONSE;
        ShotExperimentSettings2026.MODEL_PREPARE_SECONDS=.1;
        p=model();f=new Fake();c=new ShotExperimentController2026(p,f,ns(0));
        tick(c,f,0,true,false,false,false);near("step initial 3000",f.target,3000);
        f.rpm=2990;tick(c,f,.1,false,false,false,false);near("step final 3200",f.target,3200);
        near("actual model starting RPM recorded",c.getInitialMeasuredRpm(),2990);
        tick(c,f,2.1,false,false,false,false);tick(c,f,4.1,false,false,false,false);
        check("step completes without stopping warm motor",c.canReload()&&c.getCompletedCount()==1&&f.enabled&&f.stops==0);
        check("step model never feeds",f.feeds==0&&f.confirmations==0&&f.feedCues==0);defaults();
    }
    private static final class Fake implements ShotExperimentController2026.Ports {
        double rpm,target,now,feedEnd;boolean feeding,enabled;
        int starts,stops,feeds,feedCues,confirmations,stopAll;
        public double measuredRpm(){return rpm;}
        public void applyProfile(ShotExperimentPlan2026.Profile profile){}
        public void confirmFourLoaded(){confirmations++;}
        public void stopCollection(){feeding=false;}
        public void spin(double value){if(!enabled)starts++;enabled=true;target=value;}
        public void stopShooter(){stops++;enabled=false;}
        public void startFeed(){feeds++;feeding=true;feedEnd=now+.1;}
        public void updateFeed(){if(now>=feedEnd)feeding=false;}
        public boolean feedActive(){return feeding;}
        public void stopAll(){stopAll++;feeding=false;enabled=false;}
        public void trialCue(){}
        public void feedCue(){feedCues++;}
    }
    private static void tick(ShotExperimentController2026 c,Fake f,double t,boolean ready,boolean abort,boolean skip,boolean reverse){f.now=t;c.tick(ns(t),ready,abort,skip,reverse);}
    private static long ns(double t){return 1_000_000_000L+Math.round(t*1e9);}
    private static Map<Field,Object> snapshot(Class<?>...classes) {
        Map<Field,Object> out=new LinkedHashMap<>();
        for(Class<?> type:classes)for(Field f:type.getFields())if(Modifier.isStatic(f.getModifiers())&&!Modifier.isFinal(f.getModifiers())
                &&(f.getType().isPrimitive()||f.getType().isEnum()||f.getType()==String.class)) {
            try {out.put(f,f.get(null));}catch(Exception e){throw new RuntimeException(e);}
        }
        return out;
    }
    private static void restore(Map<Field,Object> values) throws Exception {for(Map.Entry<Field,Object> e:values.entrySet())e.getKey().set(null,e.getValue());}
    private static void near(String label,double a,double b){check(label+": "+a+" vs "+b,Math.abs(a-b)<1e-9);}
    private static void check(String label,boolean value){checks++;if(!value)throw new AssertionError(label);}
}
