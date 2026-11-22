package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.logging.*;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.subsystems.LEDControlSubsystem2026.LedPreset;
import org.firstinspires.ftc.teamcode.testing.shots.*;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/** Shared stationary test runner. Does not initialize or command chassis, turret or flower hardware. */
public abstract class ShooterExperimentOpMode2026 extends NextFTCOpMode {
    private final ShooterSubsystem2026 shooter=ShooterSubsystem2026.INSTANCE;
    private final IntakeWithSensorsSubsystem2026 intake=IntakeWithSensorsSubsystem2026.INSTANCE;
    private final ShotSequenceController2026 shots=new ShotSequenceController2026();
    private final RobotLogging2026 logging=new RobotLogging2026();
    private final ShotExperimentLogContext2026 context=new ShotExperimentLogContext2026();
    private ScopedShotSettings2026 scope;
    private ShotExperimentPlan2026 plan;
    private ShotExperimentController2026 controller;
    private Pose manualPose;
    private double aimX=Double.NaN,aimY=Double.NaN;
    private boolean finished;
    private int lastResultEvent;
    private boolean manifestLogged;

    protected abstract boolean comparison();
    protected ShotExperimentPlan2026 buildPlan(ScopedShotSettings2026 settings) {
        return ShotExperimentPlan2026.build(comparison(),settings);
    }
    protected String programLabel() {return comparison()?"PARAMETER COMPARISON":"REPEATABILITY";}
    protected Object selectedGroup() {return comparison()?ShotExperimentSettings2026.COMPARISON_SET:ShotExperimentSettings2026.VERIFICATION_PROFILE;}
    protected boolean unloaded() {return false;}
    public ShooterExperimentOpMode2026() {
        addComponents(new SubsystemComponent(shooter,intake,LEDControlSubsystem2026.INSTANCE),BulkReadComponent.INSTANCE);
    }
    @Override public void onInit() {
        shooter.stop();intake.stopAll();shots.reset();
        try(ScopedShotSettings2026 validation=new ScopedShotSettings2026()) {
            plan=buildPlan(validation);
        }
        logging.registerConfigurationClass(ShotExperimentSettings2026.class);
        telemetry.addData("Shot test","Place and aim robot manually. No chassis/turret commands.");
    }
    @Override public void onWaitForStart() {
        telemetry.addData("Program",programLabel());
        telemetry.addData("Settings","Panels: ShotExperimentSettings2026; changes captured at Play");
        telemetry.addData("Selected group",selectedGroup());
        if(unloaded())telemetry.addData("Model low / center / high RPM","%.0f / %.0f / %.0f",
                ShotExperimentSettings2026.MODEL_RPM_LOW,ShotExperimentSettings2026.MODEL_RPM_CENTER,ShotExperimentSettings2026.MODEL_RPM_HIGH);
        else telemetry.addData("Target RPM",ShotExperimentSettings2026.TARGET_RPM);
        telemetry.addData("Battery/run label",ShotExperimentSettings2026.BATTERY_CONDITION+" / "+ShotExperimentSettings2026.RUN_LABEL);
        telemetry.addData("After Play",unloaded()?"EMPTY robot; fresh Y confirms empty/start each trial":"Refill to FOUR, then fresh Y to confirm/start each trial");
        telemetry.addData("Controls",unloaded()?"A disabled; B reverse; X pause/stop; dpad right skips while waiting":
                "A collect; GP2 hold A intake-only override; B reverse; X pause/stop; dpad right skip while waiting");
        telemetry.update();
    }
    @Override public void onStartButtonPressed() {
        try {
            shooter.stop();intake.stopAll();shots.reset();
            scope=new ScopedShotSettings2026();
            plan=buildPlan(scope);
            manualPose=Double.isFinite(ShotExperimentSettings2026.TEST_X_INCHES)
                    ?new Pose(ShotExperimentSettings2026.TEST_X_INCHES,ShotExperimentSettings2026.TEST_Y_INCHES,
                        Math.toRadians(ShotExperimentSettings2026.TEST_HEADING_DEGREES)):null;
            aimX=ShotExperimentSettings2026.AIM_X_INCHES;aimY=ShotExperimentSettings2026.AIM_Y_INCHES;
            configureTestLogging();
            logging.start(getClass().getSimpleName(),"TUNING",shots,false);
            finished=false;lastResultEvent=0;manifestLogged=false;
            controller=new ShotExperimentController2026(plan,new Ports(),System.nanoTime(),gamepad1.y||gamepad2.y);
        } catch(RuntimeException|Error e) { shutdown();throw e; }
    }
    @Override public void onUpdate() {
        if(finished) { telemetry.addData("Shot test","COMPLETE — Stop; settings restored");telemetry.update();return; }
        try {
            long now=System.nanoTime();
            boolean stop=gamepad1.xWasPressed()||gamepad2.xWasPressed();
            boolean reverse=gamepad1.b||gamepad2.b;
            boolean skip=gamepad1.dpadRightWasPressed()||gamepad2.dpadRightWasPressed();
            controller.tick(now,gamepad1.y||gamepad2.y,stop,skip,reverse);
            if(controller.canReload()) {
                intake.updateTeleopCollectionInputs((gamepad1.aWasPressed()||gamepad2.aWasPressed())&&!unloaded(),
                        gamepad2.a&&!unloaded(),stop,reverse,false);
                if(unloaded()&&!reverse)intake.stopIntake();
            }
            updateLogContext(now);
            logging.setExperimentContext(context);
            logging.setShotContext(aimX,aimY,false,shots.isActive(),controller.getPhase()==ShotExperimentController2026.Phase.FEEDING);
            logging.update(manualPose,null,"SHOT_TEST/"+controller.getPhase(),plan.suite,"SHOT_TEST_FIXED_RPM");
            if(!manifestLogged) {logging.event("TEST_PLAN",plan.manifest());manifestLogged=true;}
            if(controller.getEventId()!=lastResultEvent) {
                logging.event("TEST_TRIAL_"+controller.getLastResult(),"trial="+controller.getLastResultTrialId()
                        +" attempt="+controller.getAttemptId()+" reason="+controller.getReason());
                lastResultEvent=controller.getEventId();
            }
            addTelemetry();
            if(controller.getPhase()==ShotExperimentController2026.Phase.COMPLETE) { shutdown();finished=true; }
        } catch(RuntimeException|Error e) { shutdown();throw e; }
    }
    private void configureTestLogging() {
        RobotLogging2026.ENABLE_MATCH_LOG=true;
        RobotLogging2026.ENABLE_SHOT_DETAIL_LOG=plan.detailedLogging;
        RobotLogging2026.DETAIL_SAMPLE_INTERVAL_MS=0;
    }
    private void updateLogContext(long now) {
        ShotExperimentPlan2026.Trial trial=controller.trialForLog();
        context.suite=plan.suite;context.runLabel=plan.runLabel;context.batteryCondition=plan.batteryCondition;
        context.trialId=trial.id;context.attemptId=controller.getAttemptId();context.repetition=trial.repetition;
        context.profile=context.attemptId==0?"NOT_STARTED":trial.profile.id;context.condition=trial.condition.name();
        context.phase=controller.getPhase().name();context.abortReason=controller.getReason();
        context.trialElapsed=controller.trialElapsed(now);context.phaseElapsed=controller.phaseElapsed(now);
        context.conditionElapsed=controller.conditionElapsed(now);context.initialTarget=trial.initialTargetRpm;
        context.finalTarget=trial.targetRpm;context.initialMeasured=controller.getInitialMeasuredRpm();
        context.rampSeconds=trial.rampSeconds;context.feedDelaySeconds=trial.feedDelaySeconds;
        context.feedEnabled=trial.feedEnabled;
        context.confirmedLoad=context.attemptId==0||!trial.feedEnabled?0:4;
    }
    private void addTelemetry() {
        ShotExperimentPlan2026.Trial upcoming=controller.trial();
        telemetry.addData("Test phase",controller.getPhase());
        telemetry.addData("Trial / total / attempt","%d / %d / %d",upcoming.id,plan.trials.size(),controller.getAttemptId());
        telemetry.addData("Selected profile / condition",upcoming.profile.id+" / "+upcoming.condition);
        telemetry.addData("Completed / skipped","%d / %d",controller.getCompletedCount(),controller.getSkippedCount());
        telemetry.addData("Initial / final target RPM","%.0f / %.0f",upcoming.initialTargetRpm,upcoming.targetRpm);
        telemetry.addData("Actual / target RPM","%.0f / %.0f",shooter.getMeasuredFlywheelRpm(),shooter.getTargetFlywheelRpm());
        telemetry.addData("Feed powers intake / transfer","%.3f / %.3f",IntakeWithSensorsSubsystem2026.SHOOT_INTAKE_POWER,IntakeWithSensorsSubsystem2026.SHOOT_TRANSFER_POWER);
        telemetry.addData("Tracked count / raw beams","%d / %d",intake.getBallCount(),intake.getCachedBeamMask());
        telemetry.addData("Battery raw / filtered","%.2f / %.2f V",shooter.getBatteryVoltageRaw(),shooter.getBatteryVoltageFiltered());
        if(controller.canReload())telemetry.addData("Next action",upcoming.feedEnabled?
                "Refill to FOUR total; release Y, then press Y. A loads; X stops; dpad right skips.":
                "Confirm EMPTY robot; release Y, then press Y. No automatic feeding. X stops; dpad right skips.");
        if(!controller.getReason().isEmpty())telemetry.addData("Pause reason",controller.getReason());
        logging.addTelemetry(telemetry);telemetry.update();
    }
    private void cue(LedPreset preset) {
        LEDControlSubsystem2026.STAGE_PULSES_ENABLED=true;
        LEDControlSubsystem2026.STAGE_PULSE_PRESET=preset;
        LEDControlSubsystem2026.STAGE_PULSE_ON_SECONDS=.15;
        LEDControlSubsystem2026.STAGE_PULSE_GAP_SECONDS=.10;
        LEDControlSubsystem2026.INSTANCE.signalStageChange();
    }
    private final class Ports implements ShotExperimentController2026.Ports {
        public double measuredRpm(){return shooter.getMeasuredFlywheelRpm();}
        public void applyProfile(ShotExperimentPlan2026.Profile profile){scope.apply(profile);configureTestLogging();RobotLogging2026.PROFILE_LABEL="SHOT_TEST/"+profile.id;}
        public void confirmFourLoaded(){intake.setBallCount(4);}
        public void stopCollection(){shots.cancel();intake.stopIntake();}
        public void spin(double rpm){shooter.spinUp(rpm);}
        public void stopShooter(){shooter.stop();}
        public void startFeed(){shots.start();shots.update();}
        public void updateFeed(){shots.update();}
        public boolean feedActive(){return shots.isActive();}
        public void stopAll(){shots.cancel();intake.stopIntake();shooter.stop();}
        public void trialCue(){cue(LedPreset.SOLID_GREEN);}
        public void feedCue(){cue(LedPreset.WHITE);}
    }
    private void shutdown() {
        try { logging.stop(); }
        finally {
            try { shots.cancel(); }
            finally {
                try { intake.stopAll(); }
                finally {
                    try { shooter.stop(); }
                    finally { if(scope!=null) { scope.close();scope=null; } }
                }
            }
        }
    }
    @Override public void onStop() {
        try { shutdown(); }
        finally { super.onStop(); }
    }
}
