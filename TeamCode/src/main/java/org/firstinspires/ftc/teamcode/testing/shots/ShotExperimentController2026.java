package org.firstinspires.ftc.teamcode.testing.shots;

/** Nonblocking sequencer. Loaded trials require confirmation; unloaded trials auto-advance after initial confirmation. */
public final class ShotExperimentController2026 {
    public enum Phase { WAITING_FOR_LOAD, COASTING_TO_STOP, PREPARING_INITIAL_RPM, TARGET_RAMP,
        WAITING_TO_FEED, FEEDING, RECOVERY, UNLOADED_HOLD, COAST_CAPTURE, BETWEEN_UNLOADED_TRIALS, PAUSED, COMPLETE }
    public interface Ports {
        double measuredRpm();
        void applyProfile(ShotExperimentPlan2026.Profile profile);
        void confirmFourLoaded();
        void stopCollection();
        void spin(double rpm);
        void stopShooter();
        void startFeed();
        void updateFeed();
        boolean feedActive();
        void stopAll();
        void trialCue();
        void feedCue();
    }
    public final ShotExperimentPlan2026 plan;
    private final Ports ports;
    private int index,attempt,completed,skipped;
    private Phase phase=Phase.WAITING_FOR_LOAD;
    private long trialStart,phaseStart,conditionStart,stoppedSince,readySince;
    private boolean previousReady;
    private double initialMeasuredRpm=Double.NaN;
    private String reason="";
    private ShotExperimentPlan2026.Trial activeTrial;
    private int eventId,lastResultTrialId;
    private String lastResult="";

    public ShotExperimentController2026(ShotExperimentPlan2026 plan,Ports ports,long now) {
        this(plan,ports,now,false);
    }
    public ShotExperimentController2026(ShotExperimentPlan2026 plan,Ports ports,long now,boolean readyAlreadyHeld) {
        this.plan=plan;this.ports=ports;phaseStart=now;
        previousReady=readyAlreadyHeld;
        if(plan.trials.isEmpty())throw new IllegalArgumentException("Empty shot experiment plan");
    }
    public void tick(long now,boolean readyDown,boolean abortPressed,boolean skipPressed,boolean reverseHeld) {
        boolean ready=readyDown&&!previousReady;previousReady=readyDown;
        if(phase==Phase.COMPLETE)return;
        if(abortPressed) { pause(now,"OPERATOR_STOP");return; }
        if(reverseHeld && !canReload()) { pause(now,"MANUAL_REVERSE");return; }
        if(canReload()) {
            if(skipPressed) { skipped++;result("SKIPPED",trial().id);advance(now,false);return; }
            if(ready && !reverseHeld)begin(now);
            return;
        }
        ShotExperimentPlan2026.Trial trial=trial();
        switch(phase) {
            case BETWEEN_UNLOADED_TRIALS:
                if(seconds(now,phaseStart)>=plan.unloadedBetweenSeconds)begin(now);
                break;
            case COASTING_TO_STOP:
                if(!Double.isFinite(ports.measuredRpm()) || Math.abs(ports.measuredRpm())>plan.stoppedRpm)stoppedSince=0;
                else {
                    if(stoppedSince==0)stoppedSince=now;
                    if(seconds(now,stoppedSince)>=plan.stoppedStableSeconds) { beginCondition(now);break; }
                }
                if(seconds(now,phaseStart)>=plan.coastTimeoutSeconds)pause(now,"COAST_TIMEOUT");
                break;
            case PREPARING_INITIAL_RPM:
                ports.spin(trial.initialTargetRpm);
                if(seconds(now,phaseStart)>=plan.prepareSeconds)beginCondition(now);
                break;
            case TARGET_RAMP:
            case WAITING_TO_FEED:
                double elapsed=seconds(now,conditionStart);
                double fraction=trial.rampSeconds>0?Math.min(1,elapsed/trial.rampSeconds):1;
                ports.spin(trial.condition==ShotExperimentPlan2026.Condition.COLD?trial.targetRpm:
                        trial.initialTargetRpm+fraction*(trial.targetRpm-trial.initialTargetRpm));
                if(phase==Phase.TARGET_RAMP && fraction>=1)setPhase(Phase.WAITING_TO_FEED,now);
                boolean readyToFeed=elapsed>=trial.feedDelaySeconds;
                if(trial.readyToleranceRpm>0) {
                    double measured=ports.measuredRpm();
                    if(fraction<1 || !Double.isFinite(measured) || Math.abs(measured-trial.targetRpm)>trial.readyToleranceRpm)
                        readySince=0;
                    else if(readySince==0)readySince=now;
                    readyToFeed=readySince!=0 && seconds(now,readySince)>=trial.readyStableSeconds;
                    if(!readyToFeed && elapsed>=trial.feedDelaySeconds) { pause(now,"RPM_READY_TIMEOUT");break; }
                }
                if(readyToFeed) {
                    ports.spin(trial.targetRpm);
                    if(trial.feedEnabled) {
                        ports.feedCue();ports.startFeed();setPhase(Phase.FEEDING,now);
                    } else setPhase(Phase.UNLOADED_HOLD,now);
                }
                break;
            case FEEDING:
                ports.spin(trial.targetRpm);ports.updateFeed();
                if(!ports.feedActive()) { ports.stopCollection();setPhase(Phase.RECOVERY,now); }
                break;
            case RECOVERY:
                ports.spin(trial.targetRpm);
                if(seconds(now,phaseStart)>=plan.recoverySeconds)finish(now);
                break;
            case UNLOADED_HOLD:
                ports.spin(trial.targetRpm);
                if(seconds(now,phaseStart)>=plan.unloadedHoldSeconds) {
                    if(trial.recordCoast) {
                        ports.stopShooter();stoppedSince=0;setPhase(Phase.COAST_CAPTURE,now);
                    } else finish(now);
                }
                break;
            case COAST_CAPTURE:
                double rpm=ports.measuredRpm();
                if(!Double.isFinite(rpm)||Math.abs(rpm)>plan.stoppedRpm)stoppedSince=0;
                else {
                    if(stoppedSince==0)stoppedSince=now;
                    if(seconds(now,stoppedSince)>=plan.stoppedStableSeconds) { finish(now);break; }
                }
                if(seconds(now,phaseStart)>=plan.coastTimeoutSeconds)pause(now,"COAST_CAPTURE_TIMEOUT");
                break;
            default:throw new IllegalStateException("Unexpected shot test phase "+phase);
        }
    }
    private void begin(long now) {
        ports.stopCollection();ports.applyProfile(trial().profile);
        if(trial().feedEnabled)ports.confirmFourLoaded();
        ports.trialCue();
        activeTrial=trial();attempt++;trialStart=now;conditionStart=0;initialMeasuredRpm=Double.NaN;stoppedSince=0;reason="";
        if(trial().condition==ShotExperimentPlan2026.Condition.COLD) {
            ports.stopShooter();setPhase(Phase.COASTING_TO_STOP,now);
        } else {
            ports.spin(trial().initialTargetRpm);setPhase(Phase.PREPARING_INITIAL_RPM,now);
        }
    }
    private void beginCondition(long now) {
        conditionStart=now;readySince=0;initialMeasuredRpm=ports.measuredRpm();
        ports.spin(trial().rampSeconds>0?trial().initialTargetRpm:trial().targetRpm);
        setPhase(trial().rampSeconds>0?Phase.TARGET_RAMP:Phase.WAITING_TO_FEED,now);
    }
    private void advance(long now,boolean completedNormally) {
        boolean unloadedCompleted=completedNormally&&!trial().feedEnabled;
        ports.stopCollection();index++;
        if(index>=plan.trials.size()) { ports.stopAll();setPhase(Phase.COMPLETE,now);return; }
        if(trial().condition==ShotExperimentPlan2026.Condition.COLD)ports.stopShooter();
        // Coast during reload where possible; keep the motor running for warm trials.
        stoppedSince=0;reason="";
        // Keep a distinct phase so the completed trial/result is logged before the next attempt begins.
        setPhase(unloadedCompleted&&!trial().feedEnabled?Phase.BETWEEN_UNLOADED_TRIALS:Phase.WAITING_FOR_LOAD,now);
    }
    private void finish(long now) { completed++;result("COMPLETE",trial().id);advance(now,true); }
    private void pause(long now,String reason) {
        ports.stopAll();this.reason=reason;result("ABORTED",trial().id);setPhase(Phase.PAUSED,now);
    }
    private void result(String outcome,int trialId) { lastResult=outcome;lastResultTrialId=trialId;eventId++; }
    private void setPhase(Phase next,long now) { phase=next;phaseStart=now; }
    private static double seconds(long now,long then) { return Math.max(0,(now-then)/1e9); }
    public boolean canReload() { return phase==Phase.WAITING_FOR_LOAD||phase==Phase.PAUSED; }
    public Phase getPhase() { return phase; }
    public ShotExperimentPlan2026.Trial trial() { return plan.trials.get(Math.min(index,plan.trials.size()-1)); }
    public int getAttemptId() { return attempt; }
    public ShotExperimentPlan2026.Trial trialForLog() { return activeTrial==null?trial():activeTrial; }
    public int getEventId() { return eventId; }
    public int getLastResultTrialId() { return lastResultTrialId; }
    public String getLastResult() { return lastResult; }
    public int getCompletedCount() { return completed; }
    public int getSkippedCount() { return skipped; }
    public String getReason() { return reason; }
    public double getInitialMeasuredRpm() { return initialMeasuredRpm; }
    public double trialElapsed(long now) { return trialStart==0?Double.NaN:seconds(now,trialStart); }
    public double phaseElapsed(long now) { return seconds(now,phaseStart); }
    public double conditionElapsed(long now) { return conditionStart==0?Double.NaN:seconds(now,conditionStart); }
}
