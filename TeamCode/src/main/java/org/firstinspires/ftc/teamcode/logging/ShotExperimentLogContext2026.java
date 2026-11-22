package org.firstinspires.ftc.teamcode.logging;

/** Reused by a test OpMode; capture copies scalars into each queued frame. No hardware references. */
public final class ShotExperimentLogContext2026 {
    public String suite="",runLabel="",batteryCondition="",profile="",condition="",phase="",abortReason="";
    public int trialId,attemptId,repetition,confirmedLoad;
    public boolean feedEnabled=true;
    public double trialElapsed=Double.NaN,phaseElapsed=Double.NaN,conditionElapsed=Double.NaN;
    public double initialTarget=Double.NaN,finalTarget=Double.NaN,initialMeasured=Double.NaN;
    public double rampSeconds=Double.NaN,feedDelaySeconds=Double.NaN;
    public void copyTo(RobotLogFrame2026 frame) {
        frame.testSuite=suite;frame.testRunLabel=runLabel;frame.testBatteryCondition=batteryCondition;
        frame.testProfile=profile;frame.testCondition=condition;frame.testPhase=phase;frame.testAbortReason=abortReason;
        frame.testTrialId=trialId;frame.testAttemptId=attemptId;frame.testRepetition=repetition;frame.testConfirmedLoad=confirmedLoad;
        frame.testTrialElapsed=trialElapsed;frame.testPhaseElapsed=phaseElapsed;frame.testConditionElapsed=conditionElapsed;
        frame.testInitialTarget=initialTarget;frame.testFinalTarget=finalTarget;frame.testInitialMeasured=initialMeasured;
        frame.testRampSeconds=rampSeconds;frame.testFeedDelaySeconds=feedDelaySeconds;
        frame.testFeedEnabled=feedEnabled;
    }
}
