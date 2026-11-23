package org.firstinspires.ftc.teamcode.testing.shots;

import com.bylazar.configurables.annotations.Configurable;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.firstinspires.ftc.teamcode.testing.shots.ShotExperimentPlan2026.*;

/** Opening repeatability and four-volley comparisons with an already-spinning wheel. */
@Configurable
public final class ShotSession2026 {
    private ShotSession2026() { }
    public static int VOLLEYS = 8;
    public enum RunningBatch {
        BOOST_COMPARISON(2), BOOST_TIMING(2), BOOST_LEVEL(2), FEED_SPEED(4), FEED_FAST(5), RPM_BRACKET(3);
        public final int settingsCount;
        RunningBatch(int settingsCount) {this.settingsCount=settingsCount;}
    }
    public static RunningBatch RUNNING_BATCH = RunningBatch.BOOST_COMPARISON;
    public static int VOLLEYS_PER_SETTING = 4;
    public static int RUNNING_ORDER_OFFSET = 0;
    public static double RPM_BRACKET_OFFSET = 100;
    public static double OPENING_RPM = 3100;
    public static double RUNNING_RPM = 3100;
    public static double READY_TOLERANCE_RPM = 75;
    public static double READY_STABLE_SECONDS = .15;
    public static double OPENING_READY_TIMEOUT_SECONDS = 2;
    public static double RUNNING_FEED_DELAY_SECONDS = 2;

    public static ShotExperimentPlan2026 build(boolean opening, ScopedShotSettings2026 scope) {
        ShotExperimentPlan2026.validateSettings();
        if(VOLLEYS<1||VOLLEYS>20||VOLLEYS_PER_SETTING<1||VOLLEYS_PER_SETTING>20
                ||!Double.isFinite(RPM_BRACKET_OFFSET)||RPM_BRACKET_OFFSET<=0||RUNNING_BATCH==null
                ||!Double.isFinite(READY_TOLERANCE_RPM)||READY_TOLERANCE_RPM<=0
                ||!Double.isFinite(READY_STABLE_SECONDS)||READY_STABLE_SECONDS<=0)
            throw new IllegalArgumentException("Select a running batch; shot counts must be 1..20 and readiness/RPM offsets positive and finite");
        String id="START6_6_FF90_110_ORIGINAL_BOOST";
        Profile candidate=RecommendedShotPlans2026.referenceProfile();
        Profile profile=candidate
                .shot(id,"BOOST_DELAY_SECONDS",.18)
                .shot(id,"ENABLE_RAMPED_BOOST",false)
                .shot(id,"ENABLE_BOOST_VOLTAGE_SCALING",false)
                .shot(id,"BOOST_OVERSPEED_TAPER_RPM",0.0);
        List<Trial> trials=new ArrayList<>();
        if(opening) {
            for(int n=1;n<=VOLLEYS;n++)
                trials.add(new Trial(n,n,profile,Condition.COLD,OPENING_RPM,0,0,
                        OPENING_READY_TIMEOUT_SECONDS,true,false,READY_TOLERANCE_RPM,READY_STABLE_SECONDS));
        } else {
            List<Profile> profiles;
            switch(RUNNING_BATCH) {
                case BOOST_COMPARISON:
                    profiles=Arrays.asList(profile,candidate);break;
                case BOOST_TIMING:
                    profiles=Arrays.asList(candidate,candidate.shot("DELAY_08","BOOST_DELAY_SECONDS",.08));break;
                case BOOST_LEVEL:
                    profiles=Arrays.asList(candidate,candidate.shot("BOOST_75_85","BOOST_START_POWER",.75)
                            .shot("BOOST_75_85","BOOST_END_POWER",.85));break;
                case FEED_SPEED:
                    profiles=Arrays.asList(candidate,candidate.intake("TRANSFER_425","SHOOT_TRANSFER_POWER",.425),
                            candidate.intake("INTAKE_425","SHOOT_INTAKE_POWER",.425),
                            candidate.intake("BOTH_425","SHOOT_INTAKE_POWER",.425)
                                    .intake("BOTH_425","SHOOT_TRANSFER_POWER",.425));break;
                case FEED_FAST:
                    profiles=Arrays.asList(candidate,candidate.intake("TRANSFER_575","SHOOT_TRANSFER_POWER",.575),
                            candidate.intake("INTAKE_575","SHOOT_INTAKE_POWER",.575),
                            candidate.intake("BOTH_575","SHOOT_INTAKE_POWER",.575)
                                    .intake("BOTH_575","SHOOT_TRANSFER_POWER",.575),
                            candidate.intake("BOTH_650","SHOOT_INTAKE_POWER",.65)
                                    .intake("BOTH_650","SHOOT_TRANSFER_POWER",.65));break;
                case RPM_BRACKET:
                    profiles=Arrays.asList(candidate,candidate,candidate);break;
                default:throw new IllegalArgumentException("Select a running batch");
            }
            for(int group=0;group<profiles.size();group++) {
                int option=Math.floorMod(group+RUNNING_ORDER_OFFSET,profiles.size());
                double rpm=RUNNING_RPM+(RUNNING_BATCH==RunningBatch.RPM_BRACKET?
                        (option-1)*RPM_BRACKET_OFFSET:0);
                for(int repeat=1;repeat<=VOLLEYS_PER_SETTING;repeat++)
                    trials.add(new Trial(trials.size()+1,repeat,profiles.get(option),Condition.ALREADY_SPINNING,
                            rpm,0,0,RUNNING_FEED_DELAY_SECONDS,true,false));
            }
        }
        try { for(Trial trial:trials)scope.apply(trial.profile); }
        finally { scope.resetToBaseline(); }
        return new ShotExperimentPlan2026(trials,opening?"OPENING_REPEAT":"RUNNING_"+RUNNING_BATCH,
                ShotExperimentSettings2026.RUNNING_PREPARE_SECONDS,ShotExperimentSettings2026.POST_FEED_CAPTURE_SECONDS,0);
    }
}
