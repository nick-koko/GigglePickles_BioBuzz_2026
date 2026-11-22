package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.testing.shots.*;

@Autonomous(name="Shot Recommended 2026",group="Test")
public final class ShooterRecommended2026 extends ShooterExperimentOpMode2026 {
    @Override protected boolean comparison() {return true;}
    @Override protected String programLabel() {return "RECOMMENDED PARAMETER BATCH";}
    @Override protected Object selectedGroup() {return ShotExperimentSettings2026.RECOMMENDED_BATCH;}
    @Override protected ShotExperimentPlan2026 buildPlan(ScopedShotSettings2026 settings) {
        return RecommendedShotPlans2026.recommended(settings);
    }
}
