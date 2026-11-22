package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.testing.shots.*;

@Autonomous(name="Flywheel Model 2026",group="Test")
public final class FlywheelModel2026 extends ShooterExperimentOpMode2026 {
    @Override protected boolean comparison() {return true;}
    @Override protected boolean unloaded() {return true;}
    @Override protected String programLabel() {return "UNLOADED FLYWHEEL MODEL DATA";}
    @Override protected Object selectedGroup() {return ShotExperimentSettings2026.MODEL_BATCH;}
    @Override protected ShotExperimentPlan2026 buildPlan(ScopedShotSettings2026 settings) {
        return RecommendedShotPlans2026.model(settings);
    }
}
