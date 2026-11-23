package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.testing.shots.*;

@Autonomous(name="Shot Opening Repeat 2026",group="Test")
public final class ShooterOpening2026 extends ShooterExperimentOpMode2026 {
    @Override protected boolean comparison() {return false;}
    @Override protected boolean startPreloaded() {return true;}
    @Override protected String programLabel() {return "PRELOADED COLD START REPEATABILITY";}
    @Override protected Object selectedGroup() {return "ShotSession2026 opening";}
    @Override protected double displayedTargetRpm() {return ShotSession2026.OPENING_RPM;}
    @Override protected ShotExperimentPlan2026 buildPlan(ScopedShotSettings2026 settings) {
        return ShotSession2026.build(true,settings);
    }
}
