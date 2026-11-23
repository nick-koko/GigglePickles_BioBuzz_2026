package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.testing.shots.*;

@Autonomous(name="Shot Running Batches 2026",group="Test")
public final class ShooterRunning2026 extends ShooterExperimentOpMode2026 {
    private boolean previousUp,previousDown,previousLeft,previousRight;
    @Override protected boolean comparison() {return false;}
    @Override protected String programLabel() {return "ALREADY-SPINNING FOUR-VOLLEY GROUPS";}
    @Override protected Object selectedGroup() {return ShotSession2026.RUNNING_BATCH;}
    @Override protected double displayedTargetRpm() {return ShotSession2026.RUNNING_RPM;}
    @Override protected void updateInitSelection() {
        boolean up=gamepad1.dpad_up||gamepad2.dpad_up;
        boolean down=gamepad1.dpad_down||gamepad2.dpad_down;
        boolean left=gamepad1.dpad_left||gamepad2.dpad_left;
        boolean right=gamepad1.dpad_right||gamepad2.dpad_right;
        if((up&&!previousUp)||(down&&!previousDown)) {
            ShotSession2026.RunningBatch[] batches=ShotSession2026.RunningBatch.values();
            int direction=up&&!previousUp?1:-1;
            ShotSession2026.RUNNING_BATCH=batches[Math.floorMod(ShotSession2026.RUNNING_BATCH.ordinal()+direction,batches.length)];
            ShotSession2026.RUNNING_ORDER_OFFSET=0;
        }
        if(left&&!previousLeft)ShotSession2026.RUNNING_ORDER_OFFSET=0;
        if(right&&!previousRight)ShotSession2026.RUNNING_ORDER_OFFSET=1;
        previousUp=up;previousDown=down;previousLeft=left;previousRight=right;
    }
    @Override protected void addSelectionTelemetry() {
        telemetry.addData("Select during Init","D-pad up/down: batch; left: default order; right: rotated order");
        telemetry.addData("Group order",ShotSession2026.RUNNING_ORDER_OFFSET==0?"DEFAULT":"ROTATED");
        telemetry.addData("Automatic volley count",ShotSession2026.RUNNING_BATCH.settingsCount*ShotSession2026.VOLLEYS_PER_SETTING);
    }
    @Override protected ShotExperimentPlan2026 buildPlan(ScopedShotSettings2026 settings) {
        return ShotSession2026.build(false,settings);
    }
}
