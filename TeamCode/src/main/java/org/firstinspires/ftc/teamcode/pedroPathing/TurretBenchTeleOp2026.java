package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem2026;

/** Standalone turret fixture. Its field position is fixed and only the Pinpoint yaw is used. */
@TeleOp(name = "Turret Bench 2026", group = "Test")
public final class TurretBenchTeleOp2026 extends OpMode {
    private static final double[] STEPS_DEG = {0.5, 1, 2, 5, 10, 15};
    private static final long REPEAT_NS = 200_000_000L;
    private GoBildaPinpointDriver pinpoint;
    private List<LynxModule> hubs;
    private TurretSubsystem2026 turret;
    private Pose startPose;
    private DeviceStatus status;
    private double fieldHeading;
    private boolean manual;
    private int stepIndex = 3;
    private int lastStickDirection;
    private long nextStep;

    @Override public void init() {
        startPose = Pickles2026Teleop.configuredStartingPose();
        if (!Double.isFinite(startPose.getX()) || !Double.isFinite(startPose.getY())
                || !Double.isFinite(startPose.getHeading())) {
            throw new IllegalArgumentException("Minimal TeleOp start pose must be finite");
        }
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        turret = TurretSubsystem2026.createBenchTurret(hardwareMap);
        hubs = hardwareMap.getAll(LynxModule.class);
        // Calibrate at the fixture's physical starting orientation. Add the field heading in software.
        pinpoint.resetPosAndIMU();
    }

    private void readSensors() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        pinpoint.update();
        status = pinpoint.getDeviceStatus();
        fieldHeading = Double.NaN;
        // A missing pod affects translation. An IMU fault or a bad I2C sample can affect yaw.
        if (status == DeviceStatus.READY || status == DeviceStatus.FAULT_NO_PODS_DETECTED
                || status == DeviceStatus.FAULT_X_POD_NOT_DETECTED || status == DeviceStatus.FAULT_Y_POD_NOT_DETECTED) {
            fieldHeading = AngleUnit.normalizeRadians(startPose.getHeading() + pinpoint.getHeading(AngleUnit.RADIANS));
        }
    }

    @Override public void init_loop() {
        readSensors();
        turret.periodic(); // Passive encoder readings until startTeleop().
        telemetry.addLine("Keep the Pinpoint still while calibrating. Play centers the turret.");
        showTelemetry(Pickles2026Teleop.calculateShotSettings(startPose));
    }

    @Override public void start() {
        manual = false;
        turret.startTeleop();
    }

    @Override public void loop() {
        readSensors();
        Pickles2026Teleop.ShotSettings shot = Pickles2026Teleop.calculateShotSettings(startPose);
        if (gamepad1.dpadUpWasPressed()) stepIndex = Math.min(stepIndex + 1, STEPS_DEG.length - 1);
        if (gamepad1.dpadDownWasPressed()) stepIndex = Math.max(stepIndex - 1, 0);
        if (gamepad1.aWasPressed()) {
            manual = !manual;
            if (manual && turret.canShoot()) turret.holdCommandedAngle();
            lastStickDirection = 0;
        }
        if (manual) {
            if (turret.canShoot()) adjustManualTarget();
        } else if (Double.isFinite(fieldHeading)) {
            turret.aimAt(shot.aimX, shot.aimY, startPose.getX(), startPose.getY(), fieldHeading);
        }
        turret.periodic();
        showTelemetry(shot);
    }

    private void adjustManualTarget() {
        double stick = gamepad1.right_stick_x;
        int direction = Math.abs(stick) >= 0.5 ? (int) Math.signum(stick) : 0;
        long now = System.nanoTime();
        if (direction != 0 && (direction != lastStickDirection || now >= nextStep)) {
            turret.stepTarget(direction * STEPS_DEG[stepIndex]);
            nextStep = now + REPEAT_NS;
        }
        lastStickDirection = direction;
    }

    private void showTelemetry(Pickles2026Teleop.ShotSettings shot) {
        telemetry.addData("Bench control", manual ? "MANUAL" : "TRACKING");
        telemetry.addData("Pinpoint status", status);
        telemetry.addData("Fixed X / Y (in)", "%.2f / %.2f", startPose.getX(), startPose.getY());
        telemetry.addData("Field heading (deg)", Math.toDegrees(fieldHeading));
        telemetry.addData("Hive X / Y (in)", "%.2f / %.2f", shot.aimX, shot.aimY);
        telemetry.addData("Manual step (deg)", STEPS_DEG[stepIndex]);
        telemetry.addLine("GP1: A switches modes. Right stick left/right steps angle. D-pad up/down changes step.");
        turret.addTelemetry(telemetry);
    }

    @Override public void stop() {
        // SDK may call stop after an interrupted Init, before a controller was constructed.
        if (turret != null) turret.stop();
    }
}
