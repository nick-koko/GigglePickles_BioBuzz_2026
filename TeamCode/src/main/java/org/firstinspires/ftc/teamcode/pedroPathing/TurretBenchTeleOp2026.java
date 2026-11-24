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
    private static final double[] SWEEP_SPEEDS_DEG_PER_SECOND = {15, 30, 60, 90, 120, 180, 270, 360};
    private static final double STICK_DEADBAND = 0.1;
    private GoBildaPinpointDriver pinpoint;
    private List<LynxModule> hubs;
    private TurretSubsystem2026 turret;
    private Pose startPose;
    private DeviceStatus status;
    private double fieldHeading;
    private boolean manual;
    private int speedIndex = 2;
    private long lastLoopNanos;

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
        lastLoopNanos = System.nanoTime();
        turret.startTeleop();
    }

    @Override public void loop() {
        readSensors();
        long now = System.nanoTime();
        double dt = Math.min(0.1, Math.max(0, (now - lastLoopNanos) / 1e9));
        lastLoopNanos = now;
        Pickles2026Teleop.ShotSettings shot = Pickles2026Teleop.calculateShotSettings(startPose);
        if (gamepad1.dpadUpWasPressed()) speedIndex = Math.min(speedIndex + 1, SWEEP_SPEEDS_DEG_PER_SECOND.length - 1);
        if (gamepad1.dpadDownWasPressed()) speedIndex = Math.max(speedIndex - 1, 0);
        if (gamepad1.aWasPressed()) {
            manual = !manual;
            if (manual && turret.canShoot()) turret.holdCommandedAngle();
        }
        if (manual) {
            if (turret.canShoot()) adjustManualTarget(dt);
        } else if (Double.isFinite(fieldHeading)) {
            turret.aimAt(shot.aimX, shot.aimY, startPose.getX(), startPose.getY(), fieldHeading);
        }
        turret.periodic();
        showTelemetry(shot);
    }

    private void adjustManualTarget(double dt) {
        double stick = gamepad1.right_stick_x;
        if (Math.abs(stick) <= STICK_DEADBAND) return;
        double input = Math.copySign((Math.abs(stick) - STICK_DEADBAND) / (1 - STICK_DEADBAND), stick);
        turret.stepTarget(input * SWEEP_SPEEDS_DEG_PER_SECOND[speedIndex] * dt);
    }

    private void showTelemetry(Pickles2026Teleop.ShotSettings shot) {
        telemetry.addData("Bench control", manual ? "MANUAL" : "TRACKING");
        telemetry.addData("Pinpoint status", status);
        telemetry.addData("Fixed X / Y (in)", "%.2f / %.2f", startPose.getX(), startPose.getY());
        telemetry.addData("Field heading (deg)", Math.toDegrees(fieldHeading));
        telemetry.addData("Hive X / Y (in)", "%.2f / %.2f", shot.aimX, shot.aimY);
        telemetry.addData("Manual sweep speed (deg/s)", SWEEP_SPEEDS_DEG_PER_SECOND[speedIndex]);
        telemetry.addLine("GP1: A switches modes. Hold right stick left/right to sweep. D-pad up/down changes speed.");
        turret.addTelemetry(telemetry);
    }

    @Override public void stop() {
        // SDK may call stop after an interrupted Init, before a controller was constructed.
        if (turret != null) turret.stop();
    }
}
