package org.firstinspires.ftc.teamcode.util;

/** Turret control and handoff logic. Hardware writes are gated independently of calibration. */
public final class TurretController2026 {
    public enum Mode { CHASSIS_ONLY, FIXED, TRACKING }
    public enum State { DISABLED, WAITING_FOR_PLAY, CENTERING, READY, FAULT }

    public interface Hardware {
        int encoderTicks();
        double absoluteVoltage();
        void writeServos(double left, double right);
    }

    /** Geometry is captured at Init; the subsystem may update the control tuning live. */
    public static final class Config {
        public String hardwareKey = "";
        public double encoderCpr = 4096.0;
        public double encoderRatio = 81.0 / 45.0;
        public double quadSign = -1.0;
        public double absoluteSign = 1.0;
        public double absoluteMaxVoltage = 3.255;
        public double absoluteOffset = 0.0;
        public double minAngle = -130.0;
        public double maxAngle = 130.0;
        public double servoCenter = 0.5;
        public double servoTravel = 355.0;
        public double minServo = 0.0;
        public double maxServo = 1.0;
        public double servoZeroOffset = 0.0;
        public double differential = 2.0;
        public double frontOffset = 0.0;
        public double robotRelativeSign = 1.0;
        public double centerAngle = 0.0;
        public double settleSeconds = 2.0;
        public double timeoutSeconds = 6.0;
        public double maxCenterError = 20.0;
        public double handoffTolerance = 5.0;
        public double readyTolerance = 2.0;
        public double readySpeed = 15.0;
        public int readyLoops = 3;
        public double rateLimit = 532.5;
        public double outerKp = 0.12;
        public double maxTrim = 8.0;
        public double fixedAngle = 0.0;
        public double autonomousAngle = 0.0;

        public double countsPerDegree() { return encoderCpr * encoderRatio / 360.0; }

        private String calibrationKey() {
            return hardwareKey + ":" + encoderCpr + ":" + encoderRatio + ":" + quadSign
                    + ":" + absoluteSign + ":" + absoluteMaxVoltage + ":" + absoluteOffset
                    + ":" + minAngle + ":" + maxAngle + ":" + servoCenter + ":" + servoTravel
                    + ":" + minServo + ":" + maxServo + ":" + servoZeroOffset + ":" + differential
                    + ":" + frontOffset + ":" + robotRelativeSign;
        }

        private boolean isValid() {
            double[] values = {encoderCpr, encoderRatio, quadSign, absoluteSign, absoluteMaxVoltage,
                    absoluteOffset, minAngle, maxAngle, servoCenter, servoTravel, minServo, maxServo,
                    servoZeroOffset, differential, frontOffset, robotRelativeSign, centerAngle,
                    settleSeconds, timeoutSeconds, maxCenterError, handoffTolerance, readyTolerance,
                    readySpeed, rateLimit, outerKp, maxTrim, fixedAngle, autonomousAngle};
            for (double value : values) if (!Double.isFinite(value)) return false;
            return encoderCpr > 0 && encoderRatio > 0 && servoTravel > 0 && absoluteMaxVoltage > 0
                    && Math.abs(quadSign) == 1 && Math.abs(absoluteSign) == 1
                    && Math.abs(robotRelativeSign) == 1 && minAngle < maxAngle
                    && minServo >= 0 && maxServo <= 1 && minServo < maxServo
                    && centerAngle >= minAngle && centerAngle <= maxAngle
                    && settleSeconds >= 0 && timeoutSeconds > settleSeconds && maxCenterError >= 0
                    && handoffTolerance >= 0 && readyTolerance >= 0 && readySpeed >= 0
                    && readyLoops > 0 && rateLimit > 0 && maxTrim >= 0;
        }
    }

    /** A reference, not an assumption that the turret stopped at this angle. */
    public static final class Snapshot {
        public final String calibrationKey;
        public final int encoderTicks;
        public final double measuredAngle, quadratureOffset, servoOffset, absoluteRaw;
        public final double leftPosition, rightPosition;

        private Snapshot(String key, int ticks, double measured, double quadOffset,
                         double servoOffset, double raw, double left, double right) {
            calibrationKey = key;
            encoderTicks = ticks;
            measuredAngle = measured;
            quadratureOffset = quadOffset;
            this.servoOffset = servoOffset;
            absoluteRaw = raw;
            leftPosition = left;
            rightPosition = right;
        }
    }

    private final Mode mode;
    private final Config config;
    private final Hardware hardware;
    private State state;
    private boolean outputsAllowed, calibrated, autonomous, targetReachable = true;
    private boolean restoredFromAutonomous;
    private String status;
    private Snapshot handoff;
    private int ticks, previousTicks, stableLoops, readyLoops;
    private double measured = Double.NaN, velocity, quadratureOffset, servoOffset;
    private double target, commanded, leftPosition = Double.NaN, rightPosition = Double.NaN;
    private double raw = Double.NaN, centerRawAverage = Double.NaN;
    private double previousTime, centerStarted, fixedStarted;

    public TurretController2026(Mode mode, Config config, Hardware hardware, double now) {
        this.mode = mode;
        this.config = config;
        this.hardware = hardware;
        previousTime = now;
        servoOffset = config.servoZeroOffset;
        state = mode == Mode.CHASSIS_ONLY ? State.DISABLED : State.WAITING_FOR_PLAY;
        status = mode == Mode.CHASSIS_ONLY ? "Chassis aiming; turret hardware unused"
                : "Waiting for Play";
        if (mode != Mode.CHASSIS_ONLY && !config.isValid()) {
            fail("Invalid turret configuration");
        } else if (mode == Mode.TRACKING) {
            ticks = previousTicks = hardware.encoderTicks();
            raw = readRaw();
        }
    }

    /** Observation only: never writes servos or resets an encoder. */
    public boolean restore(Snapshot snapshot) {
        if (mode != Mode.TRACKING || state == State.FAULT || snapshot == null) return false;
        ticks = previousTicks = hardware.encoderTicks();
        raw = readRaw();
        if (!config.calibrationKey().equals(snapshot.calibrationKey) || !matches(snapshot)) {
            status = "Handoff rejected; calibration will run after Play";
            return false;
        }
        quadratureOffset = snapshot.quadratureOffset;
        servoOffset = snapshot.servoOffset;
        measured = quadAngle(ticks) - quadratureOffset;
        target = commanded = measured;
        leftPosition = snapshot.leftPosition;
        rightPosition = snapshot.rightPosition;
        handoff = snapshot;
        calibrated = restoredFromAutonomous = true;
        status = "Autonomous reference restored; outputs blocked until Play";
        return true;
    }

    /** Call in autonomous Init, or in teleop only after Play. */
    public void start(double now, boolean autonomous) {
        if (state == State.DISABLED || state == State.FAULT || outputsAllowed) return;
        this.autonomous = autonomous;
        update(now); // Validate the handoff once more before enabling any output.
        if (state == State.FAULT) return;
        previousTime = now;
        outputsAllowed = true;
        handoff = null;
        if (mode == Mode.FIXED) {
            state = State.READY;
            status = "Fixed servo hold (no encoder required)";
            target = commanded = clip(config.fixedAngle, config.minAngle, config.maxAngle);
            fixedStarted = now;
            apply(commanded);
        } else if (calibrated) {
            state = State.READY;
            target = commanded = measured;
            if (autonomous) setTarget(config.autonomousAngle);
            status = "Continuous quadrature reference";
        } else {
            state = State.CENTERING;
            status = "Centering and calibrating";
            centerStarted = now;
            stableLoops = 0;
            centerRawAverage = Double.NaN;
            target = commanded = config.centerAngle;
            apply(commanded); // Includes the same opposing preload used during normal control.
        }
    }

    public void update(double now) {
        double dt = Math.max(0.0, now - previousTime);
        previousTime = now;
        if (state == State.DISABLED || state == State.FAULT) return;
        if (!Double.isFinite(config.rateLimit) || config.rateLimit <= 0
                || !Double.isFinite(config.outerKp) || config.outerKp < 0
                || !Double.isFinite(config.maxTrim) || config.maxTrim < 0
                || !Double.isFinite(config.fixedAngle)) {
            fail("Invalid live turret control settings");
            return;
        }
        if (mode == Mode.TRACKING) {
            ticks = hardware.encoderTicks();
            int deltaTicks = ticks - previousTicks;
            previousTicks = ticks;
            velocity = dt > 1e-6 ? deltaTicks * config.quadSign / config.countsPerDegree() / dt : 0;
            raw = readRaw();
            if (calibrated) measured = quadAngle(ticks) - quadratureOffset;
            if (!outputsAllowed && handoff != null && !matches(handoff)) {
                calibrated = restoredFromAutonomous = false;
                measured = Double.NaN;
                handoff = null;
                status = "Encoder reference changed; calibration will run after Play";
            }
        }
        if (!outputsAllowed) return;

        if (state == State.CENTERING) {
            boolean stable = Math.abs(velocity) <= config.readySpeed && Double.isFinite(raw);
            if (stable) {
                centerRawAverage = Double.isFinite(centerRawAverage)
                        ? wrap360(centerRawAverage + 0.25 * wrap180(raw - centerRawAverage)) : raw;
            } else {
                // Do not let samples taken during the sweep bias the final zero estimate.
                centerRawAverage = Double.NaN;
            }
            stableLoops = stable ? stableLoops + 1 : 0;
            if (now - centerStarted >= config.settleSeconds && stableLoops >= config.readyLoops) {
                double resolved = resolveAbsolute(centerRawAverage, config.centerAngle,
                        config.encoderRatio, config.absoluteSign, config.absoluteOffset,
                        config.minAngle, config.maxAngle);
                if (!Double.isFinite(resolved)
                        || Math.abs(resolved - config.centerAngle) > config.maxCenterError) {
                    fail("Center angle disagrees with analog encoder; check phase/zero offset");
                    return;
                }
                quadratureOffset = quadAngle(ticks) - resolved;
                // This learns servo zero ONLY after an intentional known-position command.
                servoOffset = config.servoZeroOffset + resolved - config.centerAngle;
                measured = target = commanded = resolved;
                calibrated = true;
                state = State.READY;
                status = "Calibrated from center; using quadrature";
                readyLoops = 0;
                if (autonomous) setTarget(config.autonomousAngle);
            } else if (now - centerStarted >= config.timeoutSeconds) {
                fail("Calibration timed out; check encoder and turret motion");
            }
            return;
        }

        if (mode == Mode.FIXED) {
            double fixed = clip(config.fixedAngle, config.minAngle, config.maxAngle);
            if (target != fixed) fixedStarted = now;
            setTarget(config.fixedAngle);
        }
        if (mode == Mode.TRACKING && (!Double.isFinite(measured)
                || measured < config.minAngle - config.maxCenterError
                || measured > config.maxAngle + config.maxCenterError)) {
            fail("Quadrature position outside turret travel; recalibration required");
            return;
        }
        double step = Math.max(0, config.rateLimit) * Math.min(dt, 0.1);
        double previousCommand = commanded;
        commanded += clip(target - commanded, -step, step);
        if (mode == Mode.FIXED && previousCommand != commanded) fixedStarted = now;
        double trim = mode == Mode.TRACKING
                ? clip(config.outerKp * (target - measured), -config.maxTrim, config.maxTrim) : 0;
        apply(clip(commanded + trim, config.minAngle, config.maxAngle));
        readyLoops = mode == Mode.TRACKING && targetReachable
                && Math.abs(target - measured) <= config.readyTolerance
                && Math.abs(velocity) <= config.readySpeed ? readyLoops + 1 : 0;
    }

    public void setTarget(double angle) {
        if (!Double.isFinite(angle)) return;
        target = clip(angle, config.minAngle, config.maxAngle);
        targetReachable = target == angle;
    }

    /** Aim using the chassis pose, while leaving all chassis stick inputs untouched. */
    public void aim(double fieldBearingDegrees, double robotHeadingDegrees) {
        if (mode != Mode.TRACKING || !calibrated || !outputsAllowed || state != State.READY) return;
        double base = config.robotRelativeSign * wrap180(fieldBearingDegrees - robotHeadingDegrees)
                + config.frontOffset;
        double candidate = nearestEquivalent(base, target, config.minAngle, config.maxAngle, 360);
        if (Double.isFinite(candidate)) {
            setTarget(candidate);
        } else {
            target = Math.abs(wrap180(base - config.minAngle))
                    <= Math.abs(wrap180(base - config.maxAngle)) ? config.minAngle : config.maxAngle;
            targetReachable = false;
        }
    }

    /** Reads a fresh counter/phase pair; does not move the mechanism. */
    public Snapshot snapshot() {
        if (mode != Mode.TRACKING || !calibrated || state != State.READY) return null;
        int current = hardware.encoderTicks();
        double angle = quadAngle(current) - quadratureOffset;
        double phase = readRaw();
        if (!Double.isFinite(angle) || !Double.isFinite(phase)
                || angle < config.minAngle || angle > config.maxAngle) return null;
        return new Snapshot(config.calibrationKey(), current, angle, quadratureOffset,
                servoOffset, phase, leftPosition, rightPosition);
    }

    public void stop() { outputsAllowed = false; readyLoops = 0; }
    public void fail(String reason) {
        state = State.FAULT;
        calibrated = outputsAllowed = false;
        status = reason;
    }

    private boolean matches(Snapshot reference) {
        double angle = quadAngle(ticks) - reference.quadratureOffset;
        double movement = (ticks - reference.encoderTicks) * config.quadSign / config.countsPerDegree();
        double expectedPhaseChange = movement * config.encoderRatio / config.absoluteSign;
        double phaseError = Math.abs(wrap180(raw - reference.absoluteRaw - expectedPhaseChange))
                / config.encoderRatio;
        return Double.isFinite(angle) && Double.isFinite(phaseError)
                && angle >= config.minAngle && angle <= config.maxAngle
                && phaseError <= config.handoffTolerance;
    }

    private double quadAngle(int count) { return count * config.quadSign / config.countsPerDegree(); }
    private double readRaw() {
        double voltage = hardware.absoluteVoltage();
        return !Double.isFinite(voltage) || voltage < 0 || voltage > config.absoluteMaxVoltage + 0.1
                ? Double.NaN : wrap360(voltage / config.absoluteMaxVoltage * 360);
    }

    private void apply(double angle) {
        if (!outputsAllowed) return;
        double half = config.differential / 2;
        leftPosition = position(clip(angle + half, config.minAngle, config.maxAngle));
        rightPosition = position(clip(angle - half, config.minAngle, config.maxAngle));
        hardware.writeServos(leftPosition, rightPosition);
    }

    private double position(double angle) {
        return clip(config.servoCenter - (angle - servoOffset) / config.servoTravel,
                config.minServo, config.maxServo);
    }

    public Mode getMode() { return mode; }
    public State getState() { return state; }
    public String getStatus() { return status; }
    public boolean isCalibrated() { return calibrated; }
    public boolean isRestoredFromAutonomous() { return restoredFromAutonomous; }
    public boolean canShoot(double now) {
        return mode == Mode.CHASSIS_ONLY || (outputsAllowed && state == State.READY
                && (mode != Mode.FIXED || now - fixedStarted >= config.settleSeconds));
    }
    public boolean isAimReady() { return readyLoops >= config.readyLoops; }
    public boolean isTargetReachable() { return targetReachable; }
    public double getMeasuredAngle() { return measured; }
    public double getVelocity() { return velocity; }
    public double getTarget() { return target; }
    public double getRawDegrees() { return raw; }
    public double getServoOffset() { return servoOffset; }
    public double getLeftPosition() { return leftPosition; }
    public double getRightPosition() { return rightPosition; }
    public double getFixedRobotRelativeHeading() {
        return (clip(config.fixedAngle, config.minAngle, config.maxAngle) - config.frontOffset)
                / config.robotRelativeSign;
    }

    public static double resolveAbsolute(double raw, double expected, double ratio, double sign,
                                         double offset, double min, double max) {
        return nearestEquivalent(sign * raw / ratio + offset, expected, min, max, 360 / ratio);
    }

    private static double nearestEquivalent(double base, double reference,
                                            double min, double max, double period) {
        if (!Double.isFinite(base) || !(period > 0)) return Double.NaN;
        double first = Math.ceil((min - base) / period);
        double last = Math.floor((max - base) / period);
        if (first > last) return Double.NaN;
        double sector = clip(Math.rint((reference - base) / period), first, last);
        return base + sector * period;
    }

    private static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    private static double wrap360(double value) { return (value % 360 + 360) % 360; }
    private static double wrap180(double value) { return wrap360(value + 180) - 180; }
}
