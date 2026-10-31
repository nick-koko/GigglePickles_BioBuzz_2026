package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.VectorCalculator;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

/** Heading correction only; the caller retains control of translation. */
public final class HeadingTurnController2026 {
    // Separate state prevents teleop aiming from resetting the drivetrain or follower.
    // The coefficient references come from the follower's autonomous configuration.
    private final FollowerConstants pedroConstants;
    private VectorCalculator pedroCalculator;
    private boolean pedroPrimed;
    private double previousErrorRadians;

    public HeadingTurnController2026(FollowerConstants constants) {
        pedroConstants = constants;
        reset();
    }

    public void reset() {
        // Pedro 2's breakFollowing appends velocity samples, so use fresh state.
        pedroCalculator = new VectorCalculator(pedroConstants);
        pedroCalculator.breakFollowing();
        pedroPrimed = false;
        previousErrorRadians = 0.0;
    }

    public double calculatePedro(Pose pose, double targetHeadingRadians) {
        double errorRadians = MathFunctions.normalizeAngleSigned(
                targetHeadingRadians - pose.getHeading()
        );
        // Crossing the +/-pi error branch must not appear as a 2*pi derivative.
        if (pedroPrimed && Math.abs(errorRadians - previousErrorRadians) > Math.PI) {
            reset();
        }
        pedroCalculator.updateConstants();
        Vector correction = pedroCalculator.getHeadingVector(
                errorRadians, pose, targetHeadingRadians
        );
        previousErrorRadians = errorRadians;
        if (!pedroPrimed) {
            // Seed Pedro's error history without applying its initial derivative kick.
            // Normal output begins on the next control loop.
            pedroPrimed = true;
            return 0.0;
        }
        // Pedro stores negative vector magnitudes as a positive magnitude plus pi.
        return correction.dot(new Vector(1.0, pose.getHeading()));
    }

    public static double calculateCustom(
            double errorRadians,
            double angularVelocityRadiansPerSecond,
            double kPPerDegree,
            double kDPerDegreePerSecond,
            double toleranceDegrees
    ) {
        double errorDegrees = Math.toDegrees(MathFunctions.normalizeAngleSigned(errorRadians));
        double p = Math.abs(errorDegrees) <= Math.max(0.0, toleranceDegrees)
                ? 0.0 : errorDegrees * kPPerDegree;
        // Measured velocity gives damping without derivative kick from a target change.
        double d = -Math.toDegrees(angularVelocityRadiansPerSecond) * kDPerDegreePerSecond;
        return p + d;
    }

    public static double applyCompensationAndLimits(
            double rawTurn,
            double errorRadians,
            double angularVelocityRadiansPerSecond,
            double minimumPower,
            double kS,
            double maximumPower,
            double toleranceDegrees,
            double settleSpeedDegreesPerSecond
    ) {
        double errorDegrees = Math.toDegrees(MathFunctions.normalizeAngleSigned(errorRadians));
        boolean withinTolerance = Math.abs(errorDegrees) <= Math.max(0.0, toleranceDegrees);
        if (withinTolerance && Math.abs(Math.toDegrees(angularVelocityRadiansPerSecond))
                <= Math.max(0.0, settleSpeedDegreesPerSecond)) {
            return 0.0;
        }

        double turn = rawTurn;
        // Do not force motion from a zero controller output, or amplify D's braking.
        // Inside tolerance, allow damping but omit the friction compensation.
        if (!withinTolerance && rawTurn * errorDegrees > 0.0) {
            turn += Math.copySign(Math.max(0.0, kS), rawTurn);
            turn = Math.copySign(Math.max(Math.abs(turn), Math.max(0.0, minimumPower)), turn);
        }
        double limit = Math.abs(maximumPower);
        return Math.max(-limit, Math.min(limit, turn));
    }
}
