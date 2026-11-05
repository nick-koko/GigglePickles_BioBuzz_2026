package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.TurretController2026;

public class GlobalRobotData {
    public static boolean hasAutonRun = false;
    public static int endAutonBallCount = -1;
    public static Pose endAutonPose = null;
    public static double endAutonTurretAngleDegrees = Double.NaN;
    public static double endAutonTurretServoCommandAngleDegrees = Double.NaN;
    public static double endAutonTurretServoOffsetDegrees = Double.NaN;
    public static TurretController2026.Snapshot endAutonTurret2026 = null;

    public static COLOR allianceSide = COLOR.BLUE;
    public enum COLOR {
        RED,
        BLUE
    }
}


