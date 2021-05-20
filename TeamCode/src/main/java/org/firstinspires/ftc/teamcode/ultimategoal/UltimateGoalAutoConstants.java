package org.firstinspires.ftc.teamcode.ultimategoal;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class UltimateGoalAutoConstants {
    public static int shootingEndLocY = -24;
    public static int shootWait = 500;
    public static int timeBetweenPowerShots = 1000; // was 1500

    public static String startingPose = "-58,-32,0";
    public static String powerShotPose = "0,-15,0";
    public static String towerGoal = "0,-28,0"; // was 0,-28,0
    public static String parkPose = "15,-35,0";
    public static String resetPose = "-63.75, -63.6875, 0";

    public static String squareAPose = "8,-60,0";
    public static String squareBPose = "29,-35,0";
    public static String squareCPose = "56,-60,0";
    public static String squareA2Pose = "0,-60,0.01";
    public static String squareB2Pose = "21,-39,0.01";
    public static String squareC2Pose = "49,-60,25";
    public static String wobble2Pose = "-22,-46,180"; // was -22, -46 180

    public static final int powerShotRPM = 2590;
    public static final int powerShotRPM2 = 2630;
    public static final int powerShotRPM3 = 2605;
    public static final double powerShotAngle1 = 5;
    public static final double powerShotAngle2 = 9.7;
    public static double powerShotAngle3 = 16; // used to be 15.41
    public static final int waitTimePowerShots = 0;
    public static int highGoalRPM = 2700;
    public static int turnOneRing = -140;
    public static int firstWobbleWaitTime = 0, secondWobbleWaitTime = 1700;

    static Pose2d startingPosition() {
        return pose2d(startingPose);
    }

    static Pose2d powerShotPosition() {
        return pose2d(powerShotPose);
    }

    static Pose2d towerGoalPosition() {
        return pose2d(towerGoal);
    }

    static Pose2d parkPosition() {
        return pose2d(parkPose);
    }

    static Pose2d squareAPosition() {
        return pose2d(squareAPose);
    }

    static Pose2d squareBPosition() {
        return pose2d(squareBPose);
    }

    static Pose2d squareCPosition() {
        return pose2d(squareCPose);
    }

    static Pose2d squareA2Position() {
        return pose2d(squareA2Pose);
    }

    static Pose2d squareB2Position() {
        return pose2d(squareB2Pose);
    }

    static Pose2d squareC2Position() {
        return pose2d(squareC2Pose);
    }

    static Pose2d wobble2Position() {
        return pose2d(wobble2Pose);
    }

    static Pose2d resetPosition() {
        return pose2d(resetPose);
    }

    static double[] powerShotAngles() {
        return new double[]{powerShotAngle1, powerShotAngle2 - powerShotAngle1, powerShotAngle3 - powerShotAngle2};
    }

    private static Pose2d pose2d(String s) {
        String[] split = s.split(",");
        return new Pose2d(Double.parseDouble(split[0].trim()), Double.parseDouble(split[1].trim()), Math.toRadians(Double.parseDouble(split[2].trim())));
    }

}
