package org.firstinspires.ftc.teamcode.ultimategoal;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class UltimateGoalAutoConstants {
    public static int shootingEndLocY = -24;
    public static int shootWait = 500;
    public static int timeBetweenPowerShots = 1500;

    public static String[] poses = {};
    public static String startingPose = "-58,-32,0";
    public static String powerShotPose = "0,-15,0";
    public static String towerGoal = "0,0,0";
    public static String squareAPose = "4,-60,0";
    public static String squareBPose = "24,-35,0";
    public static String squareCPose = "54,-60,0";

    public static int powerShotRPM = 2590;
    public static int powerShotRPM2 = 2630;
    public static int powerShotRPM3 = 2605;
    public static double powerShotAngle1 = 5;
    public static double powerShotAngle2 = 9.7;
    public static double powerShotAngle3 = 15.41;
    public static int waitTimePowerShots = 0;
    public static int highGoalRPM = 2700;

    static Pose2d startingPosition() {
        return pose2d(startingPose);
    }
    static Pose2d powerShotPosition() {
        return pose2d(powerShotPose);
    }
    static Pose2d towerGoalPosition() { return pose2d(towerGoal); }
    static Pose2d squareAPosition() {return pose2d(squareAPose);}
    static Pose2d squareBPosition() {return pose2d(squareBPose);}
    static Pose2d squareCPosition() {return pose2d(squareCPose);}

    static double[] powerShotAngles(){
        return new double[]{powerShotAngle1, powerShotAngle2 - powerShotAngle1, powerShotAngle3 - powerShotAngle2};
    }
    private static Pose2d pose2d(String s) {
        String[] split = s.split(",");
        return new Pose2d(Double.parseDouble(split[0].trim()), Double.parseDouble(split[1].trim()), Double.parseDouble(split[2].trim()));
    }

}
