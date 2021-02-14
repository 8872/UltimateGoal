package org.firstinspires.ftc.teamcode.rrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.DriveConstants;

/*
wee wee
*/
@Autonomous(group = "drive")
@Config
public class TrajectoryTest extends LinearOpMode {
    public static double heading1 = 0;
    public static double heading2 = 0;
    public static double heading3 = 0;
    public static double initialX = -60;
    public static double initialY = -24;
    public static double x1 = -40;
    public static double y1 = 0;
    public static double x2 = -20;
    public static double y2 = -24;
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, DriveConstants.DEFAULT);

        waitForStart();
        Pose2d startPose = new Pose2d(initialX, initialY, Math.toRadians(heading1));
        Pose2d endPose = new Pose2d(x2,y2,Math.toRadians(heading3));
        drive.setPoseEstimate(startPose);

        if (isStopRequested()) return;

        System.out.println();
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
            .splineTo(new Vector2d(x1, y1), heading2)
            .splineTo(new Vector2d(x2, y2), heading3)
            .build();

        Trajectory traj2 = drive.trajectoryBuilder(endPose).
                back(Math.abs(x2-initialX)).
                build();

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);



    }
}
