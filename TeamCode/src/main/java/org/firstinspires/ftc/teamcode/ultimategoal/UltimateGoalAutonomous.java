package org.firstinspires.ftc.teamcode.ultimategoal;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.testchassis.rrunner.SampleMecanumDrive;

public abstract class UltimateGoalAutonomous extends LinearOpMode {

    public void runOpMode(boolean blueSide) throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        final int y = blueSide ? 1 : -1;

        drive.setPoseEstimate(new Pose2d(0, 0));

        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(0, 0)).lineTo(new Vector2d(-58, 17 * y)).build());

        Thread.sleep(1000);

        Trajectory shootingLocation = drive.trajectoryBuilder(new Pose2d(-58, 17 * y))
            .lineTo(new Vector2d(-22, 17 * y))
            .splineToConstantHeading(new Vector2d(-5, 35 * y), 90 * y).build();

        Trajectory squareA = drive.trajectoryBuilder(shootingLocation.end())
            .lineTo(new Vector2d(4, 60 * y))
            .build();
        Trajectory squareB = drive.trajectoryBuilder(shootingLocation.end())
            .lineTo(new Vector2d(24, 35 * y))
            .build();
        Trajectory squareC = drive.trajectoryBuilder(shootingLocation.end())
            .lineTo(new Vector2d(50, 60 * y))
            .build();
// x value has to be between 15 and 6 to be parked properly

        Trajectory parkB = drive.trajectoryBuilder(squareB.end())
            .lineTo(new Vector2d(15, 35 * y))
            .build();
        Trajectory parkC = drive.trajectoryBuilder(squareC.end())
            .lineTo(new Vector2d(15, 35 * y))
            .build();

        waitForStart();

        telemetry.update();

        System.out.println();


        if (isStopRequested()) return;

        System.out.println();

        // TODO: speed up shooting wheels

        drive.followTrajectory(shootingLocation);

        Thread.sleep(2000);// TODO: shoot rings

        int ringsDetected = 4;

        if (ringsDetected == 0) {
            drive.followTrajectory(squareA);
            Thread.sleep(1000); // drop wobble goal
            // already parked
        } else if (ringsDetected == 1) {
            drive.followTrajectory(squareB);
            Thread.sleep(1000); // drop wobble goal
            drive.followTrajectory(parkB);
        } else if (ringsDetected == 4) {
            drive.followTrajectory(squareC);
            Thread.sleep(1000); // drop wobble goal
            drive.followTrajectory(parkC);
        }
    }

}
