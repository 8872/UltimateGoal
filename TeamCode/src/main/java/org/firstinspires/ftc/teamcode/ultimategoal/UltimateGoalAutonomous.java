package org.firstinspires.ftc.teamcode.ultimategoal;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.rrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ultimategoal.rrunner.UltimateGoalDriveConstants;

@Config
public abstract class UltimateGoalAutonomous extends UltimateGoalOpMode {

    private final SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, UltimateGoalDriveConstants.INSTANCE);

    private final Trajectory shootingLocation = drive.trajectoryBuilder(new Pose2d(-58, 17 * yMult(), 0))
        .lineTo(new Vector2d(-22, 17 * yMult()))
        .splineToConstantHeading(new Vector2d(shooterEndLocX, shootingEndLocY * yMult()), 90 * yMult())
        .build();

    private final Trajectory squareA = drive.trajectoryBuilder(shootingLocation.end())
        .lineTo(new Vector2d(4, 60 * yMult()))
        .build();
    private final Trajectory squareB = drive.trajectoryBuilder(shootingLocation.end())
        .lineTo(new Vector2d(24, 35 * yMult()))
        .build();
    private final Trajectory squareC = drive.trajectoryBuilder(shootingLocation.end())
        .lineTo(new Vector2d(50, 60 * yMult()))
        .build();
// x value has to be between 15 and 6 to be parked properly

    private final Trajectory parkB = drive.trajectoryBuilder(squareB.end())
        .lineTo(new Vector2d(15, 35 * yMult()))
        .build();
    private final Trajectory parkC = drive.trajectoryBuilder(squareC.end())
        .lineTo(new Vector2d(15, 35 * yMult()))
        .build();


    public static int turnAfterShoot = -2;
    public static int shootingEndLocY = 24;
    public static int shooterEndLocX = 8;
    public static int rpm1 = 3500;
    public static int shootWait = 6000;

    /**
     * @return the integer to multiply all y position values by
     */
    protected abstract int yMult();

    @Override
    protected void initHardwareDevices() {
        super.initHardwareDevices();


        // drive.setPoseEstimate(new Pose2d(0, 0));

        // drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(0, 0)).lineTo(new Vector2d(-58, 17 * y)).build());

        drive.setPoseEstimate(new Pose2d(-58, 17 * yMult(), 0));
        sleep(1000);
    }

    @Override
    public void start() {
        super.start();
        // TODO: speed up shooting wheels
        shooterBack.setVelocity(rpm1 * 28.0 / 60); //convert to encoder ticks
        shooterFront.setVelocity(rpm1 * 28.0 / 60);

        boxServo.setPosition(0.75);
        drive.followTrajectory(shootingLocation);
        telemetry.addLine().addData("Heading: ", drive.getRawExternalHeading());
        telemetry.update();
        drive.turn(Math.toRadians(turnAfterShoot));
        sleep(100);// TODO: shoot rings

        for (int i = 0; i < 3; i++) {
            ringServo.setPosition(1);
            sleep(300);
            ringServo.setPosition(0);
            sleep(shootWait);
        }

        shooterFront.setVelocity(0);
        shooterBack.setVelocity(0);

        int ringsDetected = 2;

        if (ringsDetected == 0) {
            drive.followTrajectory(squareA);
            sleep(1000); // drop wobble goal
            // already parked
        } else if (ringsDetected == 1) {
            drive.followTrajectory(squareB);
            sleep(1000); // drop wobble goal
            drive.followTrajectory(parkB);
        } else if (ringsDetected == 2) {
            drive.followTrajectory(squareC);
            sleep(1000); // drop wobble goal
            drive.followTrajectory(parkC);
        }
        telemetry.addLine().addData("Heading ", drive.getRawExternalHeading());
        telemetry.update();
        sleep(3000);
        requestOpModeStop();
    }

    @Override
    public void loop() { }
}
