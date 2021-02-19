package org.firstinspires.ftc.teamcode.ultimategoal;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.rrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ultimategoal.rrunner.UltimateGoalDriveConstants;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

@Config
public abstract class UltimateGoalAutonomous extends UltimateGoalOpMode {

    {this.msStuckDetectStart = Integer.MAX_VALUE;} // prevent it from getting stuck in start()

    private SampleMecanumDrive drive;

    Pose2d startingPosition = new Pose2d(-58, 17 * yMult(), 0);

    ExecutorService pool = Executors.newSingleThreadExecutor();
    @SuppressWarnings({"rawtypes"}) Future opModeFuture;

    private Trajectory shootingLocation;

    private Trajectory squareA;
    private Trajectory squareB;
    private Trajectory squareC;


    private Trajectory parkB;
    private Trajectory parkC;


    public static int shootingEndLocY = 24;
    public static int shooterEndLocX = 8;
    public static int rpm1 = 2750;
    public static int shootWait = 500;

    /**
     * @return the integer to multiply all y position values by
     */
    protected abstract int yMult();

    @Override
    protected void initHardwareDevices() {
        super.initHardwareDevices();
        drive = new SampleMecanumDrive(hardwareMap, UltimateGoalDriveConstants.INSTANCE);
        shootingLocation = drive.trajectoryBuilder(new Pose2d(-58, 17 * yMult(), 0))
            .lineTo(new Vector2d(-22, 17 * yMult()))
            .splineToConstantHeading(new Vector2d(shooterEndLocX, shootingEndLocY * yMult()), 90 * yMult())
            .build();

        squareA = drive.trajectoryBuilder(shootingLocation.end())
            .lineTo(new Vector2d(4, 60 * yMult()))
            .build();
        squareB = drive.trajectoryBuilder(shootingLocation.end())
            .lineTo(new Vector2d(24, 35 * yMult()))
            .build();
        squareC = drive.trajectoryBuilder(shootingLocation.end())
            .lineTo(new Vector2d(50, 60 * yMult()))
            .build();
        // x value has to be between 15 and 6 to be parked properly

        parkB = drive.trajectoryBuilder(squareB.end())
            .lineTo(new Vector2d(15, 35 * yMult()))
            .build();
        parkC = drive.trajectoryBuilder(squareC.end())
            .lineTo(new Vector2d(15, 35 * yMult()))
            .build();

        // drive.setPoseEstimate(new Pose2d(0, 0));

        // drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(0, 0)).lineTo(new Vector2d(-58, 17 * y)).build());

        drive.setPoseEstimate(startingPosition);
        sleep(1000);
    }

    @Override
    public void start() {
        super.start();

        opModeFuture = pool.submit(() -> {
            try {
                runOpMode();
                requestOpModeStop();
            } catch (InterruptedException ignored) {
            }
        });
    }


    private void runOpMode() throws InterruptedException {

        shooterBack.setVelocity(rpm1 * 28.0 / 60); //convert to encoder ticks
        shooterFront.setVelocity(rpm1 * 28.0 / 60);

        boxServo.setPosition(boxLauncherPosition);

        drive.followTrajectory(shootingLocation);
        drive.turn(Math.toRadians(2));
        telemetry.addLine().addData("Heading:   ", drive.getRawExternalHeading());
        telemetry.update();
        sleep(700);// TODO: shoot rings

        for (int i = 0; i < 3 && !isStopRequested(); i++) {
            ringServo.setPosition(1);
            sleep(300);
            ringServo.setPosition(0);
            sleep(shootWait);
        }
        drive.turn(Math.toRadians(-2)); // undo turn

        shooterFront.setVelocity(0);
        shooterBack.setVelocity(0);

        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).lineTo(startingPosition.vec()).build());


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
    }

    @Override
    public void loop() {}

    @Override
    public void stop() {
        super.stop();
        if (opModeFuture != null) {
            opModeFuture.cancel(true);
            opModeFuture = null;
        }
//        Thread.currentThread().interrupt();
    }

}
