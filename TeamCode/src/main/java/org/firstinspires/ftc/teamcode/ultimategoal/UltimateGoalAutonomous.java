package org.firstinspires.ftc.teamcode.ultimategoal;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.rrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ultimategoal.rrunner.UltimateGoalDriveConstants;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import static org.firstinspires.ftc.teamcode.ultimategoal.UltimateGoalAutoConstants.*;

@Config
@Autonomous
public class UltimateGoalAutonomous extends UltimateGoalOpMode {

    { msStuckDetectInit = 10_000; }

    private static final String VUFORIA_KEY = "AY2fDeT/////AAABmSQo7FZp407MmUmu4W/djRxOWPSXLk/0nKrebNwBqKhKa7iBAhQth0OvoRSpG5xwPtWE90+JwjuAN4n63OCkZS4SrWnGZTtx6UR5g2fiJ/NEmte4NJjBNPA43DQiWvM9tT5960+i0Az6HO5jT/3Ert6ucsjj+eya6gdWaKzAlVknYJrgCeSpCWhpwvwiy1BVkuqsMRRC1MF77uaUtrt04HFzckOOJ4HMTUXLvCyN+EdmxuIqIzNEH3y4LSfoK+HDBaffqNYaRGlQHm3xDdimZ/Uw57NpO+96zwcPA8wvoiEjJ1x/oJ+8o7sO0EgULM8t73chtlzmM9YBkZUM1VMapzrUeunrULin9M8hj1r2SMPI";
    private static final ExecutorService pool = Executors.newSingleThreadExecutor();


    private TFObjectDetector tfod;

    @SuppressWarnings({"rawtypes"})
    Future opModeFuture;

    private Trajectory powerShotLocation, squareA, squareB, squareC, secondWobbleA, secondWobbleB, secondWobbleC,
            squareA2, squareB2, squareC2, parkA, parkB, parkC, highGoalLocation;

    enum RingsDetected {ZERO, ONE, FOUR}

    private RingsDetected ringsDetected = RingsDetected.ZERO;

    @Override
    protected void initHardwareDevices() {
        super.initHardwareDevices();

        drive = new SampleMecanumDrive(hardwareMap, UltimateGoalDriveConstants.INSTANCE);
        drive.setPoseEstimate(UltimateGoalAutoConstants.startingPosition());

        wobbleServo.setPosition(wobbleServo.MIN_POSITION);

        powerShotLocation = drive.trajectoryBuilder(startingPosition())
                .splineToConstantHeading(powerShotPosition().vec(), 0)
                .build();

        squareA = lineTo(powerShotLocation, squareAPosition().vec());
        squareB = lineTo(powerShotLocation, squareBPosition().vec());
        squareC = lineTo(powerShotLocation, squareCPosition().vec());

        secondWobbleA = lineToLinearHeading(squareA, wobble2Position());
        secondWobbleB = splineToLinearHeading(squareB, wobble2Position(), 90);
        secondWobbleC = lineToLinearHeading(squareC, wobble2Position());

        highGoalLocation = drive.trajectoryBuilder(secondWobbleB.end().plus(new Pose2d(0, 0, turnOneRing)))
                .splineToLinearHeading(towerGoalPosition(), 0)
                .build();

        squareA2 = lineToLinearHeading(secondWobbleA, squareA2Position());
        squareB2 = splineToLinearHeading(highGoalLocation, squareB2Position(), 0);
        squareC2 = drive.trajectoryBuilder(secondWobbleC.end().plus(new Pose2d(0, 7, 0)))
                .splineToLinearHeading(squareC2Position(), 25)
                .build();

        // x value has to be between 15 and 6 to be parked properly
        parkA = drive.trajectoryBuilder(squareA2.end())
                .back(7)
                .splineToConstantHeading(parkPosition().vec(), 0)
                .build();
        parkB = lineTo(squareB, parkPosition().vec());
        parkC = lineTo(squareC, parkPosition().vec());
        initializeTfod();
    }

    @Override
    public void init_loop() {
        super.init_loop();
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            if (updatedRecognitions.size() == 0) {
                ringsDetected = RingsDetected.ZERO;
            } else {
                telemetry.addData("Label", updatedRecognitions.get(0).getLabel());
                ringsDetected = RingsDetected.valueOf(updatedRecognitions.get(0).getLabel());
                if (updatedRecognitions.size() > 1) {
                    telemetry.addData("Warning", "Multiple recognitions, choosing the first one");
                }
            }
        }
    }

    @Override
    public void start() {
        super.start();
        tfod.deactivate();

        opModeFuture = pool.submit(() -> {
            try {
                runOpMode();
                requestOpModeStop();
            } catch (InterruptedException ignored) {
            }
        });
    }

    private void runOpMode() throws InterruptedException {
        setLauncherRPM(powerShotRPM);
        boxServo.setPosition(boxLauncherPosition);

        powerShotExecution();

        boxServo.setPosition(boxInitialPosition);

        setLauncherRPM(0);

        Trajectory square, square2, secondWobbleGoal, park;


        switch (ringsDetected) {
            case ZERO:
                square = squareA;
                secondWobbleGoal = secondWobbleA;
                square2 = squareA2;
                park = parkA;
                break;
            case ONE:
                square = squareB;
                secondWobbleGoal = secondWobbleB;
                square2 = squareB2;
                park = parkB;
                firstWobbleWaitTime = 200;
                secondWobbleWaitTime = 900;
                break;
            case FOUR:
                square = squareC;
                secondWobbleGoal = secondWobbleC;
                square2 = drive.trajectoryBuilder(highGoalLocation.end()).lineTo(squareCPosition().vec()).build();
                park = parkC;
                firstWobbleWaitTime = 700;
                secondWobbleWaitTime = 1500;
                break;
            default:
                throw new IllegalStateException("Rings Detected");
        }

        // Deposits first wobble goal
        drive.followTrajectoryAsync(square);
        whileSleep(firstWobbleWaitTime);
        dropWobbleGoal();
//        waitForRoadRunnerIdle();

        // Grabs second wobble goal
        drive.followTrajectory(secondWobbleGoal);
        drive.followTrajectory(drive.trajectoryBuilder(secondWobbleGoal.end())
                .forward(5)
                .build());
        resetWobbleGoal(-140);

        // Grabs the ring
        if (ringsDetected == RingsDetected.ONE || ringsDetected == RingsDetected.FOUR) {
            intake.setPower(0.5);
            drive.turn(turnOneRing);
            setLauncherRPM(highGoalRPM);
            drive.followTrajectory(highGoalLocation);
            boxServo.setPosition(boxLauncherPosition);
            sleep(500);
            ringServo.setPosition(1);
            sleep(200);
            setLauncherRPM(0);
            intake.setPower(0);
        }

        // Drops Second wobble Goal
        drive.followTrajectoryAsync(square2);
        whileSleep(secondWobbleWaitTime);
        dropWobbleGoal();
//        waitForRoadRunnerIdle();
        drive.followTrajectory(park);

//      drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).lineTo(startingPosition.vec()).build()); // return to starting position
    }

    @Override
    public void loop() {
    }

    @Override
    public void stop() {
        super.stop();
        if (opModeFuture != null) {
            opModeFuture.cancel(true);
            opModeFuture = null;
        }
        if (tfod != null) {
            tfod.shutdown();
            tfod = null;
        }
        drive = null;
    }


    private void initializeTfod() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile("/sdcard/FIRST/vision/UltimateGoal.tflite", RingsDetected.FOUR.toString(), RingsDetected.ONE.toString());
        tfod.setZoom(1.2, 1.78);
        tfod.activate();
    }

    private void powerShotExecution() {
        drive.followTrajectory(powerShotLocation);
        sleep(waitTimePowerShots);
        for (int i = 0; i <= 2; i++) {
            if (i == 1) {
                setLauncherRPM(powerShotRPM2);
            } else if (i == 2) {
                setLauncherRPM(powerShotRPM3);
            }
            drive.turn(powerShotAngles()[i]); // angle will be tuned later on
            sleep(timeBetweenPowerShots); // time between each powershot
            ringServo.setPosition(1);
            sleep(300); // was 200
            ringServo.setPosition(0);
//            sleep(shootWait);
            telemetry.addData("Heading: ", drive.getRawExternalHeading());
        }
        drive.turn(-powerShotAngle3);
    }

    @SuppressWarnings("SameParameterValue")
    private Trajectory lineTo(Trajectory lastTrajectory, double x, double y) {
        return lineTo(lastTrajectory, new Vector2d(x, y));
    }

    private Trajectory lineTo(Trajectory lastTrajectory, Vector2d vector2d) {
        return drive.trajectoryBuilder(lastTrajectory.end())
                .lineTo(vector2d)
                .build();
    }

    @SuppressWarnings("SameParameterValue")
    private Trajectory lineToLinearHeading(Trajectory lastTrajectory, double x, double y, double headingDegrees) {
        return lineToLinearHeading(lastTrajectory, new Pose2d(x, y, Math.toRadians(headingDegrees)));
    }

    private Trajectory lineToLinearHeading(Trajectory lastTrajectory, Pose2d pose2d) {
        return drive.trajectoryBuilder(lastTrajectory.end()).lineToLinearHeading(pose2d).build();
    }

    @SuppressWarnings("SameParameterValue")
    private Trajectory splineToLinearHeading(Trajectory lastTrajectory, double x, double y, double heading, double endTangent) {
        return splineToLinearHeading(lastTrajectory, new Pose2d(x, y, Math.toRadians(heading)), Math.toRadians(endTangent));
    }

    private Trajectory splineToLinearHeading(Trajectory lastTrajectory, Pose2d pose2d, double endTangent) {
        return drive.trajectoryBuilder(lastTrajectory.end()).splineToLinearHeading(pose2d, Math.toRadians(endTangent)).build();
    }

}
