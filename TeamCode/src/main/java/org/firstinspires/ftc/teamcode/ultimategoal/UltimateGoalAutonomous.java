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

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import static org.firstinspires.ftc.teamcode.ultimategoal.UltimateGoalAutoConstants.*;

@Config
@Autonomous
public class UltimateGoalAutonomous extends UltimateGoalOpMode {

    {msStuckDetectInit = 10_000;}

    private static final String VUFORIA_KEY = "AY2fDeT/////AAABmSQo7FZp407MmUmu4W/djRxOWPSXLk/0nKrebNwBqKhKa7iBAhQth0OvoRSpG5xwPtWE90+JwjuAN4n63OCkZS4SrWnGZTtx6UR5g2fiJ/NEmte4NJjBNPA43DQiWvM9tT5960+i0Az6HO5jT/3Ert6ucsjj+eya6gdWaKzAlVknYJrgCeSpCWhpwvwiy1BVkuqsMRRC1MF77uaUtrt04HFzckOOJ4HMTUXLvCyN+EdmxuIqIzNEH3y4LSfoK+HDBaffqNYaRGlQHm3xDdimZ/Uw57NpO+96zwcPA8wvoiEjJ1x/oJ+8o7sO0EgULM8t73chtlzmM9YBkZUM1VMapzrUeunrULin9M8hj1r2SMPI";
    private static final ExecutorService pool = Executors.newSingleThreadExecutor();


    private TFObjectDetector tfod;

    @SuppressWarnings({"rawtypes"}) Future opModeFuture;

    private Trajectory powerShotLocation, squareA, squareB, squareC, secondWobbleA, secondWobbleB, secondWobbleC,
        squareA2, squareB2, squareC2, parkA, parkB, parkC;

    enum RingsDetected {ZERO, ONE, FOUR}

    private RingsDetected ringsDetected = RingsDetected.ZERO;

    @Override
    protected void initHardwareDevices() {
        super.initHardwareDevices();

        drive.setPoseEstimate(startingPosition());


        powerShotLocation = drive.trajectoryBuilder(startingPosition())
            .splineToConstantHeading(powerShotPosition().vec(), 0)
            .build();

        squareA = lineTo(powerShotLocation, squareAPosition().vec());
        squareB = lineTo(powerShotLocation, squareBPosition().vec());
        squareC = lineTo(powerShotLocation, squareCPosition().vec());

        secondWobbleA = lineToLinearHeading(squareA, -27, -45, 180);
//        secondWobbleA = drive.trajectoryBuilder(squareA.end())
//                .lineToLinearHeading(new Pose2d(-27,-45,180))
//                .forward(3)
//                .build();
        secondWobbleB = lineToLinearHeading(squareB, -37, -55, 180);
        secondWobbleC = lineToLinearHeading(squareC, -37, -55, 180);

        squareA2 = lineToLinearHeading(secondWobbleA, -2, -60, 0);
        squareB2 = lineToLinearHeading(secondWobbleB, 24, -35, 0);
        squareC2 = lineToLinearHeading(secondWobbleC, 54, -60, 0);

        // x value has to be between 15 and 6 to be parked properly
        parkA = drive.trajectoryBuilder(squareA.end())
            .back(10)
            .splineToConstantHeading(new Vector2d(18, -35), 0)
            .build();
//                drive.trajectoryBuilder(squareA.end())
//            .lineTo(new Vector2d(squareA.end().getX(), -shootingEndLocY))
        // .splineToConstantHeading(new Vector2d(15, -35), 0)
        parkB = lineTo(squareB, 15, -35);
        parkC = lineTo(squareC, 15, -35);
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

        setLauncherRPM(0);

        Trajectory square;
        Trajectory square2;
        Trajectory secondWobbleGoal;
        Trajectory park;
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
                break;
            case FOUR:
                square = squareC;
                secondWobbleGoal = secondWobbleC;
                square2 = squareC2;
                park = parkC;
                break;
            default:
                throw new IllegalStateException("Rings Detected");
        }

        drive.followTrajectoryAsync(square);
        dropWobbleGoal();
        waitForRoadRunnerIdle();
        drive.followTrajectory(secondWobbleGoal);
        drive.followTrajectory(drive.trajectoryBuilder(secondWobbleGoal.end())
            .forward(5)
            .build());
        resetWobbleGoal();
        drive.followTrajectoryAsync(square2);
        whileSleep(300);
        dropWobbleGoal();
        waitForRoadRunnerIdle();
        drive.followTrajectory(park);

//      drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).lineTo(startingPosition.vec()).build()); // return to starting position
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
            sleep(200);
            ringServo.setPosition(0);
            sleep(shootWait);
            telemetry.addData("Heading: ", drive.getRawExternalHeading());
        }
        drive.turn(-powerShotAngle3);
    }


    private Trajectory lineTo(Trajectory lastTrajectory, double x, double y) {
        return lineTo(lastTrajectory, new Vector2d(x, y));
    }

    private Trajectory lineTo(Trajectory lastTrajectory, Vector2d vector2d) {
        return drive.trajectoryBuilder(lastTrajectory.end())
            .lineTo(vector2d)
            .build();
//  Don't delete this, testing for temporal markers
//        return drive.trajectoryBuilder(lastTrajectory.end())
//                .lineTo(vector2d)
//                .addTemporalMarker(0,wobbleServo.setPosition(Servo.MIN_POSITION)
//                .build();
    }

    private Trajectory lineToLinearHeading(Trajectory lastTrajectory, double x, double y, double headingDegrees) {
        return lineToLinearHeading(lastTrajectory, new Pose2d(x, y, Math.toRadians(headingDegrees)));
    }

    private Trajectory lineToLinearHeading(Trajectory lastTrajectory, Pose2d pose2d) {
        return drive.trajectoryBuilder(lastTrajectory.end()).lineToLinearHeading(pose2d).build();
    }

}
