package org.firstinspires.ftc.teamcode.ultimategoal;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Config
public abstract class UltimateGoalAutonomous extends UltimateGoalOpMode {

    {msStuckDetectInit = 10_000;}

    private static final String VUFORIA_KEY = "AY2fDeT/////AAABmSQo7FZp407MmUmu4W/djRxOWPSXLk/0nKrebNwBqKhKa7iBAhQth0OvoRSpG5xwPtWE90+JwjuAN4n63OCkZS4SrWnGZTtx6UR5g2fiJ/NEmte4NJjBNPA43DQiWvM9tT5960+i0Az6HO5jT/3Ert6ucsjj+eya6gdWaKzAlVknYJrgCeSpCWhpwvwiy1BVkuqsMRRC1MF77uaUtrt04HFzckOOJ4HMTUXLvCyN+EdmxuIqIzNEH3y4LSfoK+HDBaffqNYaRGlQHm3xDdimZ/Uw57NpO+96zwcPA8wvoiEjJ1x/oJ+8o7sO0EgULM8t73chtlzmM9YBkZUM1VMapzrUeunrULin9M8hj1r2SMPI";
    private static final ExecutorService pool = Executors.newSingleThreadExecutor();

    public static int shootingEndLocY = 24;
    public static int shooterEndLocX = 5;
    public static int rpm1 = 2750;
    public static int shootWait = 500;

    private final Pose2d startingPosition = new Pose2d(-58, 32 * yMult(), 0);


    private TFObjectDetector tfod;

    @SuppressWarnings({"rawtypes"}) Future opModeFuture;

    private Trajectory shootingLocation;
    private Trajectory squareA;
    private Trajectory squareB;
    private Trajectory squareC;
    private Trajectory parkA;
    private Trajectory parkB;
    private Trajectory parkC;

    enum RingsDetected {ZERO, ONE, FOUR}

    private RingsDetected ringsDetected;

    /**
     * @return the integer to multiply all y position values by
     */
    protected abstract int yMult();

    @Override
    protected void initHardwareDevices() {
        super.initHardwareDevices();

        drive.setPoseEstimate(startingPosition);

        shootingLocation = drive.trajectoryBuilder(startingPosition)
            .splineToConstantHeading(new Vector2d(-22, 17 * yMult()), 0)
            .splineToConstantHeading(new Vector2d(shooterEndLocX, shootingEndLocY * yMult()), 90 * yMult())
            .build();

        squareA = drive.trajectoryBuilder(shootingLocation.end())
                .lineTo(new Vector2d(4, 60 * yMult()))
            .build();
        squareB = drive.trajectoryBuilder(shootingLocation.end())
            .lineTo(new Vector2d(24, 35 * yMult()))
            .build();
        squareC = drive.trajectoryBuilder(shootingLocation.end())
                .lineTo(new Vector2d(54, 60 * yMult()))
            .build();
        // x value has to be between 15 and 6 to be parked properly

        parkA = drive.trajectoryBuilder(squareA.end())
            .lineTo(new Vector2d(squareA.end().getX(), shootingEndLocY * yMult()))
            .splineToConstantHeading(new Vector2d(15, 35 * yMult()), 0)
            .build();
        parkB = drive.trajectoryBuilder(squareB.end())
            .lineTo(new Vector2d(15, 35 * yMult()))
            .build();
        parkC = drive.trajectoryBuilder(squareC.end())
            .lineTo(new Vector2d(15, 35 * yMult()))
            .build();
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
        shooterBack.setVelocity(rpm1 * 28.0 / 60); //convert to encoder ticks
        shooterFront.setVelocity(rpm1 * 28.0 / 60);

        boxServo.setPosition(boxLauncherPosition);

        drive.followTrajectory(shootingLocation);
        drive.turn(Math.toRadians(5)); //shoot heading was 10
        telemetry.addLine().addData("Heading:   ", drive.getRawExternalHeading());
        telemetry.update();
        sleep(700);

        for (int i = 0; i < 3 && !isStopRequested(); i++) {
            ringServo.setPosition(1);
            sleep(200);
            ringServo.setPosition(0);
            sleep(shootWait);
        }
        drive.turn(Math.toRadians(-5)); // undo turn was -10

        shooterFront.setVelocity(0);
        shooterBack.setVelocity(0);

        Trajectory square;
        Trajectory park;
        switch (ringsDetected) {
            case ZERO:
                square = squareA;
                park = parkA;
                break;
            case ONE:
                square = squareB;
                park = parkB;
                break;
            case FOUR:
                square = squareC;
                park = parkC;
                break;
            default:
                throw new IllegalStateException("Rings Detected");
        }

        drive.followTrajectory(square);
        dropWobbleGoal();
        sleep(1000); // drop wobble goal
        resetWobbleGoal();
        sleep(1000);
        drive.followTrajectory(park);

        sleep(2000);
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

    private void dropWobbleGoal() {
        wobbleGoalMotor.setTargetPosition(wobbleGoalDrop * wobbleGoalMotorTicksPerDegree);
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleGoalMotor.setVelocity(20 * wobbleGoalMotorTicksPerDegree);
        while (wobbleGoalMotor.isBusy() && !isStopRequested()) {
            System.out.println("drop wobble goal loop");
            Thread.yield();
            sleep(100);
        }
        sleep(2000);
        wobbleGoalMotor.setVelocity(0);

        sleep(1000);
        wobbleServo.setPosition(Servo.MAX_POSITION);
        sleep(500);
    }

    private void resetWobbleGoal() {
        wobbleGoalMotor.setTargetPosition(wobbleGoalPick * wobbleGoalMotorTicksPerDegree);
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleGoalMotor.setVelocity(20 * wobbleGoalMotorTicksPerDegree);
        while (wobbleGoalMotor.isBusy() && !isStopRequested()) {
            Thread.yield();
            sleep(100);
        }
        sleep(2000);

        wobbleGoalMotor.setVelocity(0);
        wobbleServo.setPosition(Servo.MIN_POSITION);
        sleep(1000);
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

}
