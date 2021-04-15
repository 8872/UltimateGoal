  package org.firstinspires.ftc.teamcode.ultimategoal;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.rrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ultimategoal.rrunner.UltimateGoalDriveConstants;
import org.firstinspires.ftc.teamcode.util.Timer;

import static org.firstinspires.ftc.teamcode.ultimategoal.UltimateGoalAutoConstants.*;
import static org.firstinspires.ftc.teamcode.ultimategoal.UltimateGoalAutoConstants.powerShotAngle3;

  @TeleOp
@Config
public class UltimateGoalDrive extends UltimateGoalOpMode {

    // High Goal RPM: 2750
    // Mid Goal RPM: 1800
    public static int launcherRPMSpeed = 1900;
    public static int launcherRPMSpeed2 = 2200; // used to be 2200
    private static double intakePower = 0.5;
    static double[] powerShotAngles2(){
        return new double[]{powerShotAngle1, powerShotAngle2 - powerShotAngle1, powerShotAngle3 - powerShotAngle2, -powerShotAngle3 + powerShotAngle1};
    }
    public static int boxServoTime = 1000;
    public static int kickerServoTime = 750;
    public static int kickerServoPowerShotTime = 1500;
    public static int kickerServoTime2 = 250;

    // These values might not update, maybe have an update function to reset the values of the class
    private final Timer boxServoTimer = new Timer(boxServoTime); // try 300
    private final Timer kickerServoTimer = new Timer(kickerServoTime); // try 200
      private final Timer kickerServoPowerShotTimer = new Timer(kickerServoPowerShotTime);
    private final Timer kickerServoTimer2 = new Timer(kickerServoTime2); // try 400
    private final Timer wobbleGoalPickUpTimer = new Timer(700);
    // previous 1500
    private final Timer wobbleGoalDropTimer = new Timer(1500);

    private enum ShooterState {IDLE, BOX, KICKER1, KICKER2}

    private ShooterState shooterState = ShooterState.IDLE;

    private boolean slowMode, controlMode = false;
    private boolean lastAState, lastBumperState, lastDPadDown, lastDPadUp, lastRightStickButton, lastYState, lastDpadLeft,lastDpadRight, lastStickButton;
    private boolean shooterEnabled = false;
    private double acceleratePower = 0;
    private int j = 0;

    public void initHardwareDevices(){
        super.initHardwareDevices();
        if (drive == null) {
            drive = new SampleMecanumDrive(hardwareMap, UltimateGoalDriveConstants.INSTANCE);
            drive.setPoseEstimate(UltimateGoalAutoConstants.parkPosition());
        }
        wobbleServo.setPosition(wobbleServoMax);
    }

    @Override
    public void loop() {
          if (gamepad1.y) {
            acceleratePower = 0;
        }
        if (!drive.isBusy() && (gamepad1.x || acceleratePower != 0)) {
            acceleratePower = accelerate(acceleratePower);
        } else if (!drive.isBusy()) {
            mechanumDrive(slowMode, false, false);
        }

        if(gamepad1.right_stick_button && !lastRightStickButton){
            drive.setPoseEstimate(UltimateGoalAutoConstants.resetPosition());
        }
        lastRightStickButton = !lastRightStickButton;


        if (gamepad1.a && !lastAState) {
            slowMode = !slowMode;
        }
        lastAState = gamepad1.a;

        //// drive to powershot
        if (gamepad1.y && !lastYState) {
            System.out.println("Position: " + drive.getPoseEstimate());
            System.out.println("Traveling to " + UltimateGoalAutoConstants.towerGoalPosition());
            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(UltimateGoalAutoConstants.powerShotPosition()).build());
            j = 0;
        }
        lastYState = gamepad1.y;

        if (gamepad1.left_stick_button && !lastStickButton && katanaServo.getPosition() == 1) {
            katanaServo.setPosition(0);
        } else if (gamepad1.left_stick_button && !lastStickButton && katanaServo.getPosition() == 0) {
            katanaServo.setPosition(1);
        }
        lastStickButton = gamepad1.left_stick_button;

        if ((gamepad1.left_bumper && gamepad1.right_bumper) && !lastBumperState) {
            intake.setPower(0);
        } else if (gamepad1.left_bumper && !lastBumperState) {
            intake.setPower(intakePower);
        } else if (gamepad1.right_bumper && !lastBumperState) {
            intake.setPower(-intakePower);
        }
        lastBumperState = gamepad1.left_bumper || gamepad1.right_bumper;

        shooterEnabled = (gamepad1.left_trigger != 0 && gamepad1.right_trigger == 0)
            || (gamepad1.left_trigger == 0 && gamepad1.right_trigger != 0);

        wobbleGoalExecution();

        if (gamepad1.dpad_left && !lastDpadLeft) {
            System.out.println("Position: " + drive.getPoseEstimate());
            System.out.println("Traveling to " + UltimateGoalAutoConstants.towerGoalPosition());
            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(UltimateGoalAutoConstants.towerGoalPosition()).build());
            j = 0;
//            drive.turnAsync(-Math.toDegrees(drive.getExternalHeading())); // turn to 0
        }
        lastDpadLeft = gamepad1.dpad_left;
/// new
        if (gamepad1.dpad_right && !lastDpadRight) {
            drive.turn(powerShotAngles2()[j]); // angle will be tuned later on
            telemetry.addData("Heading: ", drive.getRawExternalHeading());
            j++;
            if(j >= 4){
                j=0;
            }
        }
        lastDpadRight = gamepad1.dpad_right;

        if (!shooterEnabled && shooterState != ShooterState.IDLE) {
            shooterState = ShooterState.IDLE;
            resetLauncher();
        }
        switch (shooterState) {
            case IDLE:
                if (shooterEnabled) {
                    telemetry.addData("reset", false);
                    if (gamepad1.right_trigger != 0) {
                        setLauncherRPM(launcherRPMSpeed);
                    } else if (gamepad1.left_trigger != 0) {
                        setLauncherRPM(launcherRPMSpeed2);
                    }
                    shooterState = ShooterState.BOX;
                    boxServo.setPosition(boxLauncherPosition);
                    boxServoTimer.start();
                }
                break;
            case BOX:
                if (boxServoTimer.isFinished()) {
                    shooterState = ShooterState.KICKER1;
                    ringServo.setPosition(Servo.MAX_POSITION);
                    if (gamepad1.right_trigger != 0) {
                        kickerServoTimer.start();
                    } else if (gamepad1.left_trigger != 0) {
                        kickerServoPowerShotTimer.start();
                    }
                }
                break;
            case KICKER1:
                if ((gamepad1.right_trigger != 0 && kickerServoTimer.isFinished()) || (gamepad1.left_trigger != 0 && kickerServoPowerShotTimer.isFinished())) {
                    shooterState = ShooterState.KICKER2;
                    ringServo.setPosition(Servo.MIN_POSITION);
                    kickerServoTimer2.start();
                }
                break;
            case KICKER2: // move kicker out again
                if (kickerServoTimer2.isFinished()) {
                    shooterState = ShooterState.KICKER1;
                    ringServo.setPosition(Servo.MAX_POSITION);
                    if (gamepad1.right_trigger != 0) {
                        kickerServoTimer.start();
                    } else if (gamepad1.left_trigger != 0) {
                        kickerServoPowerShotTimer.start();
                    }
                }
                break;
        }
        drive.telemetry.put("Top Limit", wobbleGoalTopLimit.isPressed());
        drive.telemetry.put("Bottom Limit", wobbleGoalTopLimit.isPressed());
        drive.update();
        telemetry.update();
    }

    private void wobbleGoalExecution() {
        telemetry.addData("", "Inside of Wobble Goal Execution");

        if (!controlMode) {
            wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wobbleGoalMotor.setPower(-gamepad2.left_stick_y / 2);
        }

        if (wobbleGoalDropTimer.isFinished() && !controlMode) {
            telemetry.addData("", "About to drop Wobble Goal");
            System.out.println("About to drop Wobble Goal");
            wobbleGoalTimeSpacing(wobbleGoalDrop, wobbleGoalAngularSpeed);
            wobbleGoalDropTimer.reset();
        }
        if (wobbleGoalPickUpTimer.isFinished() && !controlMode) {
            telemetry.addData("", "About to pick up Wobble Goal");
            System.out.println("About to pick up Wobble Goal");
            wobbleGoalTimeSpacing(wobbleGoalPick, wobbleGoalAngularSpeed);
            wobbleGoalPickUpTimer.reset();
        }

        if (gamepad1.dpad_down && !lastDPadDown && !controlMode) {
            telemetry.addData("", "Setting Servo Drop");
            System.out.println("Setting Servo Drop");
            wobbleServo.setPosition(wobbleServoMax);
            wobbleGoalDropTimer.start();
        }
        lastDPadDown = gamepad1.dpad_down;

        if (gamepad1.dpad_up && !lastDPadUp && !controlMode) {
            telemetry.addData("", "Setting Servo Pick");
            System.out.println("Setting Servo Pick");
            wobbleServo.setPosition(Servo.MIN_POSITION);
            wobbleGoalPickUpTimer.start();
        }
        lastDPadUp = gamepad1.dpad_up;

        if (!wobbleGoalMotor.isBusy() && controlMode) {
            telemetry.addLine().addData("", "Resetting Position");
            System.out.println("Resetting Position");
            wobbleGoalMotor.setVelocity(0);
            controlMode = false;
        }

    }

    private void wobbleGoalTimeSpacing(int targetPosition, int angularSpeed) {
        telemetry.addData("", "Inside of Method, Wobble GoalTimeSpacing");
        System.out.println("Inside of Method, WobbleGoalTimeSpacing");
        controlMode = true;
        wobbleGoalMotor.setTargetPosition(targetPosition * wobbleGoalMotorTicksPerDegree);
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleGoalMotor.setVelocity(angularSpeed * wobbleGoalMotorTicksPerDegree);
    }

    private void resetLauncher() {
        boxServo.setPosition(boxInitialPosition);
        ringServo.setPosition(Servo.MIN_POSITION);
        setLauncherRPM(0);
    }

    @Override
    public void stop() {
        super.stop();
        drive = null;
    }

    @Override
    protected void composeTelemetry() {
        super.composeTelemetry();
        telemetry.addData("Accelerating", () -> acceleratePower != 0);
        telemetry.addData("Slow Mode", () -> slowMode);
    }
}