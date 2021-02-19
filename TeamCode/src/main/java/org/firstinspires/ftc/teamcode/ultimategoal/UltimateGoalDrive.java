package org.firstinspires.ftc.teamcode.ultimategoal;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Timer;

@Config
@TeleOp
public class UltimateGoalDrive extends UltimateGoalOpMode {



    private static final double launcherRPMSpeed = 3500;
    private static final double intakePower = 0.5;

    private final Timer boxServoTimer = new Timer(1000); // try 300
    private final Timer kickerServoTimer = new Timer(1000); // try 200
    private final Timer kickerServoTimer2 = new Timer(1000); // try 400
    private final Timer wobbleGoalPickUpTimer = new Timer(700);
    private final Timer wobbleGoalDropTimer = new Timer(1500);

    private enum ShooterState {IDLE, BOX, KICKER1, KICKER2}

    ;
    private ShooterState shooterState = ShooterState.IDLE;

    private boolean slowMode, controlMode;
    private boolean lastAState, lastBumperState, leftTriggerState, lastDPadDown, lastDPadUp;
    private boolean xDisable, yDisable;
    private double acceleratePower = 0;

    @Override
    public void loop() {
        if ((gamepad1.left_trigger > 0) && !leftTriggerState && !xDisable && !yDisable) {
            xDisable = true;
        } else if ((gamepad1.left_trigger > 0) && !leftTriggerState && xDisable && !yDisable) {
            yDisable = true;
            xDisable = false;
        } else if ((gamepad1.left_trigger > 0) && !leftTriggerState && !xDisable && yDisable) {
            xDisable = false;
            yDisable = false;
        }
        leftTriggerState = (gamepad1.left_trigger > 0);


        if (gamepad1.y) {
            acceleratePower = 0;
        }
        if (gamepad1.x || acceleratePower != 0) {
            acceleratePower = accelerate(acceleratePower);
        } else {
            mechanumDrive(slowMode, xDisable, yDisable);
        }

        if (gamepad1.a && !lastAState) {
            slowMode = !slowMode;
        }
        lastAState = gamepad1.a;

        if (!controlMode) {
            wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wobbleGoalMotor.setPower(gamepad2.left_stick_y / 2);
        }

        if (wobbleGoalDropTimer.isFinished()) {
            wobbleGoalTimeSpacing(wobbleGoalDrop, 20);

            wobbleGoalDropTimer.reset();
        }
        if (wobbleGoalPickUpTimer.isFinished()) {
            wobbleGoalTimeSpacing(wobbleGoalPick, 20);
            wobbleGoalPickUpTimer.reset();
        }

        if (gamepad1.dpad_down && !lastDPadDown && !wobbleGoalMotor.isBusy()) {
            wobbleServo.setPosition(Servo.MAX_POSITION);
            wobbleGoalDropTimer.start();
        }
        lastDPadDown = gamepad1.dpad_down;

        if (gamepad1.dpad_up && !lastDPadUp && !wobbleGoalMotor.isBusy()) {
            wobbleServo.setPosition(Servo.MIN_POSITION);
            wobbleGoalPickUpTimer.start();
        }
        lastDPadUp = gamepad1.dpad_up;

        if (!wobbleGoalMotor.isBusy()) {
            wobbleGoalMotor.setVelocity(0);
            controlMode = false;
        }
        int currentPos = wobbleGoalMotor.getCurrentPosition() / wobbleGoalMotorTicksPerDegree;
        if (currentPos > Math.max(wobbleGoalPick, wobbleGoalDrop) + 12 || currentPos < Math.min(wobbleGoalPick, wobbleGoalDrop) - 12) {
            wobbleGoalMotor.setVelocity(0);
        }

        if ((gamepad1.left_bumper && gamepad1.right_bumper) && !lastBumperState) {
            intake.setPower(0);
        } else if (gamepad1.left_bumper && !lastBumperState) {
            intake.setPower(intakePower);
        } else if (gamepad1.right_bumper && !lastBumperState) {
            intake.setPower(-intakePower);
        }
        lastBumperState = gamepad1.left_bumper || gamepad1.right_bumper;


        if (gamepad1.right_trigger == 0 && shooterState != ShooterState.IDLE) {
            shooterState = ShooterState.IDLE;
            resetLauncher();
        }
        switch (shooterState) {
            case IDLE:
                if (gamepad1.right_trigger != 0) {
                    telemetry.addData("reset", false);
                    shooterFront.setVelocity(launcherRPMSpeed * 6, AngleUnit.DEGREES);
                    shooterBack.setVelocity(launcherRPMSpeed * 6, AngleUnit.DEGREES);
                    shooterState = ShooterState.BOX;
                    boxServo.setPosition(boxLauncherPosition);
                    boxServoTimer.start();
                }
                break;
            case BOX:
                if (boxServoTimer.isFinished()) {
                    shooterState = ShooterState.KICKER1;
                    ringServo.setPosition(Servo.MAX_POSITION);
                    kickerServoTimer.start();
                }
                break;
            case KICKER1:
                if (kickerServoTimer.isFinished()) {
                    shooterState = ShooterState.KICKER2;
                    ringServo.setPosition(Servo.MIN_POSITION);
                    kickerServoTimer2.start();
                }
                break;
            case KICKER2: // move kicker out again
                if (kickerServoTimer2.isFinished()) {
                    shooterState = ShooterState.KICKER1;
                    ringServo.setPosition(Servo.MAX_POSITION);
                    kickerServoTimer.start();
                }
                break;
        }
    }

    private void wobbleGoalTimeSpacing(int targetPosition, int angularSpeed) {
        controlMode = true;
        wobbleGoalMotor.setTargetPosition(targetPosition * wobbleGoalMotorTicksPerDegree);
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleGoalMotor.setVelocity(angularSpeed * wobbleGoalMotorTicksPerDegree);
    }

    private void resetLauncher() {
        boxServo.setPosition(boxInitialPosition);
        ringServo.setPosition(Servo.MIN_POSITION);
        shooterFront.setPower(0);
        shooterBack.setPower(0);
    }

    @Override
    protected void composeTelemetry() {
        super.composeTelemetry();
        telemetry.addLine().addData("Accelerating", () -> acceleratePower != 0);
        telemetry.addLine().addData("Slow Mode", () -> slowMode);
        telemetry.addLine().addData("Launcher Front Velocity", () -> shooterFront.getVelocity(AngleUnit.DEGREES));
        telemetry.addLine().addData("Launcher Back Velocity", () -> shooterBack.getVelocity(AngleUnit.DEGREES));
        telemetry.addLine().addData("Wobble Goal Position", () -> wobbleGoalMotor.isBusy());
        telemetry.addLine().addData("TargetPositionTolerance", () -> wobbleGoalMotor.getTargetPositionTolerance());
    }
}