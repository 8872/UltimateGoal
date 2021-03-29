package org.firstinspires.ftc.teamcode.ultimategoal;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.OpMode8872;

import java.util.function.BooleanSupplier;
import java.util.function.Predicate;

@Config
abstract class UltimateGoalOpMode extends OpMode8872 {

    public static double boxInitialPosition = 0.35;
    public static double boxLauncherPosition = 0.75;
    public static int wobbleGoalPick = -40;
    public static int wobbleGoalDrop = -185;
    public static int wobbleGoalAngularSpeed = -220;
    protected static final int wobbleGoalMotorGearRatio = 2;
    protected static int wobbleGoalMotorTicksPerDegree = (1425 / 360) * wobbleGoalMotorGearRatio;;


    protected DcMotorEx shooterFront, shooterBack, wobbleGoalMotor;
    protected DcMotor intake;
    protected TouchSensor wobbleGoalBottomLimit, wobbleGoalTopLimit;
    protected Servo boxServo, ringServo, wobbleServo, katanaServo;


    @Override
    protected void initHardwareDevices() {
        shooterFront = (DcMotorEx) hardwareMap.dcMotor.get("shooterFront");
        shooterBack = (DcMotorEx) hardwareMap.dcMotor.get("shooterBack");
        wobbleGoalMotor = (DcMotorEx) hardwareMap.dcMotor.get("wobbleGoalMotor");
        wobbleGoalBottomLimit = hardwareMap.touchSensor.get("wobbleGoalBottomLimit");
        wobbleGoalTopLimit = hardwareMap.touchSensor.get("wobbleGoalTopLimit");


        intake = hardwareMap.dcMotor.get("intake");
        boxServo = hardwareMap.servo.get("boxServo");
        ringServo = hardwareMap.servo.get("ringServo");
        wobbleServo = hardwareMap.servo.get("wobbleServo");
        katanaServo = hardwareMap.servo.get("katanaServo");

//        wobbleGoalMotorTicksPerDegree = ((int) wobbleGoalMotor.getMotorType().getTicksPerRev()/ 360) * wobbleGoalMotorGearRatio;

        shooterFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wobbleGoalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleGoalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterFront.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterBack.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        // original target tolerance: 110
        wobbleGoalMotor.setTargetPositionTolerance(30);
        boxServo.setDirection(Servo.Direction.REVERSE);
        boxServo.setPosition(0.35);
        ringServo.setPosition(0);
        katanaServo.setPosition(0);
        wobbleServo.setPosition(Servo.MIN_POSITION);


    }

    @Override
    protected void composeTelemetry() {
        super.composeTelemetry();
        telemetry.addLine("DPadDown drops Wobble Goal, DPadUp lifts Wobble Goal");
        telemetry.addData("Intake Power", intake::getPower);
        telemetry.addData("Launcher Front Velocity", () -> shooterFront.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("Wobble Goal Encoder Ticks", () -> wobbleGoalMotor.getMotorType().getTicksPerRev());
        telemetry.addData("Gearing", () -> wobbleGoalMotor.getMotorType().getGearing());
        telemetry.addData("Launcher Back Velocity", () -> shooterBack.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("Launcher Back Current", () -> shooterBack.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Launcher Front Current", () -> shooterFront.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Box Servo", boxServo::getPosition);
        telemetry.addData("Ring Servo", ringServo::getPosition);
        telemetry.addData("Katana Servo", katanaServo::getPosition);
        telemetry.addData("Wobble Goal Position", () -> wobbleGoalMotor.getCurrentPosition() / wobbleGoalMotorTicksPerDegree);
        telemetry.addData("Wobble Motor Is Busy: ", wobbleGoalMotor::isBusy);
        telemetry.addData("WobbleGoalMotorMode: ", wobbleGoalMotor::getMode);
//        telemetry.addData("Bottom Limit", wobbleGoalBottomLimit::isPressed);
//        telemetry.addData("Top Limit", wobbleGoalTopLimit::isPressed);
    }

    protected void setLauncherRPM(int RPM){
        shooterBack.setVelocity(RPM * 28.0 / 60); //convert to encoder ticks
        shooterFront.setVelocity(RPM * 28.0 / 60);
    }
    protected void dropWobbleGoal() {
        wobbleGoalMechanism(wobbleGoalDrop);
        wobbleServo.setPosition(Servo.MAX_POSITION);
        whileSleep(1000);
    }

    protected void resetWobbleGoal() {
        wobbleServo.setPosition(Servo.MIN_POSITION);
        whileSleep(1000);
        wobbleGoalMechanism(-30);
    }

    private void wobbleGoalMechanism(int motorTargetDegrees) {
        wobbleGoalMotor.setTargetPosition(motorTargetDegrees * wobbleGoalMotorTicksPerDegree);
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleGoalMotor.setVelocity(wobbleGoalAngularSpeed * 2.8);
        whileSleep(wobbleGoalMotor::isBusy);
        wobbleGoalMotor.setVelocity(0);
    }

}