package org.firstinspires.ftc.teamcode.ultimategoal;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.OpMode8872;

@Config
abstract class UltimateGoalOpMode extends OpMode8872 {

    public static double boxInitialPosition = 0.35;
    public static double boxLauncherPosition = 0.73;
    protected static final int wobbleGoalPick = 0;
    protected static final int wobbleGoalDrop = -155;
    protected static final int wobbleGoalMotorGearRatio = 2;
    protected static final int wobbleGoalMotorTicksPerDegree = (5264 / 360) * wobbleGoalMotorGearRatio;

    protected DcMotorEx shooterFront, shooterBack, wobbleGoalMotor;
    protected DcMotor intake;

    protected Servo boxServo, ringServo, wobbleServo;


    @Override
    protected void initHardwareDevices() {
        shooterFront = (DcMotorEx) hardwareMap.dcMotor.get("shooterFront");
        shooterBack = (DcMotorEx) hardwareMap.dcMotor.get("shooterBack");
        wobbleGoalMotor = (DcMotorEx) hardwareMap.dcMotor.get("wobbleGoalMotor");

        intake = hardwareMap.dcMotor.get("intake");
        boxServo = hardwareMap.servo.get("boxServo");
        ringServo = hardwareMap.servo.get("ringServo");
        wobbleServo = hardwareMap.servo.get("wobbleServo");

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
        wobbleGoalMotor.setTargetPositionTolerance(160);
        boxServo.setDirection(Servo.Direction.REVERSE);

        boxServo.setPosition(0.35);
        ringServo.setPosition(0);
        wobbleServo.setPosition(Servo.MIN_POSITION);
    }

    @Override
    protected void composeTelemetry() {
        super.composeTelemetry();
        telemetry.addLine().addData("Intake Power", () -> intake.getPower());
        telemetry.addLine().addData("Launcher Front Velocity", () -> shooterFront.getVelocity(AngleUnit.DEGREES));
        telemetry.addLine().addData("Launcher Back Velocity", () -> shooterBack.getVelocity(AngleUnit.DEGREES));
        telemetry.addLine().addData("Box Servo", () -> boxServo.getPosition());
        telemetry.addLine().addData("Ring Servo", () -> ringServo.getPosition());
        telemetry.addLine().addData("Wobble Goal Position", () -> wobbleGoalMotor.getCurrentPosition() / wobbleGoalMotorTicksPerDegree);
    }

}