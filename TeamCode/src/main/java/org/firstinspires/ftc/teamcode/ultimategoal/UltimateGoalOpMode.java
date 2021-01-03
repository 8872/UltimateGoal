package org.firstinspires.ftc.teamcode.ultimategoal;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.OpMode8872;

public abstract class UltimateGoalOpMode extends OpMode8872 {

    protected DcMotorEx shooterFront, shooterBack;
    protected DcMotor intake;

    protected Servo boxServo, ringServo;

    @Override
    protected void initHardwareDevices() {
//        shooter1 = (DcMotorEx) hardwareMap.dcMotor.get("shooter1");
//        shooter2 = (DcMotorEx) hardwareMap.dcMotor.get("shooter2");
        shooter1 = NullDcMotor.INSTANCE;
        shooter2 = NullDcMotor.INSTANCE;
        intake = hardwareMap.dcMotor.get("intake");
//        boxServo = hardwareMap.servo.get("boxServo");
//        ringServo = hardwareMap.servo.get("boxServo");
        shooterFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterFront.setPower(0);
        shooterBack.setPower(0);
    }

}