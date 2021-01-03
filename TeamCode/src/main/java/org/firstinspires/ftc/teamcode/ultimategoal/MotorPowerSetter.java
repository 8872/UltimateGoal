package org.firstinspires.ftc.teamcode.ultimategoal;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Config
public class MotorPowerSetter extends UltimateGoalOpMode {

    public int rpm1 = 0;
    public int rpm2 = 0;
    public String motorChangingSetting = "Motor1";
    public String name1 = "(Config) shooter1 Motor Power: ";
    public String name2 = "shooter2 Motor Power: ";

    public final int buttonChange = 5;
    public final int stickChange = 10;

    private boolean lastAState1 = false;
    private boolean lastYState1 = false;
    private boolean lastXState1 = false;

    @Override
    public void loop() {
        telemetry.addLine().addData(name1, () -> rpm1);
        telemetry.addLine().addData(name2, () -> rpm2);

        // Change these two motors based on where you have it plugged in
        shooterFront.setPower(rpm1 / 6000.0);
        shooterBack.setPower(rpm2 / 6000.0);

        if (gamepad1.x && !lastXState1 & motorChangingSetting.equals("Motor1")) {
            motorChangingSetting = "Motor2";
            name2 = "(Config) " + name2;
            name1 = "shooter1 Motor Power: ";

        } else if (gamepad1.x && !lastXState1) {
            motorChangingSetting = "Motor1";
            name1 = "(Config) " + name1;
            name2 = "shooter2 Motor Power: ";
        }
        lastXState1 = gamepad1.x;


        if (motorChangingSetting.equals("Motor1")) {
            rpm1 = motorPowerSetting(rpm1);
        } else {
            rpm2 = motorPowerSetting(rpm2);
        }

        if (motorChangingSetting.equals("Motor1")) {
            rpm1 += Math.round(gamepad1.left_stick_y * stickChange);
        } else {
            rpm2 += Math.round(gamepad1.left_stick_y * stickChange);
        }
    }

    public int motorPowerSetting(int rpm) {
        if (gamepad1.a && !lastAState1 && rpm >= buttonChange) {
            rpm += buttonChange;
        }
        lastAState1 = gamepad1.a;

        if (gamepad1.y && !lastYState1 && rpm >= buttonChange) {
            rpm -= buttonChange;
        }
        lastYState1 = gamepad1.y;

        return rpm;
    }
}

