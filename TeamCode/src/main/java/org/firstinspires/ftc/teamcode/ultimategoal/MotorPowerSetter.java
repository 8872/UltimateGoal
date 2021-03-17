package org.firstinspires.ftc.teamcode.ultimategoal;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
@Config
@Disabled
public class MotorPowerSetter extends UltimateGoalOpMode {

    //    private File rpm1Stored = AppUtil.getInstance().getSettingsFile("RPM1TestChassis.txt");
//    private File rpm2Stored = AppUtil.getInstance().getSettingsFile("RPM2TestChassis.txt");
//    public int rpm1 = Integer.parseInt(ReadWriteFile.readFile(rpm1Stored).trim());
//    public int rpm2 = Integer.parseInt(ReadWriteFile.readFile(rpm2Stored).trim());
    private int rpm1 = 0;
    private int rpm2 = 0;
    private String motorChangingSetting = "Motor1";
    private String name1 = "(Config) shooterFront Motor Power: ";
    private String name2 = "shooterBack Power: ";

    private final int buttonChange = 5;
    private final int stickChange = 10;

    private boolean lastAState1 = false;
    private boolean lastYState1 = false;
    private boolean lastXState1 = false;

    @Override
    public void loop() {
        // Change these two motors based on where you have it plugged in
        shooterFront.setVelocity(rpm1 * 28.0 / 60);
        shooterBack.setVelocity(rpm2 * 28.0 / 60);

        if (gamepad1.b && motorChangingSetting.equals("Motor1")) {
            rpm1 = 0;
        } else if (gamepad1.b && motorChangingSetting.equals("Motor2")) {
            rpm2 = 0;
        }

        if (gamepad1.x && !lastXState1 & motorChangingSetting.equals("Motor1")) {
            motorChangingSetting = "Motor2";
            name2 = "(Config) " + name2;
            name1 = "shooterFront Motor Power: ";

        } else if (gamepad1.x && !lastXState1) {
            motorChangingSetting = "Motor1";
            name1 = "(Config) " + name1;
            name2 = "shooterBack Motor Power: ";
        }
        lastXState1 = gamepad1.x;


        if (motorChangingSetting.equals("Motor1")) {
            rpm1 = motorPowerSetting(rpm1);
        } else {
            rpm2 = motorPowerSetting(rpm2);
        }

        if (motorChangingSetting.equals("Motor1")) {
            rpm1 += Math.round(gamepad1.left_stick_y * stickChange);
            rpm1 = Math.min(6000, rpm1);
            rpm1 = Math.max(-6000, rpm1);
        } else {
            rpm2 += Math.round(gamepad1.left_stick_y * stickChange);
            rpm2 = Math.min(6000, rpm2);
            rpm2 = Math.max(-6000, rpm2);
        }
//        ReadWriteFile.writeFile(rpm1Stored, String.valueOf(rpm1));
//        ReadWriteFile.writeFile(rpm2Stored, String.valueOf(rpm2));
    }

    private int motorPowerSetting(int rpm) {
        if (gamepad1.a && !lastAState1 && rpm > -6000) {
            if (rpm == 6000) {
                rpm -= buttonChange;
            } else {
                rpm -= Math.min(buttonChange, 6000 - Math.abs(rpm));
            }
        }
        lastAState1 = gamepad1.a;

        if (gamepad1.y && !lastYState1 && rpm <= 6000) {
            rpm += Math.min(buttonChange, 6000 - rpm);
        }
        lastYState1 = gamepad1.y;

        return rpm;
    }

    @Override
    protected void composeTelemetry() {
        super.composeTelemetry();
        telemetry.addLine().addData("Testing Purposes", () -> name1 + rpm1);
        telemetry.addLine().addData("TestingPurposes", () -> name2 + rpm2);

        telemetry.addLine().addData(name1 + " (Using Angle)", () -> toRPM(shooterFront.getVelocity(AngleUnit.DEGREES)));
        telemetry.addLine().addData(name2 + " (Using Angle)", () -> toRPM(shooterBack.getVelocity(AngleUnit.DEGREES)));
        telemetry.addLine().addData(name1 + " (Using Ticks)", () -> shooterFront.getVelocity() * 60.0 / 28.0);
        telemetry.addLine().addData(name2 + " (Using Ticks)", () -> shooterBack.getVelocity() * 60.0 / 28.0);

    }
}

