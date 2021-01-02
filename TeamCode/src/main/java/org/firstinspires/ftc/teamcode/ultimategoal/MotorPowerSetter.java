package org.firstinspires.ftc.teamcode.ultimategoal;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.OpMode8872;

@TeleOp
@Config

public class MotorPowerSetter extends UltimateGoalOpMode {

    public int RPM1 = 0;
    public int RPM2 = 0;
    public String MotorChangingSetting = "Motor1";
    public String name1 = "(Config) shooter1 Motor Power: ";
    public String name2 = "shooter2 Motor Power: ";

    public final int buttonChange = 5;
    public final int stickChange = 10;

    private boolean lastAState1 = false;
    private boolean lastYState1 = false;
    private boolean lastXState1 = false;
    @Override
    public void loop(){
        telemetry.addLine().addData(name1, () -> RPM1);
        telemetry.addLine().addData(name2, () -> RPM2);

        // Change these two motors based on where you have it plugged in
        shooter1.setPower(RPM1 / 6000.0);
        shooter2.setPower(RPM2 / 6000.0);

        if (gamepad1.x && !lastXState1 & MotorChangingSetting.equals("Motor1")) {
            MotorChangingSetting = "Motor2";
            name2  = "(Config) " + name2;
            name1 = "shooter1 Motor Power: ";

        }else if(gamepad1.x && !lastXState1) {
            MotorChangingSetting = "Motor1";
            name1 = "(Config) " + name1;
            name2 = "shooter2 Motor Power: ";
        }
        lastXState1 = gamepad1.x;


        if (MotorChangingSetting.equals("Motor1")) {
            RPM1 = MotorPowerSetting(RPM1);
        } else {
            RPM2 = MotorPowerSetting(RPM2);
        }

        if(MotorChangingSetting.equals("Motor1")){
            RPM1 += Math.round(gamepad1.left_stick_y * stickChange);
        }else{
            RPM2 += Math.round(gamepad1.left_stick_y * stickChange);
        }
    }

    public int MotorPowerSetting(int RPM){
        if (gamepad1.a && !lastAState1 && RPM >= buttonChange) {
            RPM += buttonChange;
        }
        lastAState1 = gamepad1.a;

        if (gamepad1.y && !lastYState1 && RPM >= buttonChange) {
            RPM -= buttonChange;
        }
        lastYState1 = gamepad1.y;

        return RPM;
    }
}

