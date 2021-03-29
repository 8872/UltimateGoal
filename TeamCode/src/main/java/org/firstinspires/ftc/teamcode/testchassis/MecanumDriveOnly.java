package org.firstinspires.ftc.teamcode.testchassis;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.OpMode8872;

@Disabled
@TeleOp
public class MecanumDriveOnly extends OpMode8872 {

    private boolean slowMode = false;

    private boolean lastAState = false;
    private double acceleratePower = 0.0;


    @Override
    public void loop() {
        if (gamepad1.y) {
            acceleratePower = 0.0;
        }
        if (gamepad1.x || acceleratePower > 0) {
            acceleratePower = accelerate(acceleratePower);
        }

        if (acceleratePower == 0) {
            mechanumDrive(slowMode);
        }

        if (gamepad1.a && !lastAState) {
            slowMode = !slowMode;
        }
        lastAState = gamepad1.a;
    }

    @Override
    protected void composeTelemetry() {
        super.composeTelemetry();
        telemetry.addData("Accelerating", () -> acceleratePower != 0.0);
        telemetry.addData("Slow Mode", () -> slowMode);
    }

    @Override
    protected void initHardwareDevices() {}

}