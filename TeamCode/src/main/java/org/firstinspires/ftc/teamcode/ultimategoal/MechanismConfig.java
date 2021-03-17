package org.firstinspires.ftc.teamcode.ultimategoal;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Config
@Disabled
public class MechanismConfig extends UltimateGoalOpMode {
    private boolean leftTriggerState;
    private boolean lastDPadDown;
    private boolean lastDPadUp;
    private boolean lastDPadLeft;
    private boolean lastDPadRight;
    public final int wobbleServoOpen = 0;
    public final int wobbleServoClosed = 1;
    private boolean lastAState;
    private boolean lastXState;

    @Override
    public void loop() {
        leftTriggerState = (gamepad1.left_trigger > 0);

        if (gamepad1.dpad_left && !lastDPadLeft){
            boxServo.setPosition(0.1);
        }
        lastDPadLeft = gamepad1.dpad_left;

        if(gamepad1.dpad_right && !lastDPadRight){
            boxServo.setPosition(0.75);

        }
        lastDPadRight = gamepad1.dpad_right;

        if(gamepad1.dpad_down && !lastDPadDown){
            ringServo.setPosition(0);
        }
        lastDPadDown = gamepad1.dpad_down;

        if(gamepad1.dpad_up && !lastDPadUp){
            ringServo.setPosition(1);
        }
        lastDPadUp = gamepad1.dpad_up;

        if (gamepad1.a && !lastAState) {
            wobbleServo.setPosition(wobbleServoOpen);
        }
        lastAState = gamepad1.a;

        if (gamepad1.x && !lastXState) {
            wobbleServo.setPosition(wobbleServoClosed);
        }
        lastXState = gamepad1.x;


    }

    @Override
    protected void composeTelemetry(){

    }

}
