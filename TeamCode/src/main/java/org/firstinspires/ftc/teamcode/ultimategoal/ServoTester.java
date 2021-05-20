package org.firstinspires.ftc.teamcode.ultimategoal;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class ServoTester extends UltimateGoalOpMode {

    public static double boxPosition = 0;
    public static double wobbleGoalPosition = 0;
    public static double intakeSpeed = 0.5;

    @Override
    public void loop() {
        boxServo.setPosition(boxPosition);
        intake.setPower(intakeSpeed);
    }

}
