package org.firstinspires.ftc.teamcode.ultimategoal;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.ultimategoal.UltimateGoalOpMode;

@TeleOp
public class UltimateGoalDrive extends UltimateGoalOpMode {

    private boolean slowMode, accelerating;

    private boolean lastAState;
    private double acceleratePower = 0;


    @Override
    public void loop() {
        double r = Math.hypot(-gamepad1.left_stick_y, gamepad1.left_stick_x);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        double v1 = (r * Math.cos(robotAngle) + rightX) * Math.sqrt(2);
        double v2 = (r * Math.sin(robotAngle) - rightX) * Math.sqrt(2);
        double v3 = (r * Math.sin(robotAngle) + rightX) * Math.sqrt(2);
        double v4 = (r * Math.cos(robotAngle) - rightX) * Math.sqrt(2);


        if (gamepad1.y) {
            accelerating = false;
            acceleratePower = 0.0;
        }

        if (slowMode && !accelerating) {
            leftFront.setPower(v1 / 2);
            rightFront.setPower(v2 / 2);
            leftRear.setPower(v3 / 2);
            rightRear.setPower(v4 / 2);
        } else if (!accelerating) {
            leftFront.setPower(v1);
            rightFront.setPower(v2);
            leftRear.setPower(v3);
            rightRear.setPower(v4);
        }

        if (gamepad1.x) {
            accelerating = true;
            if (acceleratePower < 0.5) {
                acceleratePower += 0.001;
            }
            rightFront.setPower(acceleratePower);
            rightRear.setPower(acceleratePower);
            leftRear.setPower(acceleratePower);
            leftFront.setPower(acceleratePower);
        }

        if (acceleratePower > 0 && !gamepad1.x) {
            acceleratePower -= 0.001;
            if (acceleratePower <= 0) {
                acceleratePower = 0;
                accelerating = false;
            }
            rightFront.setPower(-acceleratePower);
            rightRear.setPower(-acceleratePower);
            leftRear.setPower(-acceleratePower);
            leftFront.setPower(-acceleratePower);
        }

        if (gamepad1.a && !lastAState) {
            slowMode = !slowMode;
        }
        lastAState = gamepad1.a;

        intake.setPower(gamepad1.right_trigger);
    }

    @Override
    protected void composeTelemetry() {
        super.composeTelemetry();
        telemetry.addLine().addData("Accelerating", () -> accelerating);
        telemetry.addLine().addData("Slow Mode", () -> slowMode);
        telemetry.addLine().addData("Intake", () -> intake.getPower());
    }
}