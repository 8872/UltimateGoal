package org.firstinspires.ftc.teamcode.testchassis;

import java.math.BigDecimal;
import java.math.RoundingMode;

//@TeleOp
public class MecanumDriveOnly extends TestChassisOpMode {

    private boolean slowMode = false, accelerating = false;

    @Override
    public void runOpMode() {
        initialize();
        boolean lastAState = false;
        double acceleratePower = 0.0;
        composeTelemetry();
        waitForStart();

        while (opModeIsActive()) {
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

            telemetry.update();
        }
    }

    private void composeTelemetry() {
        telemetry.addLine().addData("leftFront", () -> round(leftFront.getPower()));
        telemetry.addLine().addData("leftRear", () -> round(leftRear.getPower()));
        telemetry.addLine().addData("rightFront", () -> round(rightFront.getPower()));
        telemetry.addLine().addData("rightRear", () -> round(rightRear.getPower()));
        telemetry.addLine().addData("Accelerating", () -> accelerating);
        telemetry.addLine().addData("Slow Mode", () -> slowMode);
    }

    private static double round(double value) {
        return round(value, 4);
    }

    private static double round(double value, @SuppressWarnings("SameParameterValue") int places) {
        if (places < 0) throw new IllegalArgumentException();

        BigDecimal bd = new BigDecimal(Double.toString(value));
        bd = bd.setScale(places, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }
}