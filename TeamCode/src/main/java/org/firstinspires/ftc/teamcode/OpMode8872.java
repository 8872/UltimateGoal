package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.ultimategoal.rrunner.UltimateGoalDriveConstants;

import java.math.BigDecimal;
import java.math.RoundingMode;

public abstract class OpMode8872 extends OpMode {


    public static final DriveConstants DEFAULT_CONSTANTS = UltimateGoalDriveConstants.INSTANCE;
    /**
     * Calibration is not necessary, unless full accuracy of IMU is immediately needed
     * <p>
     * See the calibration sample opmode
     * {@link org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMUCalibration}
     */
    protected static boolean calibrateIMU = false;
    protected DcMotor leftRear, rightRear, leftFront, rightFront;
    protected BNO055IMU imu;


    @Override
    public final void init() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        if (calibrateIMU) {
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        }
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        telemetry.addData("Mode", "Initializing imu...");
        telemetry.update();
        imu.initialize(parameters);
        while (calibrateIMU && !isStopRequested() && !imu.isSystemCalibrated()) {
            sleep(50);
        }
        telemetry.addData("Mode", "Initializing hardware devices...");
        telemetry.update();

        // Define and Initialize Motors
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");

//        leftRear.setDirection(DcMotor.Direction.REVERSE); // uncomment if using AndyMark motors
//        leftFront.setDirection(DcMotor.Direction.REVERSE); // uncomment if using AndyMark motors
        rightRear.setDirection(DcMotor.Direction.REVERSE);// comment if using AndyMark motors
        rightFront.setDirection(DcMotor.Direction.REVERSE);// comment if using AndyMark motors


        //Set to brake mode
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        brake();

        initHardwareDevices();

        composeTelemetry();

        telemetry.addData("Mode", "Done initializing");
        telemetry.update();
    }

    protected abstract void initHardwareDevices();

    protected void brake() {
        leftRear.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }

    protected void composeTelemetry() {
        telemetry.addLine().addData("leftFront", () -> round(leftFront.getPower()));
        telemetry.addLine().addData("leftRear", () -> round(leftRear.getPower()));
        telemetry.addLine().addData("rightFront", () -> round(rightFront.getPower()));
        telemetry.addLine().addData("rightRear", () -> round(rightRear.getPower()));
    }

    protected void mechanumDrive(boolean slowMode) {
        mechanumDrive(slowMode, false, false);
    }

    protected void mechanumDrive(boolean slowMode, boolean xDisable, boolean yDisable) {
        double r;
        double robotAngle;
        if (xDisable) {
            r = Math.hypot(-gamepad1.left_stick_y, 0);
            robotAngle = Math.atan2(-gamepad1.left_stick_y, 0) - Math.PI / 4;
        } else if (yDisable) {
            r = Math.hypot(0, gamepad1.left_stick_x);
            robotAngle = Math.atan2(0, gamepad1.left_stick_x) - Math.PI / 4;
        } else {
            r = Math.hypot(-gamepad1.left_stick_y, gamepad1.left_stick_x);
            robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        }
        double rightX = gamepad1.right_stick_x;
        double v1 = (r * Math.cos(robotAngle) + rightX) * Math.sqrt(2);
        double v2 = (r * Math.sin(robotAngle) - rightX) * Math.sqrt(2);
        double v3 = (r * Math.sin(robotAngle) + rightX) * Math.sqrt(2);
        double v4 = (r * Math.cos(robotAngle) - rightX) * Math.sqrt(2);

        if (slowMode) {
            leftFront.setPower(v1 / 2);
            rightFront.setPower(v2 / 2);
            leftRear.setPower(v3 / 2);
            rightRear.setPower(v4 / 2);
        } else {
            leftFront.setPower(v1);
            rightFront.setPower(v2);
            leftRear.setPower(v3);
            rightRear.setPower(v4);
        }
    }

    protected double accelerate(double acceleratePower) {
        if (gamepad1.x && acceleratePower < 0.5) {
            acceleratePower += 0.001;
            rightFront.setPower(acceleratePower);
            rightRear.setPower(acceleratePower);
            leftRear.setPower(acceleratePower);
            leftFront.setPower(acceleratePower);
        } else if (!gamepad1.x && acceleratePower > 0) {
            acceleratePower -= 0.001;
            if (acceleratePower <= 0) {
                acceleratePower = 0;
            }
            rightFront.setPower(acceleratePower); // maybe make these negative
            rightRear.setPower(acceleratePower);
            leftRear.setPower(acceleratePower);
            leftFront.setPower(acceleratePower);
        }

        return acceleratePower;
    }

    protected static double toRPM(double degreesPerSecond) {
        return degreesPerSecond / 6;
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

    @SuppressWarnings("SameParameterValue")
    protected final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    protected final boolean isStopRequested() {
        return Thread.currentThread().isInterrupted();
    }
}
