package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.lang.reflect.Field;
import java.math.BigDecimal;
import java.math.RoundingMode;

public abstract class OpMode8872 extends OpMode {


    /**
     * Calibration is not necessary, unless full accuracy of IMU is immediately needed
     *
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
//        leftRear = hardwareMap.dcMotor.get("leftRear");
//        rightRear = hardwareMap.dcMotor.get("rightRear");
//        leftFront = hardwareMap.dcMotor.get("leftFront");
//        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = NullDcMotor.INSTANCE;
        rightRear = NullDcMotor.INSTANCE;
        leftFront = NullDcMotor.INSTANCE;
        rightFront = NullDcMotor.INSTANCE;

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
