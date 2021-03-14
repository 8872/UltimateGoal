package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.rrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ultimategoal.rrunner.UltimateGoalDriveConstants;

import java.math.BigDecimal;
import java.math.RoundingMode;

public abstract class OpMode8872 extends OpMode {


    public static final DriveConstants DEFAULT_CONSTANTS = UltimateGoalDriveConstants.INSTANCE;
    protected SampleMecanumDrive drive;
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
        if (hardwareMap.voltageSensor.get("Control Hub").getVoltage() < 12.3) {
            RobotLog.addGlobalWarningMessage("Battery voltage is very low. Motors may not run at full speed");
        }

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        telemetry.addData("Mode", "Initializing imu...");
        telemetry.update();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

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

        drive = new SampleMecanumDrive(hardwareMap, UltimateGoalDriveConstants.INSTANCE);

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
        telemetry.addLine().addData("imu x", () -> imu.getPosition().x);
        telemetry.addLine().addData("imu x", () -> imu.getPosition().y);
        telemetry.addLine().addData("Imu Heading", () -> imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
    }

    protected void mechanumDrive(boolean slowMode) {
        mechanumDrive(slowMode, false, false);
    }

    protected void mechanumDrive(boolean slowMode, boolean xDisable, boolean yDisable) {
        double horizontal = -gamepad1.left_stick_x;
        double vertical = -gamepad1.left_stick_y;
        double angle = -gamepad1.right_stick_x;

        if (xDisable) {
            horizontal = 0;
        }
        if (yDisable) {
            vertical = 0;
        }
        if (slowMode) {
            horizontal = horizontal / 2;
            vertical = vertical / 2;
            angle = angle / 2;
        }

        drive.setWeightedDrivePower(vertical, horizontal, angle); // in roadrunner x and y are reversed
        drive.update();
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
