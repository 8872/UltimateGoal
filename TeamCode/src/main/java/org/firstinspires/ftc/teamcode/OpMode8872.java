package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.rrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ultimategoal.rrunner.UltimateGoalDriveConstants;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.function.BooleanSupplier;

public abstract class OpMode8872 extends OpMode {


    public static final DriveConstants DEFAULT_CONSTANTS = UltimateGoalDriveConstants.INSTANCE;
    protected SampleMecanumDrive drive;
    protected Telemetry dashTelemetry;
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
        dashTelemetry = FtcDashboard.getInstance().getTelemetry();

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

    protected void update() {
        telemetry.update();
        drive.update();
    }

    protected void brake() {
        leftRear.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }

    protected void composeTelemetry() {
        telemetry.addData("leftFront", () -> round(leftFront.getPower()));
        telemetry.addData("leftRear", () -> round(leftRear.getPower()));
        telemetry.addData("rightFront", () -> round(rightFront.getPower()));
        telemetry.addData("rightRear", () -> round(rightRear.getPower()));
        telemetry.addData("imu x", () -> imu.getPosition().x);
        telemetry.addData("imu y", () -> imu.getPosition().y);
        telemetry.addData("Imu Heading", () -> imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
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
            drive.setMotorPowers(acceleratePower, acceleratePower, acceleratePower, acceleratePower);
        } else if (!gamepad1.x && acceleratePower > 0) {
            acceleratePower -= 0.001;
            if (acceleratePower <= 0) {
                acceleratePower = 0;
            }
            drive.setMotorPowers(acceleratePower, acceleratePower, acceleratePower, acceleratePower);
        }
        drive.update();

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

    @SuppressWarnings("SameParameterValue")
    protected void whileSleep(long millis) {
        final long end = System.currentTimeMillis() + millis;
        whileSleep(() -> System.currentTimeMillis() < end);
    }

    protected void whileSleep(BooleanSupplier supplier) {
        while (!isStopRequested() && supplier.getAsBoolean()) {
            Thread.yield();
            update();
            sleep(50);
            Thread.yield();
        }
    }

    protected void waitForRoadRunnerIdle() {
        whileSleep(drive::isBusy);
    }

    protected final boolean isStopRequested() {
        return Thread.currentThread().isInterrupted();
    }
}
