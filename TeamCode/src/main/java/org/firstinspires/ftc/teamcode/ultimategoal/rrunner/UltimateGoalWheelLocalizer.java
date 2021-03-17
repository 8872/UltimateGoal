package org.firstinspires.ftc.teamcode.ultimategoal.rrunner;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.rrunner.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
class UltimateGoalWheelLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.748031; // in - 19mm, 38 millimeter odometry wheels
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
    public BNO055IMU imu;
    // Measured lateral distance for competition bot: 14.5 inches
    // Lateral Distance which perfectly fits the heading: 14.702 inches
    // 2nd Measured Lateral Heading: 14.43in
    // TrackingWheelLateralDistanceTunerValue: 14.82
    public static double LATERAL_DISTANCE = 14.73; // in; distance between the left and right wheels
    // -1.25 Original value
    public static double FORWARD_OFFSET = -0.875;//-1.25; // in; offset of the lateral wheel

    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    private final Encoder frontEncoder;

    /* Lines 37-38 in StandardTrackingWheelLocalizer.java */
    public static double X_MULTIPLIER = 0.988263; //0.9852635446; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 0.98444483973; // Multiplier in the Y direction

    public static double PARALLEL_Y = -14.73 / 2; // X is the up and down direction
    public static double PARALLEL_X = 0.875; // Y is the strafe direction

    public static double PERPENDICULAR_Y = -0.375;
    public static double PERPENDICULAR_X = -1.125;

    public UltimateGoalWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
//                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
//                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
//                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
            new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
            new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));
        imu = hardwareMap.get(BNO055IMU.class, "imu");


        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        rightEncoder.setDirection(Encoder.Direction.REVERSE);


        //Fixing the correction(Strafe) problem: temporary
//        frontEncoder.setDirection(Encoder.Direction.REVERSE);

    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
//                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
//                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
            encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
            encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }

    @Override
    public Double getHeadingVelocity() {
        return Double.valueOf(imu.getAngularVelocity().zRotationRate);
    }

    @Override
    public double getHeading() {
        return Math.toRadians(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
    }
}
