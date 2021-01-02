package org.firstinspires.ftc.teamcode.testchassis.rrunner;

import androidx.annotation.NonNull;
import androidx.annotation.XmlRes;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.748031; // in - 19mm, 38 millimeter odometry wheels
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 12.9583; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -4.75; // in; offset of the lateral wheel

    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    private final Encoder frontEncoder;

    /* Lines 37-38 in StandardTrackingWheelLocalizer.java */
    public static double X_MULTIPLIER = 0.97279; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1; // Multiplier in the Y direction


    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
            new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
            new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
            new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition())* Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
            encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
            encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
            encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}