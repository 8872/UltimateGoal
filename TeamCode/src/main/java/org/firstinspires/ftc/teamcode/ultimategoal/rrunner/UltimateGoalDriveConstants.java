package org.firstinspires.ftc.teamcode.ultimategoal.rrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.DriveConstants;

/*
 * Constants shared between multiple drive types.
 *
 * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
 * fields may also be edited through the dashboard (connect to the robot's WiFi network and
 * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
 * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
 *
 * These are not the only parameters; some are located in the localizer classes, drive base classes,
 * and op modes themselves.
 */
@Config
public class UltimateGoalDriveConstants extends DriveConstants {

    public static final UltimateGoalDriveConstants INSTANCE = new UltimateGoalDriveConstants();

    private UltimateGoalDriveConstants() {super(LATERAL_MULTIPLIER, TRANSLATIONAL_PID, HEADING_PID, TICKS_PER_REV, MAX_RPM, RUN_USING_ENCODER, MOTOR_VELO_PID, WHEEL_RADIUS, GEAR_RATIO, TRACK_WIDTH, kV, kA, kStatic, BASE_CONSTRAINTS, UltimateGoalWheelLocalizer.LATERAL_DISTANCE);}

    private static final double LATERAL_MULTIPLIER = 1.33;
    private static final PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(12, 1, 1);
    private static final PIDCoefficients HEADING_PID = new PIDCoefficients(4, 1, 1);

    /*
     * These are motor constants that should be listed online for your motors.
     */
    private static final double TICKS_PER_REV = 537.6;
    // Actual max RPM: 349
    private static final double MAX_RPM = 340;

    /*
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., tracking wheels).
     *
     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */
    private static final boolean RUN_USING_ENCODER = false;
    private static final PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0, INSTANCE.getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    private static final double WHEEL_RADIUS = 1.9685; //  in - 50mm
    private static final double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    // Original measured track width: 14.75
    // 2nd: 13.75
    //3rd 14.5
    // 4th 13.68
    private static final double TRACK_WIDTH = 13.3; // in

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    // original kV: 1.0 / rpmToVelocity(MAX_RPM);
    // original 0.01845
    private static final double kV = 0.0172;
    private static final double kA = 0.003;
    private static final double kStatic = 0.04;//0;


    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. The velocity and
     * acceleration values are required, and the jerk values are optional (setting a jerk of 0.0
     * forces acceleration-limited profiling). All distance units are inches.
     */
    // Max Velocity Calculated: (MAX_RPM / 60) * GEAR_RATIO * WHEEL_RADIUS * Math.PI * 2
    private static final DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
        40, 30.0, 0.0,
        Math.toRadians(180.0), Math.toRadians(180.0), 0.0
    );

    @Override
    public double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    @Override
    public double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }

    @Override
    public Localizer createLocalizer(HardwareMap hardwareMap) {
        return new UltimateGoalWheelLocalizer(hardwareMap);
    }
}
