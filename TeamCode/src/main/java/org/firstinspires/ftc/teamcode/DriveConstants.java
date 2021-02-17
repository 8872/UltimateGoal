package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.jetbrains.annotations.NotNull;

public abstract class DriveConstants {

    public final double LATERAL_MULTIPLIER;
    public final PIDCoefficients TRANSLATIONAL_PID;
    public final PIDCoefficients HEADING_PID;
    public final double TICKS_PER_REV;
    public final double MAX_RPM;
    public final boolean RUN_USING_ENCODER;
    public final PIDFCoefficients MOTOR_VELO_PID;
    public final double WHEEL_RADIUS;
    public final double GEAR_RATIO;
    public final double TRACK_WIDTH;
    public final double kV;
    public final double kA;
    public final double kStatic;
    public final DriveConstraints BASE_CONSTRAINTS;
    public final double LATERAL_DISTANCE;

    protected DriveConstants(double LATERAL_MULTIPLIER, PIDCoefficients TRANSLATIONAL_PID, PIDCoefficients HEADING_PID, double TICKS_PER_REV, double MAX_RPM, boolean RUN_USING_ENCODER, PIDFCoefficients MOTOR_VELO_PID, double WHEEL_RADIUS, double GEAR_RATIO, double TRACK_WIDTH, double kV, double kA, double kStatic, DriveConstraints BASE_CONSTRAINTS, double LATERAL_DISTANCE) {
        this.LATERAL_MULTIPLIER = LATERAL_MULTIPLIER;
        this.TRANSLATIONAL_PID = TRANSLATIONAL_PID;
        this.HEADING_PID = HEADING_PID;
        this.TICKS_PER_REV = TICKS_PER_REV;
        this.MAX_RPM = MAX_RPM;
        this.RUN_USING_ENCODER = RUN_USING_ENCODER;
        this.MOTOR_VELO_PID = MOTOR_VELO_PID;
        this.WHEEL_RADIUS = WHEEL_RADIUS;
        this.GEAR_RATIO = GEAR_RATIO;
        this.TRACK_WIDTH = TRACK_WIDTH;
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
        this.BASE_CONSTRAINTS = BASE_CONSTRAINTS;
        this.LATERAL_DISTANCE = LATERAL_DISTANCE;
    }

    public abstract double encoderTicksToInches(double ticks);

    public abstract double rpmToVelocity(double rpm);

    public abstract double getMotorVelocityF(double ticksPerSecond);

    public abstract Localizer createLocalizer(HardwareMap hardwareMap);

    @NotNull
    @Override
    public String toString() {
        return getClass().getSimpleName() + "{" +
            "LATERAL_MULTIPLIER=" + LATERAL_MULTIPLIER +
            ", TRANSLATIONAL_PID=" + TRANSLATIONAL_PID +
            ", HEADING_PID=" + HEADING_PID +
            ", TICKS_PER_REV=" + TICKS_PER_REV +
            ", MAX_RPM=" + MAX_RPM +
            ", RUN_USING_ENCODER=" + RUN_USING_ENCODER +
            ", MOTOR_VELO_PID=" + MOTOR_VELO_PID +
            ", WHEEL_RADIUS=" + WHEEL_RADIUS +
            ", GEAR_RATIO=" + GEAR_RATIO +
            ", TRACK_WIDTH=" + TRACK_WIDTH +
            ", kV=" + kV +
            ", kA=" + kA +
            ", kStatic=" + kStatic +
            ", BASE_CONSTRAINTS=" + BASE_CONSTRAINTS +
            ", LATERAL_DISTANCE=" + LATERAL_DISTANCE +
            '}';
    }
}
