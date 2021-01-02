package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class NullDcMotor extends NullHardwareDevice implements DcMotorEx {

    public static final NullDcMotor INSTANCE = new NullDcMotor();

    private NullDcMotor() {
    }

    @Override
    public void setMotorEnable() {}

    @Override
    public void setMotorDisable() { }

    @Override
    public boolean isMotorEnabled() { return true; }

    @Override
    public void setVelocity(double angularRate) { }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) { }

    @Override
    public double getVelocity() { return 0; }

    @Override
    public double getVelocity(AngleUnit unit) { return 0; }

    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) { }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException { }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) { }

    @Override
    public void setPositionPIDFCoefficients(double p) { }

    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return new PIDCoefficients();
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return new PIDFCoefficients();
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) { }

    @Override
    public int getTargetPositionTolerance() { return 0; }

    @Override
    public double getCurrent(CurrentUnit unit) { return 0; }

    @Override
    public double getCurrentAlert(CurrentUnit unit) { return 0; }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) { }

    @Override
    public boolean isOverCurrent() { return false; }

    @Override
    public MotorConfigurationType getMotorType() {
        return MotorConfigurationType.getUnspecifiedMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) { }

    @Override
    public DcMotorController getController() { return null; }

    @Override
    public int getPortNumber() { return 0; }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) { }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return ZeroPowerBehavior.UNKNOWN;
    }

    @Override
    public void setPowerFloat() { }

    @Override
    public boolean getPowerFloat() {
        return false;
    }

    @Override
    public void setTargetPosition(int position) { }

    @Override
    public int getTargetPosition() { return 0; }

    @Override
    public boolean isBusy() { return false; }

    @Override
    public int getCurrentPosition() { return 0; }

    @Override
    public void setMode(RunMode mode) { }

    @Override
    public RunMode getMode() {
        return RunMode.RUN_WITHOUT_ENCODER;
    }

    @Override
    public void setDirection(Direction direction) { }

    @Override
    public Direction getDirection() {
        return Direction.FORWARD;
    }

    @Override
    public void setPower(double power) { }

    @Override
    public double getPower() { return 0; }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Unknown;
    }

    @Override
    public String getDeviceName() { return "Null DcMotor"; }

    @Override
    public String getConnectionInfo() { return "Null DcMotor"; }

}
