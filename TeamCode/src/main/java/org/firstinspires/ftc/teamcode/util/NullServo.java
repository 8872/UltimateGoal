package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class NullServo extends NullHardwareDevice implements Servo {

    public static final NullServo INSTANCE = new NullServo();

    private NullServo() {
    }

    @Override
    public ServoController getController() { return null; }

    @Override
    public int getPortNumber() { return 0; }

    @Override
    public void setDirection(Direction direction) { }

    @Override
    public Direction getDirection() {
        return Direction.FORWARD;
    }

    @Override
    public void setPosition(double position) { }

    @Override
    public double getPosition() {
        return 0;
    }

    @Override
    public void scaleRange(double min, double max) { }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Unknown;
    }

    @Override
    public String getDeviceName() {
        return "";
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

}
