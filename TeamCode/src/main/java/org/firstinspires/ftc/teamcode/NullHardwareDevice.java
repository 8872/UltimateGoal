package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareDevice;

public class NullHardwareDevice implements HardwareDevice {

    public static final NullHardwareDevice INSTANCE = new NullHardwareDevice();

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Unknown;
    }

    @Override
    public String getDeviceName() {
        return "Null HardwareDevice";
    }

    @Override
    public String getConnectionInfo() {
        return "Null HardwareDevice";
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
