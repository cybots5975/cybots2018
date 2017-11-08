package org.firstinspires.ftc.teamcode.drivebase.swerve.core;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;

/**
 * Created by jordanbrobots on 11/8/17.
 */

public class FakeAnalogInput extends AnalogInput {
    /**
     * Constructor
     *
     * @param controller AnalogInput controller this channel is attached to
     * @param channel    channel on the analog input controller
     */
    public FakeAnalogInput(AnalogInputController controller, int channel) {
        super(controller, channel);
    }

    @Override public Manufacturer getManufacturer() {
        //return controller.getManufacturer();
        return null;
    }

    /**
     * Returns the current voltage of this input.
     * @return the current analog input voltage, in volts.
     */
    public double getVoltage() {
        //return controller.getAnalogInputVoltage(channel);
        return 0;
    }

    /**
     * Returns the maximum value that getVoltage() is capable of reading
     * @return the maximum value that getVoltage() is capable of reading, in volts.
     * @see #getVoltage()
     */
    public double getMaxVoltage() {
        //return controller.getMaxAnalogInputVoltage();
        return 0;
    }

    @Override
    public String getDeviceName() {
        //return AppUtil.getDefContext().getString(com.qualcomm.robotcore.R.string.configTypeAnalogInput);
        return null;
    }

    @Override
    public String getConnectionInfo() {
        //return controller.getConnectionInfo() + "; analog port " + channel;
        return null;
    }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
    }

    @Override
    public void close() {
        // take no action
    }
}
