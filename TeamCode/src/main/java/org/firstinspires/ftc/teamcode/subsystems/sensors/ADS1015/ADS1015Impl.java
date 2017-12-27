package org.firstinspires.ftc.teamcode.subsystems.sensors.ADS1015;

import android.util.Log;

import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

/**
 * ADS1015 Sensor Implementation
 * @author Jaxon A Brown
 */
@I2cSensor(name = "ADS1015 ADC", description = "Texas Instruments ADC", xmlTag = "ADS1015")
public class ADS1015Impl extends I2cDeviceSynchDevice<I2cDeviceSynch> implements ADS1015 {
    protected GainAmplifier gain = GainAmplifier.GAIN_6_144V;

    protected ADS1015Impl(I2cDeviceSynch i2cDeviceSynch) {
        super(i2cDeviceSynch, true);

        Registers[] rgstrs = Registers.values();
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(rgstrs[0].bVal, rgstrs[rgstrs.length - 1].bVal - rgstrs[0].bVal + 1, I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    public void setGain(GainAmplifier gain) {
        this.gain = gain;
    }

    protected int readADCSingleRaw(int channel) {
        if(channel < 0 || channel > 3) {
            throw new IllegalArgumentException("Channel must be [0,3]!");
        }

        int config =
                ConfigurationFlags.COMPARATOR_QUEUE_DISABLED.flag           |
                ConfigurationFlags.COMPARATOR_LATCHING_DISABLED.flag        |
                ConfigurationFlags.COMPARATOR_POLARITY_LOW.flag             |
                ConfigurationFlags.COMPARATOR_MODE_TRADITIONAL.flag         |
                ConfigurationFlags.DATA_RATE_1600.flag                      |
                ConfigurationFlags.MODE_SINGLE.flag                         |
                gain.flag.flag;

        switch(channel) {
            case(0):
                config |= ConfigurationFlags.INPUT_MULTIPLEXER_SINGLE_0.flag;
                break;
            case(1):
                config |= ConfigurationFlags.INPUT_MULTIPLEXER_SINGLE_1.flag;
                break;
            case(2):
                config |= ConfigurationFlags.INPUT_MULTIPLEXER_SINGLE_2.flag;
                break;
            case(3):
                config |= ConfigurationFlags.INPUT_MULTIPLEXER_SINGLE_3.flag;
                break;
        }

        config |= ConfigurationFlags.OPERATIONAL_STATUS_WRITE_START_SINGLE_CONVERSION.flag;

        writeShort(Registers.CONFIGURATION, (short) config);

        try {
            Thread.sleep(1);
        } catch (InterruptedException ex) {
            Log.e(FtcRobotControllerActivity.TAG, "ADS1015 Sleep Interrupted", ex);
        }

        return readShort(Registers.CONVERSION) >> 4;
    }

    public double getVoltage(int channel) {
        return readADCSingleRaw(channel) / 4096.0 * gain.voltage;
    }

    protected void writeShort(final Registers reg, short value) {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected short readShort(Registers reg) {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    @Override
    protected boolean doInitialize() {
        return false;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Texas Instruments ADS1015 ADC";
    }
}