package org.firstinspires.ftc.teamcode.subsystems.sensors;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cSensor(name = "MaxSonar I2CXL", description = "MaxSonar I2CXL Sensor from Maxbotix", xmlTag = "MaxSonarI2CXL")
public class I2CXL extends I2cDeviceSynchDevice<I2cDeviceSynch> implements Runnable
{
    private Thread thread;
    private boolean autoPing;
    private int autoPingDelay;
    private long lastPingTime = Long.MAX_VALUE;
    private int lastDistance = Integer.MAX_VALUE;
    private long minDelay = 100;

    public void setAutoPingDelay(int delay)
    {
        autoPingDelay = delay;
    }

    @Override
    public Manufacturer getManufacturer ()
    {
        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize ()
    {
        return true;
    }

    @Override
    public String getDeviceName ()
    {
        return "MaxSonar I2CXL";
    }

    public I2CXL (I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(I2cAddr.create8bit(0xE0));

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    public void setI2cAddress (I2cAddr i2cAddr)
    {
        deviceClient.setI2cAddress(i2cAddr);
    }

    public void ping ()
    {
        lastPingTime = System.currentTimeMillis();
        deviceClient.write8(0, 0x51, I2cWaitControl.ATOMIC);
    }

    public void setMinDelay (int minDelay)
    {
        this.minDelay = minDelay;
    }

    public void startAutoPing (int delay)
    {
        if (thread == null)
        {
            thread = new Thread(this);
        }

        autoPingDelay = delay;
        autoPing = true;

        thread.start();
    }

    public void stopAutoPing ()
    {
        autoPing = false;
        thread = null;
    }

    public int getDistance ()
    {
        long currentTimeMillis = System.currentTimeMillis();
        int distance;

        if (currentTimeMillis - lastPingTime < minDelay)
        {
            distance = lastDistance;
        }
        else
        {
            distance = TypeConversion.byteArrayToShort(deviceClient.read(0x01, 2));
            lastDistance = distance;
        }

        return distance;
    }

    @Override
    public void run ()
    {
        while (autoPing)
        {
            ping();
            try
            {
                Thread.sleep(autoPingDelay);
            }
            catch (InterruptedException e)
            {
            }
        }
    }
}
