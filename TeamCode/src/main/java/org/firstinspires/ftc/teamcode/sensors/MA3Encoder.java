package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by kskrueger on 11/4/17.
 */

public class MA3Encoder{
    private final AnalogInput encoder;
    private double zeroVoltage, maxVolt = 2.06;
    private double startTime = System.currentTimeMillis();

    public MA3Encoder(HardwareMap hwMap, String configName) {
        encoder = hwMap.analogInput.get(configName);
    }

    public void setZeroVoltage(double zeroVoltage){
        this.zeroVoltage = zeroVoltage;
    }

    public void setMaxVoltage(double maxVoltage){
        this.maxVolt = maxVoltage;
    }

    public double getVoltage(){
        return encoder.getVoltage();
    }

    public double pos1, pos2, time1, time2, velocity, diff;
    public double getVelocity(){
        pos1 = getIncremental();
        time1 = System.currentTimeMillis();
        diff = (time1-time2);
        if (diff<50) {
            //nothing
        } else {
            velocity = (pos1-pos2)/(diff/1000);
        }
        pos2 = pos1;
        time2 = time1;

        return velocity;
    }

    public double getAbsolute(){
        double angle = ((encoder.getVoltage()-zeroVoltage)/ maxVolt)*360;
        if (angle<0) {
            angle = 360+angle;
        }
        return (int)angle%360;
    }

    private double prev = 0, incremental;
    public double getIncremental() {
        double curr = getAbsolute();
        if ((360 - prev) < 10 && curr < 10) {
            incremental += (360 - prev) + curr;
        } else if ((360 - curr) < 10 && prev < 10) {
            incremental -= (360 - curr) + prev;
        } else {
            incremental += curr - prev;
        }
        prev = curr;
        return incremental;
    }
}