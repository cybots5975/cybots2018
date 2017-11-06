package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by kskrueger on 11/4/17.
 */

public class MA3Encoder{
    private final AnalogInput encoder;
    private double zeroVoltage;
    private int rotations = 0;
    public boolean previousBelow0 = false, currentBelow0 = false;
    private double startTime = System.currentTimeMillis();

    public MA3Encoder(HardwareMap hwMap, String name) {
        encoder = hwMap.analogInput.get(name);
    }

    public void setZeroVoltage(double zeroVoltage){
        this.zeroVoltage = zeroVoltage;
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
        double maxVolt = 2.06;
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

/*
if ((getAbsolute()<(360-getAbsolute()))&&getAbsolute()<90) {
            currentBelow0 = true;
        } else if ((getAbsolute()>(360-getAbsolute()))&&(360-getAbsolute())<90) {
            currentBelow0 = false;
        }

        if (!currentBelow0&&previousBelow0) {
            rotations -= 1;
        }

        if (currentBelow0&&!previousBelow0) {
            rotations += 1;
        }

        if ((getAbsolute()<(360-getAbsolute()))&&getAbsolute()<90) {
            previousBelow0 = true;
        } else if ((getAbsolute()>(360-getAbsolute()))&&(360-getAbsolute())<90) {
            previousBelow0 = false;
        }




                currentReading = getAbsolute();

        difference = currentReading-lastReading;

        incremental += difference;
*/