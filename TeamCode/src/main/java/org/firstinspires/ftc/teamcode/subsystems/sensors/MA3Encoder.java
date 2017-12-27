package org.firstinspires.ftc.teamcode.subsystems.sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by kskrueger on 11/4/17.
 */

public class MA3Encoder{
    private final AnalogInput encoder; //define the Analog Input device
    private double zeroVoltage = 0, maxVolt = 2.06; //variables used for getAbsolute
    public double deltaTime, lastTime, pos1, pos2, deltaPos, velocity; //variables used for getVelocity
    private double deltaTime2, lastTime2, vel1, vel2, deltaVel, acceleration;
    private double prev = 0, incremental; //variables for the getIncremental

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

    public double getAbsolute(){
        double angle = ((encoder.getVoltage()-zeroVoltage)/ maxVolt)*360;
        if (angle<0) {
            angle = 360+angle;
        }
        return (int)angle%360;
    }

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

    public double getVelocity(){
        pos1 = getIncremental();
        deltaTime = (System.currentTimeMillis() - lastTime);
        deltaPos = pos1-pos2;

        velocity = (deltaPos)/(deltaTime)*1000;

        pos2 = getIncremental();
        lastTime = System.currentTimeMillis();

        return velocity;
    }

    public double getAcceleration(){
        vel1 = getVelocity();
        deltaTime2 = (System.currentTimeMillis() - lastTime);
        deltaVel = vel1-vel2;

        acceleration = (deltaVel)/(deltaTime2)*1000;

        vel2 = getVelocity();
        lastTime2 = System.currentTimeMillis();

        return acceleration;
    }
}