package org.firstinspires.ftc.teamcode.subsystems.sensors;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by kskrueger for Cybots Robotics on 3/5/18.
 */

public class PositionTracking {
    private double wheelDiameter = 2.445;
    private double degreesPerInch = 360 / (wheelDiameter * Math.PI);
    private boolean trackingThread = false;

    public double xWheelDown = .521;
    public double yWheelDown = .732;
    public double xWheelUp = .67;
    public double yWheelUp = .615;

    private MA3Encoder xWheel, yWheel;
    private IMU gyro;

    private Servo xServo, yServo;

    private double xPosition;
    private double yPosition;
    private double xOffset = 0;
    private double yOffset = 0;

    public PositionTracking (MA3Encoder xWheel, MA3Encoder yWheel, IMU gyro, Servo xServo, Servo yServo) {
        this.xWheel = xWheel;
        this.yWheel = yWheel;
        this.gyro = gyro;
        this.xServo = xServo;
        this.yServo = yServo;
    }

    public void init () {
        xPosition = 0;
        yPosition = 0;
        wheelsUp();
    }

    public void wheelsDown () {
        xServo.setPosition(xWheelDown);
        yServo.setPosition(yWheelDown);
    }

    public void wheelsUp () {
        xServo.setPosition(xWheelUp);
        yServo.setPosition(yWheelUp);
    }

    public void startTracking () {
        trackingThread = true;
        tracking();
    }

    public void zeroEncoders() {
        xOffset = xPosition;
        yOffset = yPosition;
    }

    public void stopTracking () {
        trackingThread = false;
    }

    public double xPosition () {
        return xPosition - xOffset;
    }

    public double xPosition (DistanceUnit unit) {
        return unit.fromInches(xPosition()/degreesPerInch);
    }

    public double yPosition () {
        return yPosition - yOffset;
    }

    public double yPosition (DistanceUnit unit) {
        return unit.fromInches(yPosition()/degreesPerInch);
    }

    private void tracking() {
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                while(trackingThread) {
                    xPosition = xWheel.getIncremental();
                    yPosition = yWheel.getIncremental();
                }
            }
        }).start();
    }


}