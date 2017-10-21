package org.firstinspires.ftc.teamcode.drivebase;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by kskrueger on 10/10/17.
 */

public class Module {
    //PID Variables
    int integral = 0;
    double u;
    int error;
    int previousError = 0;
    double PIDpower;
    int targetValue;

    //general variables
    public double powerOut;
    public double maxVolt = 2.06;
    public int reverse;

    DcMotor motor;
    Servo servo;
    AnalogInput encoder;
    double zeroPosition;

    double direction;// = reverse;

    public Module(DcMotor motor1, Servo servo1, AnalogInput encoder1, double zeroPosition1) {
        motor = motor1;
        servo = servo1;
        encoder = encoder1;
        zeroPosition = zeroPosition1;
    }

    public void set(int angle, double speed) {
        setAngle(angle);
        direction = reverse;
        /*if (Math.abs(((angle()-(angle+180))%360))<Math.abs(((angle()-(angle))%360))) {
            direction = -1;
        } else {
            direction = 1;
        }*/
        setVelocity(direction*speed);
    }

    public double direction() {
        return direction;
    }

    public void setAngle(int targetAngle) {
        double servoPosition = swivelPID(angle(),(targetAngle));
        servo.setPosition(servoPosition);
    }

    public void setVelocity(double speed) {
        motor.setPower(speed);
    }

    public int angle() {
        double angle = ((encoder.getVoltage()-zeroPosition)/maxVolt)*360;
        if (angle<0) {
            angle = 360+angle;
        }

        return (int)angle;
    }

    public int reverse180(int targetAngle, int position) {
        int angleError;
        int angleErrorOp;
        int target = (int)targetAngle;
        int targetOp = (target+180)%360;
        int measuredAngle = position;

        angleError = (target - measuredAngle);
        angleError -= (360*Math.floor(0.5+((angleError+0d)/360.0)));

        angleErrorOp = (targetOp - measuredAngle);
        angleErrorOp -= (360*Math.floor(0.5+((angleErrorOp+0d)/360.0)));

            if (Math.abs(angleError)>Math.abs(angleErrorOp)) {
                targetValue=targetOp;
                reverse=-1;
            } else {
                targetValue=target;
                reverse=1;
            }

        angleError = (targetValue - measuredAngle);
        angleError -= (360*Math.floor(0.5+((angleError+0d)/360.0)));

        error = angleError;

        return error;
    }

    public double swivelPID (int angle, int targetAngle) {
        final double Kp = .02;
        final double Ki = 0;
        final double Kd = .008;
        int dt = 20;

        error = reverse180(targetAngle,angle);

        integral += Ki * error * dt;

        u = (Kp * error + integral + Kd * (error - previousError) / dt);

        previousError = error;

        PIDpower = -1 * u;

        //convert to power for the servo (from 0-1)
        if (PIDpower>0) {
            powerOut = .5+(PIDpower/2);
        } else if (PIDpower<0) {
            powerOut = .5+(PIDpower/2);
        } else {
            powerOut = PIDpower;
        }

        return powerOut;
    }

}
