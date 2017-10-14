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

    //general variables
    public double powerOut;
    public double maxVolt = 2.06;
    public int reverse;

    public void set(double angle, double speed, DcMotor motor, Servo servo, AnalogInput encoder, double zeroPosition) {
        setAngle(servo,angle,zeroPosition,encoder);
        double direction = reverse;
        setVelocity(motor,direction*speed);
    }

    public void setAngle(Servo servo, double targetAngle, double zeroPosition, AnalogInput encoder) {
        servo.setPosition(swivelPID(angle(encoder,zeroPosition),((int)targetAngle)));
    }

    public void setVelocity(DcMotor motor, double speed) {
        motor.setPower(speed);
    }

    public int angle(AnalogInput encoder, double zeroPosition) {
        double angle = ((encoder.getVoltage()-zeroPosition)/maxVolt)*360;
        if (angle<0) {
            angle = 360+angle;
        }

        return (int)angle;
    }

    public int reverse180(int target, int position) {
        int target180 = (target+180)%360;
        int error;

        double angError = (target-position)-(360*Math.floor(0.5+(((target-position)+0d)/360.0)));
        double angError180 = (target180-position)-(360*Math.floor(0.5+(((target180-position)+0d)/360.0)));

        if (Math.abs(angError)>Math.abs(angError180)) {
            error = (int)angError;
            reverse = 1;
        } else {
            error = (int)angError180;
            reverse = -1;
        }

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

        PIDpower = -1*u;

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
