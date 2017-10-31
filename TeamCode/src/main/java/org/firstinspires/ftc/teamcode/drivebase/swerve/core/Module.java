package org.firstinspires.ftc.teamcode.drivebase.swerve.core;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Karter Krueger on 10/10/17.
 */

public class Module {
    //PID loop Variables
    private int integral = 0;
    private int error;
    private int previousError = 0;

    private double zeroPosition;
    private int reverse;
    private boolean zeroReset;
    private boolean efficiency = true;

    //define the motor, servo, and encoders
    private DcMotor motor;
    private Servo servo;
    private AnalogInput encoder;

    Module(DcMotor motor, Servo servo, AnalogInput encoder, double zeroPosition) {
        this.motor = motor;
        this.servo = servo;
        this.encoder = encoder;
        this.zeroPosition = zeroPosition;
    }

    //sets the angle and speed
    void set(double angle, double speed) {
        setAngle((int)angle);
        setVelocity(reverse*speed);
    }

    private void setAngle(int targetAngle) {
        double servoPosition = swivelPID(angle(),(targetAngle));
        servo.setPosition(servoPosition);
    }

    private void setVelocity(double speed) {
        motor.setPower(speed);
    }

    private int angle() {
        double maxVolt = 2.06;
        double angle = ((encoder.getVoltage()-zeroPosition)/ maxVolt)*360;
        if (angle<0) {
            angle = 360+angle;
        }
        return (int)angle;
    }

    private double angleRadians() {
        return Math.toRadians(angle());
    }

    void zeroReset(boolean zeroReset){
        this.zeroReset = zeroReset;
    }

    void setEfficiency(boolean efficiency){
        this.efficiency = efficiency;
    }

    //reverse180 calculates the error (difference) from the current angle to the the target angle...
    //...it also finds the opposite angle (180° offset) to see if it is colser for the module to rotate to
    private int reverse180(int targetAngle, int position) {
        int angleError;
        int angleErrorOp;
        int targetOp = (targetAngle +180)%360;

        angleError = (targetAngle - position);
        angleError -= (360*Math.floor(0.5+((angleError+0d)/360.0)));

        angleErrorOp = (targetOp - position);
        angleErrorOp -= (360*Math.floor(0.5+((angleErrorOp+0d)/360.0)));

        int targetValue;
        if ((Math.abs(angleError)>Math.abs(angleErrorOp))&&efficiency) {
                targetValue =targetOp;
                reverse=-1;
            } else {
                targetValue = targetAngle;
                reverse=1;
            }

        if (zeroReset) {
            targetValue = 0;
        }

        angleError = (targetValue - position);
        angleError -= (360*Math.floor(0.5+((angleError+0d)/360.0)));

        error = angleError;

        return error;
    }

    //PID (Proportional Integral Derivative) loop is used to take the error from target and...
    //...proportionally calculate what speed it needs to rotate to reach the target value
    private double swivelPID (int angle, int targetAngle) {
        final double Kp = .03;
        final double Ki = 0;
        final double Kd = .01;
        int dt = 20;

        error = reverse180(targetAngle,angle);

        integral += Ki * error * dt;

        double u = (Kp * error + integral + Kd * (error - previousError) / dt);

        previousError = error;

        double PIDpower = -1 * u;

        //convert to servo power range from 0-1
        double powerOut;
        if (PIDpower >0) {
            powerOut = .5+(PIDpower /2);
        } else if (PIDpower <0) {
            powerOut = .5+(PIDpower /2);
        } else {
            powerOut = PIDpower;
        }
        return powerOut;
    }

    //FROM BACK TO THE DRAWING BOARD'S CODE STARTING HERE
    int targetAngle = 0;



    public double getDelta(){
        Vector targetVector = new Vector(Math.cos(targetAngle), Math.sin(targetAngle));
        Vector currentVector = new Vector(Math.cos(angleRadians()), Math.sin(angleRadians()));
        //angleBetween is the angle from currentPosition to target position in radians
        //it has a range of -pi to pi, with negative values being clockwise and positive counterclockwise of the current angle
        double angleBetween = Math.atan2(currentVector.x * targetVector.y - currentVector.y * targetVector.x,
                currentVector.x * targetVector.x + currentVector.y * targetVector.y);
        return angleBetween;
    }

    private double wrapAngle(double angle) {
        angle %= 2 * Math.PI;
        if (angle < 0) angle += 2 * Math.PI;


        return angle;
    }

    public void update() {
        Vector targetVector = new Vector(Math.cos(targetAngle), Math.sin(targetAngle));
        Vector currentVector = new Vector(Math.cos(angleRadians()), Math.sin(angleRadians()));

        //angleBetween is the angle from currentPosition to target position in radians
        //it has a range of -pi to pi, with negative values being clockwise and positive counterclockwise of the current angle
        double angleBetween = Math.atan2(currentVector.x * targetVector.y - currentVector.y * targetVector.x, currentVector.x * targetVector.x + currentVector.y * targetVector.y);

        //give the servo a piecewise scaling function: full speed until about 5 degrees away, then linearly slower
        if (angleBetween > Math.toRadians(50)){
            servo.setPosition(0);
        } else if (angleBetween < -Math.toRadians(50)) {
            servo.setPosition(1);
        }else{
            double scaleFactor = Math.abs(angleBetween) / Math.toRadians(50);
            if(angleBetween>0){
                servo.setPosition(.5 - .5 * Math.abs(scaleFactor));
            }else if(angleBetween<0){
                servo.setPosition(.5 + .5 * Math.abs(scaleFactor));

            }
        }
        //set power if close enough
    }


}
