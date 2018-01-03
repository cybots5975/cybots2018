package org.firstinspires.ftc.teamcode.subsystems.drivebase.swerve;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Karter Krueger on 10/10/17.
 */

public class Module {
    //PID loop Variables
    private int integral = 0;
    private int error;
    private int previousError = 0;
    double angleError;
    double angleErrorOp;
    int targetOp;

    private double zeroPosition;
    int reverse;
    private boolean zeroReset;
    private boolean efficiency = true;

    private int holdPosition;

    //define the motor, servo, and encoders
    private DcMotor motor;
    private CRServo servo;
    private AnalogInput encoder;

    Module(DcMotor motor, CRServo servo, AnalogInput encoder, double zeroPosition) {
        this.motor = motor;
        this.servo = servo;
        this.encoder = encoder;
        this.zeroPosition = zeroPosition;
    }

    //sets the angle and speed to the module
    void set(double angle, double speed) {
        holdPosition = 0;
        setAngle((int)angle);
        setVelocity(reverse*speed);
    }

    //set the angle to the module (ex: 90° is sideways and 0° is forward)
    private void setAngle(int targetAngle) {
        double servoPower = swivelPID(angle(),(targetAngle));
        servo.setPower(servoPower);
    }

    //set the velocity to the drive motor for the wheel
    private void setVelocity(double speed) {
        motor.setPower(speed);
    }

    //find the current angle of the encoder
    public int angle() {
        double maxVolt = 2.06;
        double angle = ((encoder.getVoltage()-zeroPosition)/ maxVolt)*360;
        if (angle<0) {
            angle = 360+angle;
        }
        return (int)angle;
    }

    void holdAngle(int angle) {
        holdPosition = angle;
        setAngle(holdPosition);
    }

    //convert the angle of the module to radians
    private double angleRadians() {
        return Math.toRadians(angle());
    }

    //zeroReset is used to force the modules to rotate back to the zero starting position
    void zeroReset(boolean zeroReset){
        this.zeroReset = zeroReset;
    }

    //setEfficiency is used to set the efficiency variable of the program
    void setEfficiency(boolean efficiency){
        this.efficiency = efficiency;
    }

    //reverse180 calculates the error (difference) from the current angle to the the target angle...
    //...it also finds the opposite angle (180° offset) to see if it is colser for the module to rotate to
    private int reverse180(int targetAngle, int position) {
        targetOp = (targetAngle +180)%360;

        angleError = (targetAngle - position);
        angleError -= (360*Math.floor(0.5+((angleError)/360.0)));

        angleErrorOp = (targetOp - position);
        angleErrorOp -= (360*Math.floor(0.5+((angleErrorOp)/360.0)));

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

        error = (int)angleError;

        return error;
    }

    //PID (Proportional Integral Derivative) loop is used to take the error from target and...
    //...proportionally calculate what speed it needs `to rotate to reach the target value
    private double swivelPID(int angle, int targetAngle) {
        final double Kp = .02;
        final double Ki = 0;
        final double Kd = .02;
        int dt = 20;

        error = reverse180(targetAngle,angle);

        integral += Ki * error * dt;

        double u = (Kp * error + integral + Kd * (error - previousError) / dt);

        previousError = error;

        //convert to servo power range from (-1) to 1
        double powerOut = u;

        powerOut = Range.clip(powerOut,-.88,.88);

        if (powerOut<.2&&powerOut>.01) {
            powerOut *= 1.75 + .08;
        } else if (powerOut>-.2&&powerOut<-.01) {
            powerOut *= 1.75 - .08;
        }

        return powerOut;
    }

    int getCurrentEncoder(){
        return motor.getCurrentPosition();
    }
}
