package org.firstinspires.ftc.teamcode.drivebase;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.io.IOException;

/**
 * Created by kskrueger for Cybots Robotics on 12/26/17.
 */

public interface VectorDrive {

    public void robotCentric(double forward, double strafe, double theta);

    //uses the orientation of the robot to offset the joystick values.
    public void fieldCentric(double forwards, double horizontal, double turning);

    public void gyroTurn(double turnSpeed, int targetAngle, int allowedError);

    //gyroMove uses the value of the current robot heading from the orientation sensor to keep the robot going straight
    public void gyroDrive(double ySpeed, double xSpeed, double heading);

    public void zeroEncoders();

    public void setEncoderMode(DcMotor.RunMode mode);

    public int getStrafeEncoderAverage();

    public int getFwdEncoderAverage();

    public void encoderStrafe(double power, int encoder);

    public void encoderFwd(double power, int encoder);

    public void encoderPidStrafeDistance(double power, int encoder, boolean gyroOn);

    //Calculate the average heading of the 2 absolute orientation sensors on the robot
    public double getAvgHeading();

    public void holdModuleAngle(int angle);

    //used to zero all the modules when reset is needed
    public void zeroReset(boolean zeroReset);

    //used to set the efficiency mode to the modules
    public void setEfficiency(boolean efficiency);

    //moveEncoder finds the average encoder positions of all 4 wheel modules and drives until the target value is reached
    public void moveEncoder(double ySpeed, double xSpeed, int encoder);

    public void robotCentricLOG(double strafe, double forward, double theta, boolean zeroReset) throws IOException;

    public void a(boolean value);

    public void initializeLogging();

    public void log(double forward,double strafe,double theta, double[] ws, double[] wa);
}
