package org.firstinspires.ftc.teamcode.subsystems.drivebase;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.PID;
import org.firstinspires.ftc.teamcode.subsystems.sensors.IMU;
import org.firstinspires.ftc.teamcode.util.logging.ArrayLogging;

import java.io.IOException;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

/**
 * Created by kskrueger on 12/12/17.
 */

public class VectorDrive {
    private LinearOpMode opMode;
    private DcMotor FLMotor, BLMotor, FRMotor, BRMotor;
    private IMU imu, imu2;
    public PID turnPID = new PID();
    private PID distancePID = new PID();
    public boolean moveOn = false;

    public ArrayLogging logging = new ArrayLogging(10,10000);
    public ElapsedTime timer = new ElapsedTime();

    public VectorDrive(LinearOpMode opMode, IMU imu, IMU imu2,
                       DcMotor FLMotor, CRServo FLServo, AnalogInput FLSensor,
                       DcMotor BLMotor, CRServo BLServo, AnalogInput BLSensor,
                       DcMotor FRMotor, CRServo FRServo, AnalogInput FRSensor,
                       DcMotor BRMotor, CRServo BRServo, AnalogInput BRSensor){
        this.opMode = opMode;
        this.FLMotor = FLMotor;
        this.BLMotor = BLMotor;
        this.FRMotor = FRMotor;
        this.BRMotor = BRMotor;

        this.imu = imu;
        this.imu2 = imu2;

        turnPID.setVariables(.1,0,.08);
    }

    ///////////UNIQUE TO EACH DRIVETYPE///////////
    public void robotCentric(double forward, double strafe, double theta){
        //this code is specific to swerve or mecanum so it's contained in their own classes
    }

    ///////////SHARED SAME FOR EACH DRIVETYPE///////////
    public void fieldCentric(double forwards, double horizontal, double turning){
        //uses the orientation of the robot to offset the joystick values.
        double forwrd = forwards * -1;
        double strafe = horizontal;

        double gyro_radians = getAvgHeading() * Math.PI/180;
        double temp = forwrd * cos(gyro_radians) +
                strafe * sin(gyro_radians);
        strafe = -forwrd * sin(gyro_radians) +
                strafe * cos(gyro_radians);
        forwrd = temp;

        robotCentric(-forwrd,strafe,turning);
    }

    public void gyroTurn(double turnSpeed, int targetAngle, int allowedError) {
        if (targetAngle<0) {
            targetAngle += 360;
        }
        while (Math.abs(getAvgHeading()-targetAngle)>allowedError&&!opMode.isStopRequested()&&opMode.opModeIsActive()) {
            double pidOffset = turnPID.run(targetAngle,(int)getAvgHeading());
            double power = -pidOffset * turnSpeed;
            robotCentric(0, 0, power);
        }
        robotCentric(0,0,0);
    }

    public void gyroDrivePID(double ySpeed, double xSpeed, double heading) {
        turnPID.setVariables(.08,0,.15);
        double offset = turnPID.run((int)heading,(int)getAvgHeading());
        robotCentric(ySpeed,xSpeed,-offset*.2);
    }

    public void turnDrivePID(double ySpeed, double xSpeed, int position, int target, double multiplier) {
        double offset = turnPID.run(target,position);
        robotCentric(ySpeed,xSpeed,-offset*multiplier);
    }

    public void gyroDrive(double ySpeed, double xSpeed, double heading) {
        //turnPID.setVariables(.0007,0,.12);
        double offset = ((getAvgHeading() - heading) / 25);
        robotCentric(ySpeed,xSpeed,offset);
    }

    public double getAvgHeading() {
        imu.setHeadingOffset(0);
        imu2.setHeadingOffset(0);
        return ((imu.getHeading()+360)%360);
        //Calculate the average heading of the 2 IMUs on the robot
        //use this to average IMUS again...
        //return ((((imu.getHeading()+360)%360)+((imu2.getHeading()+360)%360))/2)%360;
    }

    public void zeroEncoders(){
        //save the mode of each motor encoder
        DcMotor.RunMode FLMode = FLMotor.getMode();
        DcMotor.RunMode FRMode = FRMotor.getMode();
        DcMotor.RunMode BLMode = BLMotor.getMode();
        DcMotor.RunMode BRMode = BRMotor.getMode();

        //reset the encoders
        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //restore motors back to original mode
        FLMotor.setMode(FLMode);
        FRMotor.setMode(FRMode);
        BLMotor.setMode(BLMode);
        BRMotor.setMode(BRMode);
    }

    public void setEncoderMode(DcMotor.RunMode mode){
        FLMotor.setMode(mode);
        FRMotor.setMode(mode);
        BLMotor.setMode(mode);
        BRMotor.setMode(mode);
    }

    ///////////SWERVE SPECIFIC MODES///////////
    public void holdModuleAngle(int angle) {
        //swerve overrides
    }

    public void zeroReset(boolean zeroReset){
        //used to zero all the modules when reset is needed
    }

    public void setEfficiency(boolean efficiency) {
        //used to set the efficiency mode to the modules
    }

    public void initializeLogging () {
        //only used in SwerveDrive
    }

    public void robotCentricLog(double forward, double strafe, double theta) throws IOException {
        //overrides in SwerveDrive
    }

    public void log (double forward,double strafe,double theta, double[] ws, double[] wa) {
        //overrides in SwerveDrive
    }

    ///////////MECANUM SPECIFIC MODES///////////
    public int getStrafeEncoderAverage(){
        //overrides in MecanumDrive
        return 0;
    }

    public int getFwdEncoderAverage(){
        //overrides in MecanumDrive
        return 0;
    }

    public void encoderStrafe(double power, int encoder){
        //overrides in MecanumDrive
    }

    public void encoderStrafe(double power, int encoder, int heading) {
        //overrides in MecanumDrive
    }

    public void encoderFwd(double power, int encoder) {
        //overrides in MecanumDrive
    }

    public void encoderFwd(double power, int encoder, int heading) {
        //overrides in MecanumDrive
    }

    public void encoderPidStrafeDistance(double power, int encoder, boolean gyroOn) {
        //overrides in MecanumDrive
    }

    //encoderPidFwdDistance
    public void encoderFwd(double power, int encoder, int heading, int tolerance) {
        distancePID.setTolerance(tolerance);
        distancePID.setVariables(.08,0,.1);
        while (!distancePID.withinTolerance&&!opMode.isStopRequested()&&opMode.opModeIsActive()&&!moveOn) {
            gyroDrivePID(power/6*distancePID.runDistance(encoder,getFwdEncoderAverage()),0,heading);
        }
        robotCentric(0,0,0);
        moveOn = false;
    }

    public void timeDrive(double yPower, double xPower, double turnPower, double time) {
        timer.reset();
        while (timer.seconds()<time && !opMode.isStopRequested() && opMode.opModeIsActive()) {
            robotCentric(yPower,xPower,turnPower);
        }
        robotCentric(0,0,0);
    }

    //enum for driveType, this is used when selecting Mecanum or Swerve drivebase
    public enum driveType {
        MECANUM,
        SWERVE
    }
}