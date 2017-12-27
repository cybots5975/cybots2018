package org.firstinspires.ftc.teamcode.drivebase.mecanum.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drivebase.VectorDrive;
import org.firstinspires.ftc.teamcode.general.PID;
import org.firstinspires.ftc.teamcode.logging.ArrayLogging;
import org.firstinspires.ftc.teamcode.sensors.IMU;

import java.io.IOException;
import java.util.Arrays;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

/**
 * Created by kskrueger on 12/17/17.
 */

public class MecanumDrive implements VectorDrive{
    private LinearOpMode opMode;
    public DcMotor FLMotor, BLMotor, FRMotor, BRMotor;
    private IMU imu, imu2;
    private ArrayLogging log = new ArrayLogging(32,10000);
    public PID turnPID = new PID();

    public MecanumDrive(LinearOpMode opMode,
                        DcMotor FLMotor, CRServo FLServo, AnalogInput FLSensor,
                        DcMotor BLMotor, CRServo BLServo, AnalogInput BLSensor,
                        DcMotor FRMotor, CRServo FRServo, AnalogInput FRSensor,
                        DcMotor BRMotor, CRServo BRServo, AnalogInput BRSensor){

        this.opMode = opMode;
        this.FLMotor = FLMotor;
        this.BLMotor = BLMotor;
        this.FRMotor = FRMotor;
        this.BRMotor = BRMotor;
    }

    public enum direction {
        FORWARD, STRAFE, TURN
    }

    public void robotCentric(double forwards, double horizontal, double turning) {
        double leftFront = forwards + horizontal + turning;
        double leftBack = forwards - horizontal + turning;
        double rightFront = forwards - horizontal - turning;
        double rightBack = forwards + horizontal - turning;

        double[] wheelPowers = {Math.abs(rightFront), Math.abs(leftFront), Math.abs(leftBack), Math.abs(rightBack)};
        Arrays.sort(wheelPowers);
        double biggestInput = wheelPowers[3];
        if (biggestInput > 1) {
            leftFront /= biggestInput;
            leftBack /= biggestInput;
            rightFront /= biggestInput;
            rightBack /= biggestInput;
        }

        FLMotor.setPower(leftBack);
        FRMotor.setPower(rightFront);
        BLMotor.setPower(leftFront);
        BRMotor.setPower(rightBack);
    }

    public void fieldCentric(double forwards, double horizontal, double turning) {
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

    public void gyroTurn (double turnSpeed, int targetAngle, int allowedError) {
        turnPID.setVariables(.1,0,.08);
        while (Math.abs(getAvgHeading()-targetAngle)>allowedError&&!opMode.isStopRequested()) {
            double pidOffset = turnPID.run(targetAngle,(int)getAvgHeading());
            double power = -pidOffset * turnSpeed;
            robotCentric(0, 0, power);
        }
        robotCentric(0,0,0);
    }

    public void gyroDrive(double ySpeed, double xSpeed, double heading) {
        double offset = turnPID.run((int)heading,(int)getAvgHeading());
        robotCentric(ySpeed,xSpeed,offset);
    }

    public void mecanumPIDTurn(double speed, double heading, double kP, double kI, double kD) {

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

    public int getStrafeEncoderAverage(){
        double FL = FLMotor.getCurrentPosition();
        double FR = FRMotor.getCurrentPosition();
        double BL = -BLMotor.getCurrentPosition();
        double BR = -BRMotor.getCurrentPosition();

        return (int)(FL+FR+BL+BR)/4;
    }

    public int getFwdEncoderAverage(){
        double FL = FLMotor.getCurrentPosition();
        double FR = FRMotor.getCurrentPosition();
        double BL = BLMotor.getCurrentPosition();
        double BR = BRMotor.getCurrentPosition();

        return (int)(FL+FR+BL+BR)/4;
    }

    public void encoderStrafe(double power, int encoder){
        if (encoder>0) {
            while (getStrafeEncoderAverage()<encoder&&!opMode.isStopRequested()) {
                robotCentric(0,-power,0);
            }
        } else {
            while (getStrafeEncoderAverage()>encoder&&!opMode.isStopRequested()) {
                robotCentric(0,-power,0);
            }
        }
        robotCentric(0,0,0);
    }

    public void encoderFwd(double power, int encoder) {
        if (encoder>0) {
            while (getFwdEncoderAverage()<encoder/*&&!opMode.isStopRequested()*/) {
                robotCentric(power,0,0);
            }
        } else {
            while (getFwdEncoderAverage()>encoder/*&&!opMode.isStopRequested()*/) {
                robotCentric(power,0,0);
            }
        }
        robotCentric(0,0,0);
    }

    public void encoderPidStrafeDistance(double power, int encoder, boolean gyroOn) {

    }

    public double getAvgHeading() {
        //Calculate the average heading of the 2 absolute orientation sensors on the robot
        return ((imu.getHeading()+imu2.getHeading())/2)%360;
    }

    public void holdModuleAngle(int angle) {
    }

    public void zeroReset(boolean zeroReset) {

    }

    public void setEfficiency(boolean efficiency) {

    }

    public void moveEncoder(double ySpeed, double xSpeed, int encoder) {

    }

    public void robotCentricLOG(double strafe, double forward, double theta, boolean zeroReset) throws IOException {

    }

    public void a(boolean value) {

    }

    public void initializeLogging() {

    }

    public void log(double forward, double strafe, double theta, double[] ws, double[] wa) {

    }
}