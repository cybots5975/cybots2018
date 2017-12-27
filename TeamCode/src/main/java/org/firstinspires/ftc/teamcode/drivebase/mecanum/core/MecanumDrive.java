package org.firstinspires.ftc.teamcode.drivebase.mecanum.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drivebase.VectorDrive;
import org.firstinspires.ftc.teamcode.drivebase.swerve.core.SwerveDrive;
import org.firstinspires.ftc.teamcode.general.PID;
import org.firstinspires.ftc.teamcode.logging.ArrayLogging;
import org.firstinspires.ftc.teamcode.sensors.IMU;

import java.util.Arrays;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

/**
 * Created by kskrueger on 12/17/17.
 */

public class MecanumDrive extends VectorDrive{
    private LinearOpMode opMode;
    private DcMotor FLMotor, BLMotor, FRMotor, BRMotor;
    private IMU imu, imu2;
    private ArrayLogging log = new ArrayLogging(32,10000);
    private PID turnPID = new PID();

    public MecanumDrive(LinearOpMode opMode,
                        DcMotor FLMotor,
                        DcMotor BLMotor,
                        DcMotor FRMotor,
                        DcMotor BRMotor){
        /*super(opMode,
                FLMotor,
                BLMotor,
                FRMotor,
                BRMotor);*/

        this.opMode = opMode;
        this.FLMotor = FLMotor;
        this.BLMotor = BLMotor;
        this.FRMotor = FRMotor;
        this.BRMotor = BRMotor;
    }

    public void arcadeMecanum(double y, double x, double c) {
        // y - forwards
        // x - side
        // c - rotation
        double leftFrontVal = y + x + c;
        double rightFrontVal = y - x - c;
        double leftBackVal = y - x + c;
        double rightBackVal = y + x - c;

        //Move range to between 0 and +1, if not already
        double[] wheelPowers = {rightFrontVal, leftFrontVal, leftBackVal, rightBackVal};
        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            leftFrontVal /= wheelPowers[3];
            rightFrontVal /= wheelPowers[3];
            leftBackVal /= wheelPowers[3];
            rightBackVal /= wheelPowers[3];
        }
        double scaledPower = 1;

        FLMotor.setPower(leftFrontVal*scaledPower+FLMotor.getPower()*(1-scaledPower));
        FRMotor.setPower(rightFrontVal*scaledPower+FRMotor.getPower()*(1-scaledPower));
        BLMotor.setPower(leftBackVal*scaledPower+BLMotor.getPower()*(1-scaledPower));
        BRMotor.setPower(rightBackVal*scaledPower+BRMotor.getPower()*(1-scaledPower));
    }

    public enum direction {
        FORWARD, STRAFE, TURN
    }

    public void driveMecanum(double forwards, double horizontal, double turning) {
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

    public void driveMecanumField(double forwards, double horizontal, double turning) {
        double forwrd = forwards * -1;
        double strafe = horizontal;

        double gyro_radians = getAvgHeading() * Math.PI/180;
        double temp = forwrd * cos(gyro_radians) +
                strafe * sin(gyro_radians);
        strafe = -forwrd * sin(gyro_radians) +
                strafe * cos(gyro_radians);
        forwrd = temp;

        driveMecanum(-forwrd,strafe,turning);
    }

    public void gyroTurn (double turnSpeed, int targetAngle, int allowedError) {
        turnPID.setVariables(.1,0,.08);
        while (Math.abs(getAvgHeading()-targetAngle)>allowedError&&!opMode.isStopRequested()) {
            double pidOffset = turnPID.run(targetAngle,(int)getAvgHeading());
            double power = -pidOffset * turnSpeed;
            driveMecanum(0, 0, power);
        }
        driveMecanum(0,0,0);
    }

    public void mecanumGyroCorrect(double ySpeed, double xSpeed, double heading, double offsetMult/*, double kP, double kI, double kD*/) {
        //double dt = (System.currentTimeMillis() - lastTime);

        //double offset = PID(kP,kI,kD,20,(int)heading,(int)robot.getAvgHeading());
        double offset = turnPID.run((int)heading,(int)getAvgHeading());
        driveMecanum(ySpeed,xSpeed,offset/offsetMult);
        //lastTime = System.currentTimeMillis();
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

    private int getStrafeEncoderAverage(){
        double FL = FLMotor.getCurrentPosition();
        double FR = FRMotor.getCurrentPosition();
        double BL = -BLMotor.getCurrentPosition();
        double BR = -BRMotor.getCurrentPosition();

        return (int)(FL+FR+BL+BR)/4;
    }

    private int getFwdEncoderAverage(){
        double FL = FLMotor.getCurrentPosition();
        double FR = FRMotor.getCurrentPosition();
        double BL = BLMotor.getCurrentPosition();
        double BR = BRMotor.getCurrentPosition();

        return (int)(FL+FR+BL+BR)/4;
    }

    public int getEncoderCounts(SwerveDrive.direction direction) {
        if (direction.equals(SwerveDrive.direction.STRAFE)) {
            return getStrafeEncoderAverage();
        } else if (direction.equals(SwerveDrive.direction.FORWARD)) {
            return getFwdEncoderAverage();
        } else {
            return getFwdEncoderAverage();
        }
    }

    public void encoderStrafe(double power, int encoder){
        if (encoder>0) {
            while (getStrafeEncoderAverage()<encoder&&!opMode.isStopRequested()) {
                driveMecanum(0,-power,0);
            }
        } else {
            while (getStrafeEncoderAverage()>encoder&&!opMode.isStopRequested()) {
                driveMecanum(0,-power,0);
            }
        }
        driveMecanum(0,0,0);
    }

    public void encoderFwd(double power, int encoder) {
        if (encoder>0) {
            while (getFwdEncoderAverage()<encoder/*&&!opMode.isStopRequested()*/) {
                driveMecanum(power,0,0);
            }
        } else {
            while (getFwdEncoderAverage()>encoder/*&&!opMode.isStopRequested()*/) {
                driveMecanum(power,0,0);
            }
        }
        driveMecanum(0,0,0);
    }

    public void encoderPidStrafeDistance(double power, int encoder, boolean gyroOn) {

    }

    private double getAvgHeading() {
        //Calculate the average heading of the 2 absolute orientation sensors on the robot
        return ((imu.getHeading()+imu2.getHeading())/2)%360;
    }
}