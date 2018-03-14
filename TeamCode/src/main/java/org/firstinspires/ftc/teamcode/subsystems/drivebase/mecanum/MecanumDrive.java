package org.firstinspires.ftc.teamcode.subsystems.drivebase.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.drivebase.VectorDrive;
import org.firstinspires.ftc.teamcode.subsystems.sensors.IMU;

import java.util.Arrays;

/**
 * Created by kskrueger on 12/17/17.
 */

public class MecanumDrive extends VectorDrive{
    private LinearOpMode opMode;
    public DcMotor FLMotor, BLMotor, FRMotor, BRMotor;

    public MecanumDrive(LinearOpMode opMode, IMU imu, IMU imu2,
                        DcMotor FLMotor, CRServo FLServo, AnalogInput FLSensor,
                        DcMotor BLMotor, CRServo BLServo, AnalogInput BLSensor,
                        DcMotor FRMotor, CRServo FRServo, AnalogInput FRSensor,
                        DcMotor BRMotor, CRServo BRServo, AnalogInput BRSensor){
        super( opMode, imu, imu2,
                FLMotor, FLServo, FLSensor,
                BLMotor, BLServo, BLSensor,
                FRMotor, FRServo, FRSensor,
                BRMotor, BRServo, BRSensor);

        this.opMode = opMode;
        this.FLMotor = FLMotor;
        this.BLMotor = BLMotor;
        this.FRMotor = FRMotor;
        this.BRMotor = BRMotor;
    }

    @Override
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

    @Override
    public int getStrafeEncoderAverage(){
        double FL = FLMotor.getCurrentPosition();
        double FR = FRMotor.getCurrentPosition();
        double BL = -BLMotor.getCurrentPosition();
        double BR = -BRMotor.getCurrentPosition();

        return (int)(FL/*+FR*/+BL+BR)/3; //was 4
    }

    @Override
    public int getFwdEncoderAverage(){
        double FL = FLMotor.getCurrentPosition();
        double FR = FRMotor.getCurrentPosition();
        double BL = BLMotor.getCurrentPosition();
        double BR = BRMotor.getCurrentPosition();

        return (int)(FL/*+FR*/+BL+BR)/3; //was 4
    }

    @Override
    public void encoderStrafe(double power, int encoder){
        if (encoder>0) {
            while (getStrafeEncoderAverage()<encoder&&!opMode.isStopRequested()&&opMode.opModeIsActive()) {
                robotCentric(0,-power,0);
            }
        } else {
            while (getStrafeEncoderAverage()>encoder&&!opMode.isStopRequested()&&opMode.opModeIsActive()) {
                robotCentric(0,-power,0);
            }
        }
        robotCentric(0,0,0);
    }

    @Override
    public void encoderStrafe(double power, int encoder, int heading) {
        if (encoder>0) {
            while (getStrafeEncoderAverage()<encoder&&!opMode.isStopRequested()&&opMode.opModeIsActive()) {
                gyroDrivePID(0,-power,heading);
            }
        } else {
            while (getStrafeEncoderAverage()>encoder&&!opMode.isStopRequested()&&opMode.opModeIsActive()) {
                gyroDrivePID(0,-power,heading);
            }
        }
        robotCentric(0,0,0);
    }

    @Override
    public void encoderFwd(double power, int encoder) {
        if (encoder>0) {
            while (getFwdEncoderAverage()<encoder&&!opMode.isStopRequested()&&opMode.opModeIsActive()) {
                robotCentric(power,0,0);
            }
            robotCentric(0,0,0);
            return;
        } else {
            while (getFwdEncoderAverage()>encoder&&!opMode.isStopRequested()&&opMode.opModeIsActive()) {
                robotCentric(power,0,0);
            }
            robotCentric(0,0,0);
            return;
        }
    }

    @Override
    public void encoderFwd(double power, int encoder, int heading) {
        if (encoder>0) {
            while (getFwdEncoderAverage()<encoder&&!opMode.isStopRequested()&&opMode.opModeIsActive()&&!moveOn) {
                gyroDrivePID(power,0,heading);
            }
            robotCentric(0,0,0);
            moveOn = false;
        } else {
            while (getFwdEncoderAverage()>encoder&&!opMode.isStopRequested()&&opMode.opModeIsActive()&&!moveOn) {
                gyroDrivePID(power,0,heading);
            }
            robotCentric(0,0,0);
            moveOn = false;
        }
        return;
    }

    @Override
    public void encoderPidStrafeDistance(double power, int encoder, boolean gyroOn) {
        //work in progress
    }
}