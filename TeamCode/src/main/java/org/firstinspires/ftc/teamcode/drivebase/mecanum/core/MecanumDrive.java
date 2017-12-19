package org.firstinspires.ftc.teamcode.drivebase.mecanum.core;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.logging.ArrayLogging;
import org.firstinspires.ftc.teamcode.sensors.IMU;

import java.util.Arrays;

/**
 * Created by kskrueger on 12/17/17.
 */

public class MecanumDrive {
    private double SCALEDPOWER = 1;
    private DcMotor FLMotor, BLMotor, FRMotor, BRMotor;
    private IMU imu, imu2;
    private ArrayLogging log = new ArrayLogging(32,10000);

    public MecanumDrive(IMU imuDS, IMU imuPS,
                       DcMotor FLMotor, CRServo FLServo, AnalogInput FLSensor,
                       DcMotor BLMotor, CRServo BLServo, AnalogInput BLSensor,
                       DcMotor FRMotor, CRServo FRServo, AnalogInput FRSensor,
                       DcMotor BRMotor, CRServo BRServo, AnalogInput BRSensor){


        this.FLMotor = FLMotor;
        this.BLMotor = BLMotor;
        this.FRMotor = FRMotor;
        this.BRMotor = BRMotor;

        this.imu = imuDS;
        this.imu2 = imuPS;
    }

    // y - forwards
    // x - side
    // c - rotation
    public void arcadeMecanum(double y, double x, double c) {
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
        double scaledPower = SCALEDPOWER;

        FLMotor.setPower(leftFrontVal*scaledPower+FLMotor.getPower()*(1-scaledPower));
        FRMotor.setPower(rightFrontVal*scaledPower+FRMotor.getPower()*(1-scaledPower));
        BLMotor.setPower(leftBackVal*scaledPower+BLMotor.getPower()*(1-scaledPower));
        BRMotor.setPower(rightBackVal*scaledPower+BRMotor.getPower()*(1-scaledPower));
    }
}
