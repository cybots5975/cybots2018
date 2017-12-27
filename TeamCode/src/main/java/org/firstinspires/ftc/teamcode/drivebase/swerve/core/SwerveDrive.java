package org.firstinspires.ftc.teamcode.drivebase.swerve.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drivebase.VectorDrive;
import org.firstinspires.ftc.teamcode.general.Constants;
import org.firstinspires.ftc.teamcode.logging.ArrayLogging;
import org.firstinspires.ftc.teamcode.sensors.IMU;

import java.io.IOException;

/**
 * Created by Karter Krueger on 10/10/17.
 */

public class SwerveDrive extends VectorDrive{
    private Module FL, BL, FR, BR;
    private IMU imu, imu2;
    private ArrayLogging log = new ArrayLogging(32,10000);
    private int count;
    private boolean done = false;
    private LinearOpMode opMode;
    boolean doa;

    public SwerveDrive(LinearOpMode opMode, IMU imu, IMU imu2,
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
        this.imu = imu;
        this.imu2 = imu2;

        //define the 4 swerve drive modules with motor,servo,encoder sensor,and starting voltage
        FL = new Module(FLMotor,FLServo,FLSensor, Constants.FL_OFFSET); //driver side module 1
        BL = new Module(BLMotor,BLServo,BLSensor,Constants.BL_OFFSET); //driver side module 2
        FR = new Module(FRMotor,FRServo,FRSensor,Constants.FR_OFFSET); //passenger side module 1
        BR = new Module(BRMotor,BRServo,BRSensor, Constants.BR_OFFSET); //passenger side module 2

        initializeLogging();
    }

    @Override
    public void robotCentric(double forward, double strafe, double theta) {
        //RobotCentric is one method of driving the swerve drive robot
        //This is the primary function for doing all the math for the module speeds and angles as well
        final double length = 13.25; //length between axles
        final double width = 15.5; //width between axles

        double radius = Math.hypot(length,width); //find the radius between of the drivebase

        //calculate the 4 variables to be used for calculating the module wheel speeds and angles
        double a = strafe - theta * (length / radius);
        double b = strafe + theta * (length / radius);
        double c = forward - theta * (width / radius);
        double d = forward + theta * (width / radius);

        //store wheel speeds to an array for each speed
        double[] ws = new double[4];
        ws[0] = Math.hypot(a, d);
        ws[1] = Math.hypot(a, c);
        ws[2] = Math.hypot(b, d);
        ws[3] = Math.hypot(b, c);

        //store wheel module angles to an array for each angle
        double[] wa = new double[4];
        wa[0] = Math.atan2(a, d) / Math.PI * 180 - 180;
        wa[1] = Math.atan2(a, c) / Math.PI * 180 - 180;
        wa[2] = Math.atan2(b, d) / Math.PI * 180 + 180;
        wa[3] = Math.atan2(b, c) / Math.PI * 180 + 180;

        //find the maximum speed value over 1 and scale all the other speeds down to to be under 1
        final double maxWheelSpeed = Math.max(Math.max(ws[0], ws[1]), Math.max(ws[2], ws[3]));
        if (maxWheelSpeed > 1.0) {
            for (int i = 0; i < 4; i++) {
                ws[i] /= maxWheelSpeed;
            }
        }

        //set the angle and speed to all 4 modules
        FL.set(wa[3],ws[3]);
        FR.set(wa[2],ws[2]);
        BL.set(wa[1],ws[1]);
        BR.set(wa[0],ws[0]);
    }

    @Override
    public void holdModuleAngle(int angle) {
        FL.holdAngle(angle);
        BL.holdAngle(angle);
        FR.holdAngle(angle);
        BR.holdAngle(angle);
    }

    @Override
    public void zeroReset(boolean zeroReset){
        //used to zero all the modules when reset is needed
        FL.zeroReset(zeroReset);
        BL.zeroReset(zeroReset);
        FR.zeroReset(zeroReset);
        BR.zeroReset(zeroReset);
    }

    @Override
    public void setEfficiency(boolean efficiency) {
        //used to set the efficiency mode to the modules
        FL.setEfficiency(efficiency);
        BL.setEfficiency(efficiency);
        FR.setEfficiency(efficiency);
        BR.setEfficiency(efficiency);
    }

    @Override
    public void robotCentricLog(double strafe, double forward, double theta) throws IOException {
        final double length = 13.25; //length between axles
        final double width = 15.5; //width between axles

        double radius = Math.hypot(length,width); //find the radius between of the drivebase

        //calculate the 4 variables to be used for calculating the module wheel speeds and angles
        double a = strafe - theta * (length / radius);
        double b = strafe + theta * (length / radius);
        double c = forward - theta * (width / radius);
        double d = forward + theta * (width / radius);

        //store wheel speeds to an array for each speed
        double[] ws = new double[4];
        ws[0] = Math.hypot(a, d);
        ws[1] = Math.hypot(a, c);
        ws[2] = Math.hypot(b, d);
        ws[3] = Math.hypot(b, c);

        //store wheel module angles to an array for each angle
        double[] wa = new double[4];
        wa[0] = Math.atan2(a, d) / Math.PI * 180 - 180;
        wa[1] = Math.atan2(a, c) / Math.PI * 180 - 180;
        wa[2] = Math.atan2(b, d) / Math.PI * 180 + 180;
        wa[3] = Math.atan2(b, c) / Math.PI * 180 + 180;

        //find the maximum speed value over 1 and scale all the other speeds down to to be under 1
        final double maxWheelSpeed = Math.max(Math.max(ws[0], ws[1]), Math.max(ws[2], ws[3]));
        if (maxWheelSpeed > 1.0) {
            for (int i = 0; i < 4; i++) {
                ws[i] /= maxWheelSpeed;
            }
        }

        //set the angle and speed to all 4 modules
        BR.set(wa[0],ws[0]);
        BL.set(wa[1],ws[1]);
        FR.set(wa[2],ws[2]);
        FL.set(wa[3],ws[3]);

        log(forward,strafe,theta,ws,wa);

        if (doa&&!done) {
            log.log();
            done = true;
        }
    }

    @Override
    public void initializeLogging () {
        log.storeValue(0, 0, "Count #");
        log.storeValue(1, 0, "Y JoyStick");
        log.storeValue(2, 0, "X JoyStick");
        log.storeValue(3, 0, "Turn JoyStick");

        log.storeValue(4, 0, "D1 Math Speed");
        log.storeValue(5, 0, "D2 Math Speed");
        log.storeValue(6, 0, "P1 Math Speed");
        log.storeValue(7, 0, "P2 Math Speed");

        log.storeValue(8, 0, "D1 Math Angle");
        log.storeValue(9, 0, "D2 Math Angle");
        log.storeValue(10, 0, "P1 Math Angle");
        log.storeValue(11, 0, "P2 Math Angle");

        log.storeValue(12, 0, "D1 Angle");
        log.storeValue(13, 0, "D1 Reverse");
        log.storeValue(14, 0, "D1 angleError");
        log.storeValue(15, 0, "D1 angleErrorOp");
        log.storeValue(16, 0, "D1 targetOp");

        log.storeValue(17, 0, "D2 Angle");
        log.storeValue(18, 0, "D2 Reverse");
        log.storeValue(19, 0, "D2 angleError");
        log.storeValue(20, 0, "D2 angleErrorOp");
        log.storeValue(21, 0, "D2 targetOp");

        log.storeValue(22, 0, "P1 Angle");
        log.storeValue(23, 0, "P1 Reverse");
        log.storeValue(24, 0, "P1 angleError");
        log.storeValue(25, 0, "P1 angleErrorOp");
        log.storeValue(26, 0, "P1 targetOp");

        log.storeValue(27, 0, "P2 Angle");
        log.storeValue(28, 0, "P2 Reverse");
        log.storeValue(29, 0, "P2 angleError");
        log.storeValue(30, 0, "P2 angleErrorOp");
        log.storeValue(31, 0, "P2 targetOp");
    }

    @Override
    public void log (double forward,double strafe,double theta, double[] ws, double[] wa) {
        count += 1;

        log.storeValueInt(0,count,count);
        log.storeValueInt(1,count,forward);
        log.storeValueInt(2,count,strafe);
        log.storeValueInt(3,count,theta);

        log.storeValueInt(4,count,ws[3]);
        log.storeValueInt(5,count,ws[2]);
        log.storeValueInt(6,count,ws[1]);
        log.storeValueInt(7,count,ws[0]);

        log.storeValueInt(8,count,wa[3]);
        log.storeValueInt(9,count,wa[2]);
        log.storeValueInt(10,count,wa[1]);
        log.storeValueInt(11,count,wa[0]);

        log.storeValueInt(12,count,FL.angle());
        log.storeValueInt(13,count,FL.reverse);
        log.storeValueInt(14,count,FL.angleError);
        log.storeValueInt(15,count,FL.angleErrorOp);
        log.storeValueInt(16,count,FL.targetOp);

        log.storeValueInt(17,count,BL.angle());
        log.storeValueInt(18,count,BL.reverse);
        log.storeValueInt(19,count,BL.angleError);
        log.storeValueInt(20,count,BL.angleErrorOp);
        log.storeValueInt(21,count,BL.targetOp);

        log.storeValueInt(22,count,FR.angle());
        log.storeValueInt(23,count,FR.reverse);
        log.storeValueInt(24,count,FR.angleError);
        log.storeValueInt(25,count,FR.angleErrorOp);
        log.storeValueInt(26,count,FR.targetOp);

        log.storeValueInt(27,count,BR.angle());
        log.storeValueInt(28,count,BR.reverse);
        log.storeValueInt(29,count,BR.angleError);
        log.storeValueInt(30,count,BR.angleErrorOp);
        log.storeValueInt(31,count,BR.targetOp);
    }
}