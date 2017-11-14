package org.firstinspires.ftc.teamcode.drivebase.swerve.core;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.general.Constants;
import org.firstinspires.ftc.teamcode.sensors.IMU;

import org.firstinspires.ftc.teamcode.logging.ArrayLogging;

import java.io.IOException;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

/**
 * Created by Karter Krueger on 10/10/17.
 */

public class SwerveDrive {
    private Module D1, D2, P1, P2;
    private DcMotor FLMotor, BLMotor, FRMotor, BRMotor;
    private IMU imu, imu2;
    private ArrayLogging log = new ArrayLogging(32,10000);
    public int count;
    public boolean done = false;
    boolean doa = false;

    public SwerveDrive(IMU imuDS, IMU imuPS,
                       DcMotor FLMotor, Servo FLServo, AnalogInput FLSensor,
                       DcMotor BLMotor, Servo BLServo, AnalogInput BLSensor,
                       DcMotor FRMotor, Servo FRServo, AnalogInput FRSensor,
                       DcMotor BRMotor, Servo BRServo, AnalogInput BRSensor){


        this.FLMotor = FLMotor;
        this.BLMotor = BLMotor;
        this.FRMotor = FRMotor;
        this.BRMotor = BRMotor;

        this.imu = imuDS;
        this.imu2 = imuPS;

        //define the 4 swerve drive modules with motor,servo,encoder sensor,and starting voltage
        D1 = new Module(FLMotor,FLServo,FLSensor, Constants.FL_OFFSET); //driver side module 1  ----Constants.FL_OFFSET
        D2 = new Module(BLMotor,BLServo,BLSensor,Constants.BL_OFFSET); //driver side module 2
        P1 = new Module(FRMotor,FRServo,FRSensor,Constants.FR_OFFSET); //passenger side module 1
        P2 = new Module(BRMotor,BRServo,BRSensor, Constants.BR_OFFSET); //passenger side module 2

        previousError = 0;

        log.storeValue(0,0,"Count #");
        log.storeValue(1,0,"Y JoyStick");
        log.storeValue(2,0,"X JoyStick");
        log.storeValue(3,0,"Turn JoyStick");

        log.storeValue(4,0,"D1 Math Speed");
        log.storeValue(5,0,"D2 Math Speed");
        log.storeValue(6,0,"P1 Math Speed");
        log.storeValue(7,0,"P2 Math Speed");

        log.storeValue(8,0,"D1 Math Angle");
        log.storeValue(9,0,"D2 Math Angle");
        log.storeValue(10,0,"P1 Math Angle");
        log.storeValue(11,0,"P2 Math Angle");

        log.storeValue(12,0,"D1 Angle");
        log.storeValue(13,0,"D1 Reverse");
        log.storeValue(14,0,"D1 angleError");
        log.storeValue(15,0,"D1 angleErrorOp");
        log.storeValue(16,0,"D1 targetOp");

        log.storeValue(17,0,"D2 Angle");
        log.storeValue(18,0,"D2 Reverse");
        log.storeValue(19,0,"D2 angleError");
        log.storeValue(20,0,"D2 angleErrorOp");
        log.storeValue(21,0,"D2 targetOp");

        log.storeValue(22,0,"P1 Angle");
        log.storeValue(23,0,"P1 Reverse");
        log.storeValue(24,0,"P1 angleError");
        log.storeValue(25,0,"P1 angleErrorOp");
        log.storeValue(26,0,"P1 targetOp");

        log.storeValue(27,0,"P2 Angle");
        log.storeValue(28,0,"P2 Reverse");
        log.storeValue(29,0,"P2 angleError");
        log.storeValue(30,0,"P2 angleErrorOp");
        log.storeValue(31,0,"P2 targetOp");
    }

    //RobotCentric is one method of driving the swerve drive robot
    //This is the primary function for doing all the math for the module speeds and angles as well
    public void RobotCentric (double strafe, double forward, double theta, boolean zeroReset) {
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

        zeroReset(zeroReset); //used to override the wheel angles to 0 when needed to reset

        //set the angle and speed to all 4 modules
        P2.set(wa[0],ws[0]);
        D2.set(wa[1],ws[1]);
        P1.set(wa[2],ws[2]);
        D1.set(wa[3],ws[3]);
    }

    public void setModuleAngle(int angle) {
        double power = 0;
        D1.set(angle,0);
        D2.set(angle,0);
        P1.set(angle,0);
        P2.set(angle,0);
    }

    public void resetEncoders() {
        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //used to zero all the modules when reset is needed
    private void zeroReset(boolean zeroReset){
        D1.zeroReset(zeroReset);
        D2.zeroReset(zeroReset);
        P1.zeroReset(zeroReset);
        P2.zeroReset(zeroReset);
    }

    //used to set the efficiency mode to the modules
    public void setEfficiency(boolean efficiency) {
        D1.setEfficiency(efficiency);
        D2.setEfficiency(efficiency);
        P1.setEfficiency(efficiency);
        P2.setEfficiency(efficiency);
    }

    //fieldCentric is one type of common driving method for swerve drive
    //It uses the orientation of the robot to offset the joystick values. This often makes the robot easier to drive because you can pull the joystick towards you and the robot will always come towards you, even if facing another direction
    public void FieldCentric(double x1, double y1, double x2, boolean zeroReset) {
        double forwrd = y1 * -1;
        double strafe = x1;

        double gyro_radians = getAvgHeading() * Math.PI/180;
        double temp = forwrd * cos(gyro_radians) +
                strafe * sin(gyro_radians);
        strafe = -forwrd * sin(gyro_radians) +
                strafe * cos(gyro_radians);
        forwrd = temp;

        RobotCentric(forwrd,strafe, x2,zeroReset);
    }

    //moveEnvoder finds the average encoder positions of all 4 wheel modules and drives until the target value is reached
    public void moveEncoder (double ySpeed, double xSpeed, int encoder) {
        double avgEncoder = (FLMotor.getCurrentPosition()+BLMotor.getCurrentPosition()+
                            FRMotor.getCurrentPosition()+BRMotor.getCurrentPosition())/4;
        double encoderposition = FLMotor.getCurrentPosition();

        while (encoderposition<encoder) {
            gyroMove(xSpeed,ySpeed,90);
        }
        FLMotor.setPower(0);
        BLMotor.setPower(0);
        FRMotor.setPower(0);
        BRMotor.setPower(0);
    }

    //gyroMove uses the value of the current robot heading from the orientation sensor to keep the robot going straight
    public void gyroMove(double ySpeed, double xSpeed, double heading) {
        double offset = PID(.0007,0,.12,20,(int)heading,(int)getAvgHeading());
        RobotCentric(xSpeed,ySpeed,offset/40,false);
    }

    //PID loop Variables
    private int integral = 0;
    private int previousError = 0;

    double PID (double kP, double kI, double kD, int dt, int targetValue, int position) {
        int angleError = (targetValue - position);
        angleError -= (360*Math.floor(0.5+(((double)angleError)/360.0)));

        int error = angleError;

        integral += kI * error * dt;

        double u = (kP * error + integral + kD * (error - previousError) / dt);

        previousError = error;

        return u;
    }

    //Calculate the average heading of the 2 absolute orientation sensors on the robot
    private double getAvgHeading() {
        double angle = ((imu.getHeading(90)+imu2.getHeading(90))/2)%360;
        return angle;
    }

    public void RobotCentricLOG (double strafe, double forward, double theta, boolean zeroReset) throws IOException {
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

        zeroReset(zeroReset); //used to override the wheel angles to 0 when needed to reset

        //set the angle and speed to all 4 modules
        P2.set(wa[0],ws[0]);
        D2.set(wa[1],ws[1]);
        P1.set(wa[2],ws[2]);
        D1.set(wa[3],ws[3]);

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

        log.storeValueInt(12,count,D1.angle());
        log.storeValueInt(13,count,D1.reverse);
        log.storeValueInt(14,count,D1.angleError);
        log.storeValueInt(15,count,D1.angleErrorOp);
        log.storeValueInt(16,count,D1.targetOp);

        log.storeValueInt(17,count,D2.angle());
        log.storeValueInt(18,count,D2.reverse);
        log.storeValueInt(19,count,D2.angleError);
        log.storeValueInt(20,count,D2.angleErrorOp);
        log.storeValueInt(21,count,D2.targetOp);

        log.storeValueInt(22,count,P1.angle());
        log.storeValueInt(23,count,P1.reverse);
        log.storeValueInt(24,count,P1.angleError);
        log.storeValueInt(25,count,P1.angleErrorOp);
        log.storeValueInt(26,count,P1.targetOp);

        log.storeValueInt(27,count,P2.angle());
        log.storeValueInt(28,count,P2.reverse);
        log.storeValueInt(29,count,P2.angleError);
        log.storeValueInt(30,count,P2.angleErrorOp);
        log.storeValueInt(31,count,P2.targetOp);

        if (doa&&!done) {
            log.log();
            done = true;
        }
    }

    public void a (boolean value) {
        doa = value;
    }

}

/*    private void initIMU(HardwareMap hwMap, BNO055IMU imu, String name){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, name);
        imu.initialize(parameters);
    }*/