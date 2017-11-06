package org.firstinspires.ftc.teamcode.drivebase.swerve.core;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Sensors.IMU;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

/**
 * Created by Karter Krueger on 10/10/17.
 */

public class SwerveDrive {
    private Module D1, D2, P1, P2;
    private DcMotor FLMotor, BLMotor, FRMotor, BRMotor;
    private IMU imu, imu2;

    public SwerveDrive(HardwareMap hwMap, IMU imuDS, IMU imuPS,
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
        D1 = new Module(FLMotor,FLServo,FLSensor,Constants.FL_OFFSET); //driver side module 1
        D2 = new Module(BLMotor,BLServo,BLSensor,Constants.BL_OFFSET); //driver side module 2
        P1 = new Module(FRMotor,FRServo,FRSensor,Constants.FR_OFFSET); //passenger side module 1
        P2 = new Module(BRMotor,BRServo,BRSensor,Constants.BR_OFFSET); //passenger side module 2

        previousError = 0;
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
        angleError -= (360*Math.floor(0.5+((angleError +0d)/360.0)));

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