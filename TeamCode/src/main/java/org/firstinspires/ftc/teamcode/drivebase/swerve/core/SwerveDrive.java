package org.firstinspires.ftc.teamcode.drivebase.swerve.core;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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
    public DcMotor FLMotor, BLMotor, FRMotor, BRMotor;
    private IMU imu, imu2;

    public SwerveDrive(HardwareMap hwMap, BNO055IMU imuDS, BNO055IMU imuPS,
                       DcMotor FLMotor, Servo FLServo, AnalogInput FLSensor,
                       DcMotor BLMotor, Servo BLServo, AnalogInput BLSensor,
                       DcMotor FRMotor, Servo FRServo, AnalogInput FRSensor,
                       DcMotor BRMotor, Servo BRServo, AnalogInput BRSensor){

        //Define and initialize drive DC Motors
        FLMotor = hwMap.dcMotor.get("DM1"); //Driver Motor Front(1)
        BLMotor = hwMap.dcMotor.get("DM2"); //Driver Motor Back(2)
        FRMotor = hwMap.dcMotor.get("PM1"); //Passenger Motor Front(1)
        BRMotor = hwMap.dcMotor.get("PM2"); //Passenger Motor Back(2)

        //Define and initialize Servos motors
        FLServo = hwMap.servo.get("DS1"); //Driver Servo Front(1)
        BLServo = hwMap.servo.get("DS2"); //Driver Servo Back(2)
        FRServo = hwMap.servo.get("PS1"); //Pass Servo Front(1)
        BRServo = hwMap.servo.get("PS2"); //Pass Servo Back(2)

        //Define and initialize
        FLSensor = hwMap.analogInput.get("DSe1");
        BLSensor = hwMap.analogInput.get("DSe2");
        FRSensor = hwMap.analogInput.get("PSe1");
        BRSensor = hwMap.analogInput.get("PSe2");

        FLServo.setPosition(.5); //Set Driver Servo Front(1) to 0 power
        BLServo.setPosition(.5); //Set Driver Servo Back(2) to 0 power
        FRServo.setPosition(.5); //Set Pass Servo Front(1) to 0 power
        BRServo.setPosition(.5); //Set Pass Servo Back(2) to 0 power

        //reverse the motors directions on the passenger side
        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);

        //set motors to run using encoders for positioning and speed control
        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.FLMotor = FLMotor;
        this.BLMotor = BLMotor;
        this.FRMotor = FRMotor;
        this.BRMotor = BRMotor;

        //define the 4 swerve drive modules with motor,servo,encoder sensor,and starting voltage
        D1 = new Module(FLMotor,FLServo,FLSensor,Constants.FL_OFFSET); //driver side module 1
        D2 = new Module(BLMotor,BLServo,BLSensor,Constants.BL_OFFSET); //driver side module 2
        P1 = new Module(FRMotor,FRServo,FRSensor,Constants.FR_OFFSET); //passenger side module 1
        P2 = new Module(BRMotor,BRServo,BRSensor,Constants.BR_OFFSET); //passenger side module 2

        imu = new IMU(imuDS);
        imu2 = new IMU(imuPS);

        imu.initIMU(hwMap,"imu");
        imu2.initIMU(hwMap,"imu2");
        previousError = 0;
    }

    private void initIMU(HardwareMap hwMap, BNO055IMU imu, String name){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, name);
        imu.initialize(parameters);
    }

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

    public void setEfficiency(boolean efficiency) {
        D1.setEfficiency(efficiency);
        D2.setEfficiency(efficiency);
        P1.setEfficiency(efficiency);
        P2.setEfficiency(efficiency);
    }

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

/*        if (Math.abs(angleError)<2) {
            angleError = 0;
        }*/

        integral += kI * error * dt;

        double u = (kP * error + integral + kD * (error - previousError) / dt);

        previousError = error;

        return u;
    }

/*    private double getHeading1() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle+360+90)%360;
    }

    private double getHeading2() {
        Orientation angles = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle+360+90)%360;
    }*/

    private double getAvgHeading() {
        double angle = ((imu.getHeading(90)+imu2.getHeading(90))/2)%360;
        return angle;//(Math.abs(360-angle));
    }

}