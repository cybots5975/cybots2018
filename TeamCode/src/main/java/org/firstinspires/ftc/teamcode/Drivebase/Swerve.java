package org.firstinspires.ftc.teamcode.drivebase;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

/**
 * Created by kskrueger on 10/10/17.
 */

public class Swerve {
    public Module D1, D2, P1, P2;
    //public DcMotor DMotor1, DMotor2, PMotor1, PMotor2;
    //public Servo DServo1, DServo2, PServo1, PServo2;
    //public AnalogInput DSensor1, DSensor2, PSensor1, PSensor2;
    public BNO055IMU imu;

    double d1, d2, p1, p2;

    public Swerve (HardwareMap hwMap,
                   DcMotor DMotor1, Servo DServo1, AnalogInput DSensor1,
                   DcMotor DMotor2, Servo DServo2, AnalogInput DSensor2,
                   DcMotor PMotor1, Servo PServo1, AnalogInput PSensor1,
                   DcMotor PMotor2, Servo PServo2, AnalogInput PSensor2){

        //Define and Initialize Motors
        DMotor1 = hwMap.dcMotor.get("DM1"); //Driver Motor Front(1)
        DMotor2 = hwMap.dcMotor.get("DM2"); //Driver Motor Back(2)
        PMotor1 = hwMap.dcMotor.get("PM1"); //Passenger Motor Front(1)
        PMotor2 = hwMap.dcMotor.get("PM2"); //Passenger Motor Back(2)

        //Define and initialize ALL installed servos.
        DServo1 = hwMap.servo.get("DS1"); //Driver Servo Front(1)
        DServo2 = hwMap.servo.get("DS2"); //Driver Servo Back(2)
        PServo1 = hwMap.servo.get("PS1"); //Pass Servo Front(1)
        PServo2 = hwMap.servo.get("PS2"); //Pass Servo Back(2)

        DSensor1 = hwMap.analogInput.get("DSe1");
        DSensor2 = hwMap.analogInput.get("DSe2");
        PSensor1 = hwMap.analogInput.get("PSe1");
        PSensor2 = hwMap.analogInput.get("PSe2");

        DServo1.setPosition(.5); //Set Driver Servo Front(1) to 0 power
        DServo2.setPosition(.5); //Set Driver Servo Back(2) to 0 power
        PServo1.setPosition(.5); //Set Pass Servo Front(1) to 0 power
        PServo2.setPosition(.5); //Set Pass Servo Back(2) to 0 power


        PMotor1.setDirection(DcMotor.Direction.REVERSE);
        PMotor2.setDirection(DcMotor.Direction.REVERSE);

        //Set all motors to zero power
        DMotor1.setPower(0); //Set Drive Motor 1 to 0% power
        DMotor2.setPower(0); //Set Drive Motor 2 to 0% power
        PMotor1.setPower(0); //Set Pass Motor 1 to 0% power
        PMotor2.setPower(0); //Set Pass Motor 2 to 0% power

        DMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        D1 = new Module(DMotor1,DServo1,DSensor1,.391);
        D2 = new Module(DMotor2,DServo2,DSensor2,1.542);
        P1 = new Module(PMotor1,PServo1,PSensor1,1.503);
        P2 = new Module(PMotor2,PServo2,PSensor2,.025);
    }

    public void RobotCentric (double x1, double y1, double x2) {
        final double L = 13.25; //length between axles
        final double W = 15.5; //width between axles

        double r = (Math.sqrt ((L * L) + (W * W)));
        y1 *= -1;

        double a = x1 - x2 * (L / r);
        double b = x1 + x2 * (L / r);
        double c = y1 - x2 * (W / r);
        double d = y1 + x2 * (W / r);

        double P2Speed = Math.sqrt ((a * a) + (d * d));
        double D2Speed = Math.sqrt ((a * a) + (c * c));
        double P1Speed = Math.sqrt ((b * b) + (d * d));
        double D1Speed = Math.sqrt ((b * b) + (c * c));

        double P2Angle = Math.atan2 (a, d) / Math.PI * 180 - 180;
        double D2Angle = Math.atan2 (a, c) / Math.PI * 180 - 180;
        double P1Angle = Math.atan2 (b, d) / Math.PI * 180 + 180;
        double D1Angle = Math.atan2 (b, c) / Math.PI * 180 + 180;

        D1.set((int)D1Angle,D1Speed);
        d1 = D1.direction();
        D2.set((int)D2Angle,D2Speed);
        d2 = D2.direction();
        P1.set((int)P1Angle,P1Speed);
        p1 = P1.direction();
        P2.set((int)P2Angle,P2Speed);
        p2 = P2.direction();
    }

    public String directions() {
        return "D1: "+d1+" D2: "+d2+" P1: "+p1+" P2: "+p2;
    }

    public void FieldCentric(double x1, double y1, double x2) {
        double rcw = x2;
        double forwrd = y1 * -1;
        double strafe = x1;

        double gyro_degrees = 0;//angles.firstAngle;
        //sensor.IMU.Angle.Average(IMU1, IMU2);
        double gyro_radians = gyro_degrees * Math.PI/180;
        double temp = forwrd * cos(gyro_radians) +
                strafe * sin(gyro_radians);
        strafe = -forwrd * sin(gyro_radians) +
                strafe * cos(gyro_radians);
        forwrd = temp;

        RobotCentric(forwrd,strafe,rcw);
    }
}
