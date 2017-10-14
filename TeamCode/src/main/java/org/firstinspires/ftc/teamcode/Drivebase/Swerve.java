package org.firstinspires.ftc.teamcode.drivebase;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

/**
 * Created by kskrueger on 10/10/17.
 */

public class Swerve {
    public Module D1, D2, P1, P2;
    private DcMotor DMotor1, DMotor2, PMotor1, PMotor2;
    private Servo DServo1, DServo2, PServo1, PServo2;
    private AnalogInput DSensor1, DSensor2, PSensor1, PSensor2;
    public BNO055IMU imu;

    public void init() {
        D1 = new Module();
        D2 = new Module();
        P1 = new Module();
        P2 = new Module();
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

        D1.set(D1Angle,D1Speed,DMotor1,DServo1,DSensor1,1.627);
        D2.set(D2Angle,D2Speed,DMotor2,DServo2,DSensor2,0.496);
        P1.set(P1Angle,P1Speed,PMotor1,PServo1,PSensor1,0.426);
        P2.set(P2Angle,P2Speed,PMotor2,PServo2,PSensor2,1.059);
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
