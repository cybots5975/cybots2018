/*Copyright (c) 2016 Robert AtkinsonAll rights reserved.Redistribution and use in source and binary forms, with or without modification,are permitted (subject to the limitations in the disclaimer below) provided thatthe following conditions are met:Redistributions of source code must retain the above copyright notice, this listof conditions and the following disclaimer.Redistributions in binary form must reproduce the above copyright notice, thislist of conditions and the following disclaimer in the documentation and/orother materials provided with the distribution.Neither the name of Robert Atkinson nor the names of his contributors may be used toendorse or promote products derived from this software without specific priorwritten permission.NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THISLICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSEARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLEFOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIALDAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS ORSERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVERCAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, ORTORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OFTHIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*/package org.firstinspires.ftc.teamcode.oldSwerve;import com.qualcomm.hardware.bosch.BNO055IMU;import com.qualcomm.robotcore.hardware.AnalogInput;import com.qualcomm.robotcore.hardware.DcMotor;import com.qualcomm.robotcore.hardware.Servo;import org.firstinspires.ftc.robotcore.external.navigation.Orientation;import static java.lang.Math.cos;import static java.lang.Math.sin;public class SwerveDrive {    //PID Variables    int integral = 0;    double u;    int error;    int previousError = 0;    double PIDpower;    //general variables    public double powerOut;    public double maxVolt = 2.06;    public int reverse;    //define hardware devices    private DcMotor DMotor1, DMotor2, PMotor1, PMotor2;    private Servo DServo1, DServo2, PServo1, PServo2;    private AnalogInput DSensor1, DSensor2, PSensor1, PSensor2;    public BNO055IMU imu;    Orientation angles;    public void RobotCentric (double x1, double y1, double x2) {        final double L = 13.25; //length between axles        final double W = 15.5; //width between axles        double r = (Math.sqrt ((L * L) + (W * W)));        y1 *= -1;        double a = x1 - x2 * (L / r);        double b = x1 + x2 * (L / r);        double c = y1 - x2 * (W / r);        double d = y1 + x2 * (W / r);        double P2Speed = Math.sqrt ((a * a) + (d * d));        double D2Speed = Math.sqrt ((a * a) + (c * c));        double P1Speed = Math.sqrt ((b * b) + (d * d));        double D1Speed = Math.sqrt ((b * b) + (c * c));        double P2Angle = Math.atan2 (a, d) / Math.PI * 180 - 180;        double D2Angle = Math.atan2 (a, c) / Math.PI * 180 - 180;        double P1Angle = Math.atan2 (b, d) / Math.PI * 180 + 180;        double D1Angle = Math.atan2 (b, c) / Math.PI * 180 + 180;        module(D1Angle,D1Speed,DMotor1,DServo1,DSensor1,1.627);        module(D2Angle,D2Speed,DMotor2,DServo2,DSensor2,0.496);        module(P1Angle,P1Speed,PMotor1,PServo1,PSensor1,0.426);        module(P2Angle,P2Speed,PMotor2,PServo2,PSensor2,1.059);    }    public void module(double angle, double speed, DcMotor motor, Servo servo, AnalogInput encoder, double zeroPosition) {        setAngle(servo,angle,zeroPosition,encoder);        double direction = reverse;        setVelocity(motor,direction*speed);    }    void setAngle(Servo servo, double targetAngle, double zeroPosition, AnalogInput encoder) {        servo.setPosition(swivelPID(angle(encoder,zeroPosition),((int)targetAngle)));    }    public void setVelocity(DcMotor motor, double speed) {        motor.setPower(speed);    }    public int angle(AnalogInput encoder, double zeroPosition) {        double angle = ((encoder.getVoltage()-zeroPosition)/maxVolt)*360;        if (angle<0) {            angle = 360+angle;        }        return (int)angle;    }    public int reverse180(int target, int position) {        int target180 = (target+180)%360;        int error;        double angError = (target-position)-(360*Math.floor(0.5+(((target-position)+0d)/360.0)));        double angError180 = (target180-position)-(360*Math.floor(0.5+(((target180-position)+0d)/360.0)));        if (Math.abs(angError)>Math.abs(angError180)) {            error = (int)angError;            reverse = 1;        } else {            error = (int)angError180;            reverse = -1;        }        return error;    }    public double swivelPID (int angle, int targetAngle) {        final double Kp = .02;        final double Ki = 0;        final double Kd = .008;        int dt = 20;        error = reverse180(targetAngle,angle);        integral += Ki * error * dt;        u = (Kp * error + integral + Kd * (error - previousError) / dt);        previousError = error;        PIDpower = -1*u;        //convert to power for the servo (from 0-1)        if (PIDpower>0) {            powerOut = .5+(PIDpower/2);        } else if (PIDpower<0) {            powerOut = .5+(PIDpower/2);        } else {            powerOut = PIDpower;        }        return powerOut;    }    public void FieldCentric(double x1, double y1, double x2) {        double rcw = x2;        double forwrd = y1 * -1;        double strafe = x1;        double gyro_degrees = angles.firstAngle;        double gyro_radians = gyro_degrees * Math.PI/180;        double temp = forwrd * cos(gyro_radians) +                strafe * sin(gyro_radians);        strafe = -forwrd * sin(gyro_radians) +                strafe * cos(gyro_radians);        forwrd = temp;        RobotCentric(forwrd,strafe,rcw);    }}