/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.HardwareSwerveV1;

@TeleOp(name="SwerveLinearTeleop Template", group="Swerve")
@Disabled
public class SwerveLinearTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareSwerveV1 robot           = new HardwareSwerveV1();   // Use the SwerveV1 hardware file

    private DcMotor  DMotor1 = null; //Driver Motor Front (1)
    private DcMotor  DMotor2 = null; //Driver Motor Back (2)
    private DcMotor  PMotor1 = null; //Passenger Motor Front (1)
    private DcMotor  PMotor2 = null; //Passenger Motor Back (2)
    //Swerve Drivebase Servos
    private Servo    DServo1 = null; //Driver ServoFront (1)
    private Servo    DServo2 = null; //Driver ServoFront (2)
    private Servo    PServo1 = null; //Passenger ServoFront (1)
    private Servo    PServo2 = null; //Passenger ServoFront (2)

    //Swerve Drivebase Encoders
    private AnalogInput DSensor1 = null; //Driver Sensor Front (1)
    private AnalogInput  DSensor2 = null; //Driver Sensor Back (2)
    private AnalogInput  PSensor1 = null; //Passenger Sensor Front (1)
    private AnalogInput  PSensor2 = null; //Passenger Sensor Back (2)

    private static final double Kp = .02;
    private static final double Ki = 0;
    private static final double Kd = .008;
    private int integral;
    private int dt = 20;
    private double u;
    private int error;
    private int previousError;
    private int setPoint;
    private double PIDpower;
    private int targetValue = 0; //180
    private double angle;
    private double opAngle;
    private boolean turnEfficiency = true;
    private int driveDirection;
    private double powerOut;

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            gamepad1.setJoystickDeadzone(.05F); //Set joystick deadzone to a lower number

            double leftX = -gamepad1.left_stick_x;
            double leftY = gamepad1.left_stick_y;
            double rightX = -gamepad1.right_stick_x;

            SwerveDriveV1(leftX,leftY,rightX);



            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }

    public void SwerveDriveV1 (double x1, double y1, double x2) {
        final double L = 12; //length between axles
        final double W = 14; //width between axles

        double r = Math.sqrt ((L * L) + (W * W));
        y1 *= -1;

        double a = x1 - x2 * (L / r);
        double b = x1 + x2 * (L / r);
        double c = y1 - x2 * (W / r);
        double d = y1 + x2 * (W / r);

        double backRightSpeed = Math.sqrt ((a * a) + (d * d));
        double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
        double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
        double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));

        //PI = 3.1415926536

        double backRightAngle = Math.atan2 (a, d) / Math.PI * 180 + 180;
        double backLeftAngle = Math.atan2 (a, c) / Math.PI * 180 + 180;
        double frontRightAngle = Math.atan2 (b, d) / Math.PI * 180 + 180;
        double frontLeftAngle = Math.atan2 (b, c) / Math.PI * 180 + 180;

        DMotor1.setPower(frontLeftSpeed);   //Set speed of Driver Motor Front(1) to front left
        DMotor2.setPower(backLeftSpeed);    //Set speed of Driver Motor Back(2) to back left
        PMotor1.setPower(frontRightSpeed);   //Set speed of Pass Motor Front(1) to front right
        PMotor2.setPower(backRightSpeed);    //Set speed of Pass Motor Back(2) to back right

        Double DSe1 = DSensor1.getVoltage(); //Get voltage of Driver Front(1) encoder
        Double DSe2 = DSensor2.getVoltage(); //Get voltage of Driver Front(1) encoder
        Double PSe1 = PSensor1.getVoltage(); //Get voltage of Driver Front(1) encoder
        Double PSe2 = PSensor1.getVoltage(); //Get voltage of Driver Front(1) encoder

        DServo1.setPosition(SwivelMathV2(DSe1,frontLeftAngle,0,5)); //Rotate the module to position
        DServo2.setPosition(SwivelMathV2(DSe2,backLeftAngle,0,5)); //Rotate the module to position
        PServo1.setPosition(SwivelMathV2(PSe1,frontRightAngle,0,5)); //Rotate the module to position
        PServo2.setPosition(SwivelMathV2(PSe2,backRightAngle,0,5)); //Rotate the module to position

    }

    public double SwivelMathV1 (double voltage, double targetAngle, double startVolt, double maxVolt) {
        double voltDegree = maxVolt/360;
        double targetVolt = startVolt + (targetAngle * voltDegree);
        double servoPower = .5;

        if (voltage>targetVolt+.5) {
            servoPower = 1;
        }else if (voltage<targetVolt+.25) {
            servoPower = .8;
        }else if (voltage<targetVolt+.15) {
            servoPower = .7;
        }else if (voltage<targetVolt+.1) {
            servoPower = .6;
        }else if (voltage<targetVolt+.02) {
            servoPower = .5;
        }else if (voltage>targetVolt-.02) {
            servoPower = .5;
        }else if (voltage>targetVolt-.1) {
            servoPower = .4;
        }else if (voltage>targetVolt-.15) {
            servoPower = .3;
        }else if (voltage>targetVolt-.25) {
            servoPower = .2;
        }else if (voltage>targetVolt-.5) {
            servoPower = 0;
        }
        return servoPower;
    }

    public double SwivelMathV2 (double voltage, double targetAngle, double zeroPosVolt, double maxVolt) {

        targetValue = (int) targetAngle;

        angle = ((voltage-zeroPosVolt)/maxVolt)*360;
        if (angle<0) {
            angle = 360+angle;
        } else {
            //
        }

        if ((angle-180)<0) {
            opAngle = 360+(angle-180);
        } else {
            opAngle = angle-180;
        }

        if (turnEfficiency=true) {
            if ((Math.abs(targetValue-angle))<=(Math.abs(targetValue-opAngle))) {
                setPoint = (int) angle;
                driveDirection = 1;
            } else {
                setPoint = (int) opAngle;
                driveDirection = -1;
            }
        } else {
            setPoint = (int) angle;
        }

        error = targetValue - setPoint;

        integral += Ki * error * dt;

        if(integral > targetValue * 0.25) {
            integral = (int) (targetValue * 0.25);
        }

        u = (Kp * error + integral + Kd * (error - previousError) / dt);

        previousError = error;

        PIDpower = -1*u;

        if (PIDpower>0) {
            powerOut = .5+(PIDpower/2);
        } else if (PIDpower<0) {
            powerOut = .5+(PIDpower/2);
        } else {
            powerOut = PIDpower;
        }

        return powerOut;
    }


}
