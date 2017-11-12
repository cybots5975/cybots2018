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
package org.firstinspires.ftc.teamcode.oldSwerve;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="SwerveLinearTeleop Template", group="Swerve")
@Disabled
public class FieldCentricTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareSwerveV1 robot           = new HardwareSwerveV1();   // Use the SwerveV1 hardware file

    public DcMotor  DMotor1 = null; //Driver Motor Front (1)
    public DcMotor  DMotor2 = null; //Driver Motor Back (2)
    public DcMotor  PMotor1 = null; //Passenger Motor Front (1)
    public DcMotor  PMotor2 = null; //Passenger Motor Back (2)
    //Swerve Drivebase Servos
    public Servo    DServo1 = null; //Driver ServoFront (1)
    public Servo    DServo2 = null; //Driver ServoFront (2)
    public Servo    PServo1 = null; //Passenger ServoFront (1)
    public Servo    PServo2 = null; //Passenger ServoFront (2)

    //Swerve Drivebase Encoders
    public AnalogInput DSensor1 = null; //Driver Sensor Front (1)
    public AnalogInput  DSensor2 = null; //Driver Sensor Back (2)
    public AnalogInput  PSensor1 = null; //Passenger Sensor Front (1)
    public AnalogInput  PSensor2 = null; //Passenger Sensor Back (2)

    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    // Used to swap robot orientation

    private String orientation = "FORWARD";

    //Helper class with common robot methods
    //private Robot robot = new Robot();


    // Speed values for drive motors

    private double mFrontLeftThrottle = 0;

    private double mFrontRightThrottle = 0;

    private double mBackLeftThrottle = 0;

    private double mBackRightThrottle = 0;


    // Values from gamepad joystick

    private double xStick = 0;

    private double yStick = 0;

    private double zStick = 0;



    // Enable or disable field oriented drive

    private boolean fodEnabled = true;


    private boolean timerStarted = false;

    private double startTime = 0;

    private boolean driveWithTrig = false;

    // Rotate force vectors using navx gyro

    private static double[] rotateVector(double x, double y, double angle) {

        double cosA = Math.cos(angle * (3.14159 / 180.0));

        double sinA = Math.sin(angle * (3.14159 / 180.0));

        double[] out = new double[2];

        out[0] = x * cosA - y * sinA;

        out[1] = x * sinA + y * cosA;

        return out;

    }


    // Apply rotated vectors to throttle values

    private void fieldOrientedDrive(double x, double y, double rotation, double gyroAngle) {

        double xIn = x;
        double yIn = y;
        yIn = -yIn;
        double[] rotated = rotateVector(xIn, yIn, gyroAngle);
        xIn = rotated[0];
        yIn = rotated[1];


        mFrontLeftThrottle = xIn + yIn + rotation;
        mFrontRightThrottle = -xIn + yIn - rotation;
        mBackLeftThrottle = -xIn + yIn + rotation;
        mBackRightThrottle = xIn + yIn - rotation;

    }

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //set the maximum power to -ve .15 and +ve 0.15 regardless of the throttle

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /*

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ,AngleUnit.DEGREES);

            //double gyro_degrees = imu.getPosition();




            telemetry.addData("MFrontLeft", robot.DMotor1.getPower());

            telemetry.addData("MFrontRight", robot.PMotor1.getPower());

            telemetry.addData("MBackLeft", robot.DMotor2.getPower());

            telemetry.addData("MBackRight", robot.PMotor2.getPower());

            telemetry.update();


            xStick = -gamepad1.left_stick_x;

            yStick = -gamepad1.left_stick_y;

            zStick = gamepad1.right_stick_x;


            //allows to switch back and front of the robot in a head-oriented drive

            if (gamepad1.dpad_up) {

                orientation = "FORWARD";

            }

            if (gamepad1.dpad_down) {

                orientation = "BACKWARD";

            }


            // Allows the driver to switch between a head-oriented and field-oriented drive.

            if (gamepad1.left_stick_button) {

                fodEnabled = false;

            }

            if (gamepad1.right_stick_button) {

                fodEnabled = true;

            }


            //if field oriented drive, drive based on drivers reference point

            //else drive based on head-orientation

            if (fodEnabled) {

                fieldOrientedDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gyro_degrees);

            }

            if (!fodEnabled) {
                //else drive based on driver's input

                headorientedDrive();

            }


            //Gamepad1 Drive controls

            if (gamepad1.right_trigger > 0) {

                double speed = gamepad1.right_trigger;

                driveWithTrig = true;

                robot.setSpeed(speed,speed,speed,speed);

            }

            if (gamepad1.left_trigger > 0) {

                double speed = gamepad1.left_trigger;

                driveWithTrig = true;

                robot.setSpeed(-speed,-speed,-speed,-speed);

            }

            if (gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0) {

                driveWithTrig = false;

            }


            if (!robot.isNull() && !driveWithTrig) {

                robot.setSpeed(mFrontLeftThrottle, mFrontRightThrottle, mBackLeftThrottle, mBackRightThrottle);

            }






            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }

    public void SwerveDrive (double x1, double y1, double x2) {
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

        double backRightAngle = Math.atan2 (a, d) / 3.1415926536 *180 +180;
        double backLeftAngle = Math.atan2 (a, c) / 3.1415926536 *180 +180;
        double frontRightAngle = Math.atan2 (b, d) / 3.1415926536 *180 +180;
        double frontLeftAngle = Math.atan2 (b, c) / 3.1415926536 *180 +180;

        DMotor1.setPower(frontLeftSpeed);   //Set speed of Driver Motor Front(1) to front left
        DMotor2.setPower(backLeftSpeed);    //Set speed of Driver Motor Back(2) to back left
        PMotor1.setPower(frontRightSpeed);   //Set speed of Pass Motor Front(1) to front right
        PMotor2.setPower(backRightSpeed);    //Set speed of Pass Motor Back(2) to back right

        Double DS1 = DSensor1.getVoltage(); //Get voltage of Driver Front(1) encoder
        Double DS2 = DSensor2.getVoltage(); //Get voltage of Driver Front(1) encoder
        Double PS1 = PSensor1.getVoltage(); //Get voltage of Driver Front(1) encoder
        Double PS2 = PSensor2.getVoltage(); //Get voltage of Driver Front(1) encoder

        DServo1.setPosition(SwivelMath(DS1,frontLeftAngle,0,5)); //Rotate the module to position
        DServo2.setPosition(SwivelMath(DS2,backLeftAngle,0,5)); //Rotate the module to position
        PServo1.setPosition(SwivelMath(PS1,frontRightAngle,0,5)); //Rotate the module to position
        PServo2.setPosition(SwivelMath(PS2,backRightAngle,0,5)); //Rotate the module to position

    }

    public double SwivelMath (double voltage, double targetAngle, double startVolt, double maxVolt) {
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

    private void driveSlowAvoidCollision()

    {

        if (orientation.equals("FORWARD") ) {

            if (yStick < 0 && (xStick < .08 && xStick > -.08)) {

                mFrontLeftThrottle = Range.clip(yStick - xStick + zStick, -.15, .15);

                mFrontRightThrottle = Range.clip(yStick + xStick - zStick, -.15, .15);

                mBackLeftThrottle = Range.clip(yStick + xStick + zStick, -.15, .15);

                mBackRightThrottle = Range.clip(yStick - xStick - zStick, -.15, .15);

            }

            else {

                headorientedDrive();

            }

        }

        else if (orientation.equals("BACKWARD")) {

            if (yStick > 0 && (xStick < .08 && xStick > -.08)) {

                mFrontLeftThrottle = -Range.clip(yStick - xStick - zStick, -.15, .15);

                mFrontRightThrottle = -Range.clip(yStick + xStick + zStick, -.15, .15);

                mBackLeftThrottle = -Range.clip(yStick + xStick - zStick, -.15, .15);

                mBackRightThrottle = -Range.clip(yStick - xStick + zStick, -.15, .15);

            }

            else {

                headorientedDrive();

            }

        }

    }


    //change direction based the orientation set by the driver

    private void headorientedDrive()

    {

        if (orientation.equals("FORWARD")) {

            mFrontLeftThrottle = Range.clip(yStick - xStick + zStick, -1, 1);

            mFrontRightThrottle = Range.clip(yStick + xStick - zStick, -1, 1);

            mBackLeftThrottle = Range.clip(yStick + xStick + zStick, -1, 1);

            mBackRightThrottle = Range.clip(yStick - xStick - zStick, -1, 1);

        }

        else if (orientation.equals("BACKWARD")) {

            mFrontLeftThrottle = -Range.clip(yStick - xStick - zStick, -1, 1);

            mFrontRightThrottle = -Range.clip(yStick + xStick + zStick, -1, 1);

            mBackLeftThrottle = -Range.clip(yStick + xStick - zStick, -1, 1);

            mBackRightThrottle = -Range.clip(yStick - xStick + zStick, -1, 1);

        }

    }*/

    }

}





