/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="K9bot: Telop Tank", group="K9bot")
//@Disabled
public class K9botTeleopTank_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    //HardwareSwerveV1 robot           = new HardwareSwerveV1();   // Use the SwerveV1 hardware file

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

    @Override
    public void runOpMode() {
       /* Initialize the hardware variables.
        * The init() method of the hardware class does all the work here
        */
        //robot.init(hardwareMap);

        DSensor1 = hardwareMap.analogInput.get("DSe1");
        DServo1 = hardwareMap.servo.get("DS1");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

<<<<<<< HEAD
            double leftX = gamepad1.left_stick_x;

            double dvolt = DSensor1.getVoltage();

            //DServo1.setPosition(SwivelMath2(dvolt,leftX*180 +180,0,4.8));

            if (dvolt>2.41+2) {
                DServo1.setPosition(.6);
            } else if (dvolt>2.41) {
                DServo1.setPosition(.6);
            }else if (dvolt<2.39) {
                DServo1.setPosition(.4);
            }else DServo1.setPosition(.5);


            //telemetry.addData("Position",SwivelMath(dvolt,leftX*180 +180,0,5));
            telemetry.addData("Volt",DSensor1.getVoltage());
            telemetry.update();



            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            //robot.waitForTick(40);
=======
            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;
            robot.leftDrive.setPower(left);
            robot.rightDrive.setPower(right);

            // Use gamepad Y & A raise and lower the arm
            if (gamepad1.a)
                armPosition += ARM_SPEED;
            else if (gamepad1.y)
                armPosition -= ARM_SPEED;

            // Use gamepad X & B to open and close the claw
            if (gamepad1.x)
                clawPosition += CLAW_SPEED;
            else if (gamepad1.b)
                clawPosition -= CLAW_SPEED;

            // Move both servos to new position.
            armPosition  = Range.clip(armPosition, robot.ARM_MIN_RANGE, robot.ARM_MAX_RANGE);
            robot.arm.setPosition(armPosition);
            clawPosition = Range.clip(clawPosition, robot.CLAW_MIN_RANGE, robot.CLAW_MAX_RANGE);
            robot.claw.setPosition(clawPosition);

            // Send telemetry message to signify robot running;
            telemetry.addData("arm",   "%.2f", armPosition);
            telemetry.addData("claw",  "%.2f", clawPosition);
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();

            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);
>>>>>>> 9f1e829930e7e5e9c74d26aada5e19742dcd1e82
        }
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

    public double SwivelMath2 (double voltage, double targetAngle, double startVolt, double maxVolt) {
        double currentAngle = (voltage-startVolt)/(maxVolt/360);
        double servoPower;

        if (currentAngle>targetAngle+45) {
            servoPower = 1;
        }else if (currentAngle<targetAngle+25) {
            servoPower = .8;
        }else if (currentAngle<targetAngle+10) {
            servoPower = .65;
        }else if (currentAngle<targetAngle+5) {
            servoPower = .55;
        }else if (currentAngle<targetAngle+1) {
            servoPower = .5;
        }else if (currentAngle>targetAngle-1) {
            servoPower = .5;
        }else if (currentAngle>targetAngle-5) {
            servoPower = .45;
        }else if (currentAngle>targetAngle-10) {
            servoPower = .35;
        }else if (currentAngle>targetAngle-25) {
            servoPower = .2;
        }else if (currentAngle>targetAngle-45) {
            servoPower = 0;
        }else servoPower = .5;

        telemetry.addData("Current Angle", currentAngle);

        return servoPower;
    }


}