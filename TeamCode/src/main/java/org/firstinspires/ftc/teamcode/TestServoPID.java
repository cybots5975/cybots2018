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

@TeleOp(name="Servo Swivel Test PID", group="Swerve")
//@Disabled
public class TestServoPID extends LinearOpMode {

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

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        DSensor1 = hardwareMap.analogInput.get("DSe1");
        DServo1 = hardwareMap.servo.get("DS1");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double leftX = gamepad1.left_stick_x;

            double dvolt = DSensor1.getVoltage();

            //DServo1.setPosition(SwivelMath2(dvolt,leftX*180 +180,0,4.8));

            if (dvolt>2.41) {
                DServo1.setPosition(.55);
            }else if (dvolt<2.39) {
                DServo1.setPosition(.45);
            }else DServo1.setPosition(.5);


            //telemetry.addData("Position",SwivelMath(dvolt,leftX*180 +180,0,5));
            telemetry.addData("Volt",DSensor1.getVoltage());
            telemetry.update();



            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
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