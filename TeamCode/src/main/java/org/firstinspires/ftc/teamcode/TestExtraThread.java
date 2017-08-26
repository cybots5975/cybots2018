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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name="", group="Swerve")
@Disabled
public class TestExtraThread extends SwerveLinearBase {

    /* Declare OpMode members. */
    HardwareSwerveV1 robot           = new HardwareSwerveV1();   // Use the SwerveV1 hardware file

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        final boolean thread_run=true; /*Set to false to stop the thread i.e. when your opmode is ending */
        final double rpm_gate_time=250; /*How long to wait (mS) between encoder samples - trade off between update rate and precision*/
        //double LRPM,RRPM; /*Left motor RPM, Right motor RPM*/

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            gamepad1.setJoystickDeadzone(.05F); //Set joystick deadzone to a lower number

            double leftX = -gamepad1.left_stick_x;
            double leftY = gamepad1.left_stick_y;
            double rightX = -gamepad1.right_stick_x;

            SwerveDriveRobotCentricV2(leftX,leftY,rightX,false);

            telemetry.addData("Front Driver Power", DMotor1.getPower());
            telemetry.addData("Back Driver Power", DMotor2.getPower());
            telemetry.addData("Front Pass Power", PMotor1.getPower());
            telemetry.addData("Back Pass Power", PMotor2.getPower());

            telemetry.addData("Front Driver Angle", ((DSensor1.getVoltage())/5)*360);
            telemetry.addData("Back Driver Angle", ((DSensor2.getVoltage())/5)*360);
            telemetry.addData("Front Pass Angle", ((PSensor1.getVoltage())/5)*360);
            telemetry.addData("Back Pass Angle", ((PSensor2.getVoltage())/5)*360);


            //double LRPM,RRPM; /*Left motor RPM, Right motor RPM*/

            final ElapsedTime tm = new ElapsedTime();
            new Thread(new Runnable() {
                @Override
                public void run() {
                    double sms;
                    double LRPM, RRPM; /*Left motor RPM, Right motor RPM*/
                    while (thread_run) {
                /*left and right are dcMotor instances*/
                        int last_left_encoder = DMotor1.getCurrentPosition(); /*Get first sample*/
                        int last_right_encoder = PMotor1.getCurrentPosition();
                        sms = tm.milliseconds();
                        while (tm.milliseconds() - sms < rpm_gate_time) {
                        } /*Wait rpm_gate_time mS*/
                        int delta_l = DMotor1.getCurrentPosition() - last_left_encoder; /*Get second sample, subtract first sample to get change*/
                        int delta_r = PMotor1.getCurrentPosition() - last_right_encoder;
                        double factor = ((1000 / rpm_gate_time) * 60) / 1120; /*Compute factor to convert encoder ticks per gate window to RPM (assumes 1120 ticks/rev)*/
                        double RPM = delta_l * factor; /*Calculate the RPM for the left motor*/
                        if (Math.abs(RPM) < 400) {
                    /*If we get an absurdly high RPM, the it may be encoder jitter or the encoder was reset mid reading; keep the last reading instead*/
                            LRPM = -RPM; /*Store the calculated RPM in the LRPM variable*/
                        }
                        RPM = delta_r * factor; /*^ Ditto for the right motor*/
                        if (Math.abs(RPM) < 400) {
                            RRPM = -RPM;
                        }
                    }
                }
            }).start();


                        //telemetry.addData("Motor RPM: ", "%.f, %.f", LRPM, RRPM); //The last measured/computed RPM value will always be available in the LRPM and RRPM global variables
                        telemetry.update();

                        // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
                        robot.waitForTick(40);
                    }
                }

                //boolean thread_run=true; /*Set to false to stop the thread i.e. when your opmode is ending */
                //double rpm_gate_time=250; /*How long to wait (mS) between encoder samples - trade off between update rate and precision*/
                //double LRPM,RRPM; /*Left motor RPM, Right motor RPM*/
                //public void init(){
    /* ..... */
        /*final ElapsedTime tm = new ElapsedTime();
        new Thread(new Runnable() {
            @Override
            public void run() {
                double sms;
                while (thread_run) {
                *//*left and right are dcMotor instances*//*
                    int last_left_encoder = DMotor1.getCurrentPosition(); *//*Get first sample*//*
                    int last_right_encoder = PMotor1.getCurrentPosition();
                    sms = tm.milliseconds();
                    while(tm.milliseconds()-sms < rpm_gate_time){} *//*Wait rpm_gate_time mS*//*
                    int delta_l = DMotor1.getCurrentPosition() - last_left_encoder; *//*Get second sample, subtract first sample to get change*//*
                    int delta_r = PMotor1.getCurrentPosition() - last_right_encoder;
                    double factor = ((1000/rpm_gate_time)*60)/1120; *//*Compute factor to convert encoder ticks per gate window to RPM (assumes 1120 ticks/rev)*//*
                    double RPM = delta_l * factor; *//*Calculate the RPM for the left motor*//*
                    if(Math.abs(RPM) < 400){
                    *//*If we get an absurdly high RPM, the it may be encoder jitter or the encoder was reset mid reading; keep the last reading instead*//*
                        LRPM = -RPM; *//*Store the calculated RPM in the LRPM variable*//*
                    }
                    RPM = delta_r * factor; *//*^ Ditto for the right motor*//*
                    if(Math.abs(RPM) < 400){
                        RRPM = -RPM;
                    }
                }
            }
        }).start();
    *//* ..... *//*
    }*/
   /* public void loop(){
    *//* ..... *//*
        telemetry.addData("Motor RPM: ", "%.f, %.f", LRPM, RRPM); //The last measured/computed RPM value will always be available in the LRPM and RRPM global variables
        telemetry.update();
    *//* ..... *//*
    }*/

            }
