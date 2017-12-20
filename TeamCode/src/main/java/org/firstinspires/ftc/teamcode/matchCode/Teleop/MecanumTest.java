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
package org.firstinspires.ftc.teamcode.matchCode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.general.Robot;

@TeleOp(name="Test Mecanum", group="Template")
//@Disabled
public class MecanumTest extends LinearOpMode {
    Robot robot = new Robot(); //use the SwerveV1 hardware file to configure
    Boolean lastButton = true, stateButton = true, modeField = false;

    @Override
    public void runOpMode() {


        robot.init(hardwareMap);
        robot.drive.zeroEncoders();
        robot.drive.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.a && !lastButton) {
                stateButton = !stateButton;
                if(stateButton) {
                    modeField = true;
                    telemetry.addData("Mode","Robot Centric");
                } else {
                    //other mode here
                    modeField = false;
                    telemetry.addData("Mode","Field Centric");
                }
            }
            lastButton = gamepad1.a;

            if (modeField) {
                robot.drive.driveMecanumField(-gamepad1.left_stick_y,-gamepad1.left_stick_x,gamepad1.right_stick_x);
            } else {
                robot.drive.driveMecanum(-gamepad1.left_stick_y,-gamepad1.left_stick_x,gamepad1.right_stick_x);
            }

            if (gamepad1.right_trigger>.1) {
                robot.intake.setSpeed(gamepad1.right_trigger);
            } else {
                robot.intake.setSpeed(-gamepad1.left_trigger);
            }

            double FL = robot.drive.FLMotor.getCurrentPosition();
            double FR = robot.drive.FRMotor.getCurrentPosition();
            double BL = robot.drive.BLMotor.getCurrentPosition();
            double BR = robot.drive.BRMotor.getCurrentPosition();

            telemetry.addData("Encoder Counts",robot.drive.getStrafeEncoderAverage());
            telemetry.addData("FL",FL);
            telemetry.addData("FR",FR);
            telemetry.addData("BL",BL);
            telemetry.addData("BR",BR);

            telemetry.update();
        }
    }
}