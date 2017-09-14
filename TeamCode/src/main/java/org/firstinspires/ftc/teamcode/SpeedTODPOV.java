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
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Speed Tod POD", group="Demo")
@Disabled
public class SpeedTODPOV extends LinearOpMode {
    DcMotor leftSide;
    DcMotor rightSide;

    double left;
    double right;
    double max;

    @Override
    public void runOpMode() throws InterruptedException {
        //leftSide = hardwareMap.dcMotor.get("lw");
        //rightSide = hardwareMap.dcMotor.get("rw");
        //rightSide.setDirection(DcMotor.Direction.REVERSE);

        //leftSide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightSide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSide = hardwareMap.dcMotor.get("left");
        rightSide = hardwareMap.dcMotor.get("right");


        waitForStart();

        //rightSide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(opModeIsActive()) {

            left  = gamepad1.left_stick_y + gamepad1.right_stick_x;
            right = gamepad1.left_stick_y - gamepad1.right_stick_x;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            leftSide.setPower(-left);
            rightSide.setPower(right);

        }
    }
}