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
package org.firstinspires.ftc.teamcode.General.Vuforia;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EmptyHardware;

@TeleOp(name="Test Vuforia actual", group="Testing")
//@Disabled
public class TestVuforia extends LinearOpMode {

    /* Declare OpMode members. */
    EmptyHardware robot           = new EmptyHardware();   // Use the SwerveV1 hardware file
    //FTCVuforia vuforia;
    FTCVuforia2 vuforia2;
    FTCVuforia vuforia;

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //vuforia = new FTCVuforia();
        vuforia2 = new FTCVuforia2();
        vuforia = new FTCVuforia();

        vuforia2.preOp();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        vuforia2.activate();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("Vuforia X 0", vuforia2.getX(0));
            telemetry.addData("Vuforia Y 0", vuforia2.getY(0));
            telemetry.addData("Vuforia Angle 0", vuforia2.getAngle(0));
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            //robot.waitForTick(40);
        }
    }
}

