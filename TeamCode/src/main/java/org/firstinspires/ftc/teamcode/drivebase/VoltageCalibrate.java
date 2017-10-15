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
package org.firstinspires.ftc.teamcode.drivebase;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Old_Swerve.HardwareSwerveV1;
import org.firstinspires.ftc.teamcode.Old_Swerve.SwerveLinearBase;

@TeleOp(name="Voltage Calibrate actual", group="Swerve")
//@Disabled
public class VoltageCalibrate extends SwerveLinearBase {

    /* Declare OpMode members. */
    HardwareSwerveV1 robot           = new HardwareSwerveV1();   // Use the SwerveV1 hardware file

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //Define and initialize ALL installed servos.
        DServo1 = hardwareMap.servo.get("DS1"); //Driver Servo Front(1)
        DServo2 = hardwareMap.servo.get("DS2"); //Driver Servo Back(2)
        PServo1 = hardwareMap.servo.get("PS1"); //Pass Servo Front(1)
        PServo2 = hardwareMap.servo.get("PS2"); //Pass Servo Back(2)*/

        DServo1.setPosition(.5); //Set Driver Servo Front(1) to 0 power
        DServo2.setPosition(.5); //Set Driver Servo Back(2) to 0 power
        PServo1.setPosition(.5); //Set Pass Servo Front(1) to 0 power
        PServo2.setPosition(.5); //Set Pass Servo Back(2) to 0 power

        DSensor1 = hardwareMap.analogInput.get("DSe1");
        DSensor2 = hardwareMap.analogInput.get("DSe2");
        PSensor1 = hardwareMap.analogInput.get("PSe1");
        PSensor2 = hardwareMap.analogInput.get("PSe2");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("DS1",DSensor1.getVoltage());
            telemetry.addData("DS2",DSensor2.getVoltage());
            telemetry.addData("PS1",PSensor1.getVoltage());
            telemetry.addData("PS2",PSensor2.getVoltage());
            telemetry.update();



            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }


}
