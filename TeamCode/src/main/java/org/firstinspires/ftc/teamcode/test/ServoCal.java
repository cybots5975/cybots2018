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
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name="Position Track Test", group="Test")
//@Disabled
public class ServoCal extends LinearOpMode {
    Robot robot = new Robot(this);

    double leftPosition = .5, rightPosition = .5;

    @Override
    public void runOpMode() {
        robot.init();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.positionTracking.startTracking();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            if (gamepad1.dpad_up) {
                rightPosition += .001;
            } else if (gamepad1.dpad_down) {
                rightPosition -= .001;
            } else if (gamepad1.dpad_left) {
                leftPosition += .001;
            } else if (gamepad1.dpad_right) {
                leftPosition -= .001;
            }

            robot.positionTracking.xWheelDown = leftPosition;
            robot.positionTracking.yWheelDown = rightPosition;

            robot.positionTracking.xWheelUp = leftPosition;
            robot.positionTracking.yWheelUp = rightPosition;

            if (gamepad1.a) {
                robot.positionTracking.wheelsUp();
            } else {
                robot.positionTracking.wheelsDown();
            }

            telemetry.addData("X Position",leftPosition);
            telemetry.addData("Y Position",rightPosition);
            telemetry.addData("abs Y pos",robot.positionTracking.yPosition());
            telemetry.addData("abs X pos",robot.positionTracking.xPosition());

            telemetry.addData("x inch",robot.positionTracking.xPosition(DistanceUnit.INCH));
            telemetry.addData("y inch",robot.positionTracking.yPosition(DistanceUnit.INCH));

            telemetry.update();
        }
    }
}
