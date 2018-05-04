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

import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.Objects;

@TeleOp(name="Jewel Test", group="Test")
//@Disabled
public class JewelTest extends LinearOpMode {
    Robot robot = new Robot(this);

    double armInit = .830,kickInit = .005;
    double armMid = .179, kickMid = .018;
    double armLow = .122, kickLow = .639;
    double kickLeft = .05, kickRight = .99;

    boolean loop = true;

    JewelDetector.JewelOrder jewelOrder = JewelDetector.JewelOrder.UNKNOWN;
    public ElapsedTime runtime  = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init();

        robot.JewelArm.setPosition(armInit);
        robot.JewelKick.setPosition(kickInit);

        while (!isStarted()) {
            telemetry.addData("Select color on the left","");

            if (gamepad1.x) {
                jewelOrder = JewelDetector.JewelOrder.BLUE_RED;
            } else if (gamepad1.b) {
                jewelOrder = JewelDetector.JewelOrder.RED_BLUE;
            }

            telemetry.addData("Color",jewelOrder.toString());
            telemetry.update();
        }
        robot.JewelArm.setPosition(armInit);
        robot.JewelKick.setPosition(kickInit);
        robot.drive.zeroEncoders();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()&&loop) {

            flipBallColor(jewelOrder);

            robot.JewelKick.setPosition(kickLow);
            robot.JewelArm.setPosition(armLow);
            robot.pause(.5);
            switch (jewelOrder) {
                case BLUE_RED:
                    robot.JewelKick.setPosition(kickRight);
                    robot.speak("RIGHT BALL OFF!");
                    robot.pause(.3);
                    break;
                case RED_BLUE:
                    robot.JewelKick.setPosition(kickLeft);
                    robot.speak("LEFT BALL OFF!");
                    robot.pause(.2);
                    break;
                case UNKNOWN:
                    //nothing
                    //better safe than sorry
                    break;
            }
            telemetry.addData("Time",runtime.seconds());
            telemetry.update();
            robot.JewelArm.setPosition(armInit);
            robot.intake.setSpeed(1);
            robot.drive.encoderFwd(.8,500,0);

            robot.positionTracking.xPositionAbs = 0;
            robot.positionTracking.yPositionAbs = 0;

            robot.drive.timeDrive(.8,0,-.2,.5);

            loop = false;
        }

        robot.stop();
    }

    private void flipBallColor (JewelDetector.JewelOrder jewelOrder) {
        if (Objects.equals(robot.prefs.read("color"), "blue")) {
            switch (jewelOrder) {
                case RED_BLUE:
                    robot.jewelOrder = JewelDetector.JewelOrder.BLUE_RED;
                    break;
                case BLUE_RED:
                    robot.jewelOrder = JewelDetector.JewelOrder.RED_BLUE;
                    break;
            }
        }
    }
}
