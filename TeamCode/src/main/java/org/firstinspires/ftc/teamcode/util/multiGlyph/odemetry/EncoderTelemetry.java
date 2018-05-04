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
package org.firstinspires.ftc.teamcode.util.multiGlyph.odemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.logging.ArrayLogging;

@TeleOp(name="Encoder Telemetry Test", group="Test")
//@Disabled
public class EncoderTelemetry extends LinearOpMode {
    Robot robot = new Robot(this);

    private String frontGlyph = "", backGlyph = "";

    private ArrayLogging log = new ArrayLogging(4,10000);
    public ElapsedTime runtime = new ElapsedTime();
    private int count = 0;

    private double xPosition, yPosition;

    @Override
    public void runOpMode() {
        robot.init();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.oldTracking.startTracking();

        log.storeValue(0, 0, "Count #");
        log.storeValue(1, 0, "Time");
        log.storeValue(2, 0, "Y");
        log.storeValue(3, 0, "X");

        robot.oldTracking.wheelsDown();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            robot.xWheel.setMaxVoltage(robot.revHubDS.read5vMonitor()/1000/2.475);
            robot.yWheel.setMaxVoltage(robot.revHubDS.read5vMonitor()/1000/2.475);


            telemetry.addLine("---Position Tracking---");

            telemetry.addData("Y pos",robot.oldTracking.yPosition());
            telemetry.addData("X pos",robot.oldTracking.xPosition());
            telemetry.addData("X inch",robot.oldTracking.xPosition(DistanceUnit.INCH));
            telemetry.addData("Y inch",robot.oldTracking.yPosition(DistanceUnit.INCH));
            telemetry.addData("Aux power",robot.revHubDS.read5vMonitor()/1000);


            double leftY = Math.signum(-gamepad1.left_stick_y) * Math.pow(-gamepad1.left_stick_y, 2);
            double leftX = Math.signum(-gamepad1.left_stick_x) * Math.pow(-gamepad1.left_stick_x, 2);
            double rightX = Math.signum(gamepad1.right_stick_x) * Math.pow(gamepad1.right_stick_x, 2);

            robot.drive.robotCentric(leftY,leftX,rightX);

            xPosition = robot.oldTracking.xPosition();
            yPosition = robot.oldTracking.yPosition();

            log();

            if (gamepad1.b) {
                log.log("encoderLog");
                telemetry.addData("Written","");
            }

            telemetry.update();
        }

        robot.oldTracking.stopTracking();
    }

    public void log() {
        count += 1;

        log.storeValueInt(0, count, count);
        log.storeValueInt(1, count, runtime.milliseconds());
        log.storeValueInt(2, count,yPosition);
        log.storeValueInt(3, count,xPosition);
    }
}
