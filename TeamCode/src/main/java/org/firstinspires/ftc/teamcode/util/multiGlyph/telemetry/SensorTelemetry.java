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
package org.firstinspires.ftc.teamcode.util.multiGlyph.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.logging.ArrayLogging;

@TeleOp(name="Auton Sensor Telemetry Test", group="Test")
//@Disabled
public class SensorTelemetry extends LinearOpMode {
    Robot robot = new Robot(this);

    private String frontGlyph = "", backGlyph = "";

    private ArrayLogging log = new ArrayLogging(2,10000);
    public ElapsedTime runtime = new ElapsedTime();
    private int count = 0;

    double previousMilli = 0;

    @Override
    public void runOpMode() {
        robot.init();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.oldTracking.startTracking();
        robot.oldTracking.wheelsUp();

        log.storeValue(0, 0, "Count #");
        log.storeValue(1, 0, "Time");

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double leftY = Math.signum(-gamepad1.left_stick_y) * Math.pow(-gamepad1.left_stick_y, 2);
            double leftX = Math.signum(-gamepad1.left_stick_x) * Math.pow(-gamepad1.left_stick_x, 2);
            double rightX = Math.signum(gamepad1.right_stick_x) * Math.pow(gamepad1.right_stick_x, 2);

            robot.drive.robotCentric(leftY,leftX,rightX);

            if (gamepad1.right_trigger>.1) {
                robot.intake.setSpeed(gamepad1.right_trigger);
            } else {
                robot.intake.setSpeed(0);
            }

            telemetry.addLine("---Position Tracking---");

            telemetry.addData("Y pos",robot.oldTracking.yPosition());
            telemetry.addData("X pos",robot.oldTracking.xPosition());
            telemetry.addData("X inch",robot.oldTracking.xPosition(DistanceUnit.INCH));
            telemetry.addData("Y inch",robot.oldTracking.yPosition(DistanceUnit.INCH));

            telemetry.addLine();

            telemetry.addLine("---Glyph Sensors---");
            if ((robot.glyphDistance1.getDistance(DistanceUnit.CM) >= 4
                    && robot.glyphDistance1.getDistance(DistanceUnit.CM) <= 20)&&

                    (robot.glyphDistance3.getDistance(DistanceUnit.CM) >= 4
                    && robot.glyphDistance3.getDistance(DistanceUnit.CM) <= 20)) {

                if (((robot.glyphColor1.alpha()+robot.glyphColor3.alpha())/2)>75) {
                    frontGlyph = "Grey";
                } else {
                    frontGlyph = "Brown";
                }
            } else {
                frontGlyph = "None";
            }

            if ((robot.glyphDistance2.getDistance(DistanceUnit.CM) >= 4
                    && robot.glyphDistance2.getDistance(DistanceUnit.CM) <= 20)&&

                    (robot.glyphDistance4.getDistance(DistanceUnit.CM) >= 4
                            && robot.glyphDistance4.getDistance(DistanceUnit.CM) <= 20)){

                if (((robot.glyphColor2.alpha()+robot.glyphColor4.alpha())/2)>47) {
                    backGlyph = "Grey";
                } else {
                    backGlyph = "Brown";
                }
            } else {
                backGlyph = "None";
            }

            telemetry.addData("Order (front, back): ",frontGlyph+", "+backGlyph);

            telemetry.addData("Front 1 Distance",robot.glyphDistance1.getDistance(DistanceUnit.CM));
            telemetry.addData("Front 2 Distance",robot.glyphDistance3.getDistance(DistanceUnit.CM));
            telemetry.addData("Back 1 Distance",robot.glyphDistance2.getDistance(DistanceUnit.CM));
            telemetry.addData("Back 2 Distance",robot.glyphDistance4.getDistance(DistanceUnit.CM));
            telemetry.addLine();
            telemetry.addData("Front 1 Alpha",robot.glyphColor1.alpha());
            telemetry.addData("Front 2 Alpha",robot.glyphColor3.alpha());
            telemetry.addData("Back 1 Alpha",robot.glyphColor2.alpha());
            telemetry.addData("Back 2 Alpha",robot.glyphColor4.alpha());

            telemetry.addLine();
                                                                             //Grey;            Brown
                                                                            //Alpha 83-81-99    49-66-68
            telemetry.addData("Front 1 Red",robot.glyphColor1.red()); //30-29-34        19-24-24
            telemetry.addData("Front 1 Blue",robot.glyphColor1.blue()); //26-26-30      15-21-21
            telemetry.addData("Front 1 Green",robot.glyphColor1.blue()); //26-26-30     15-21-21
            telemetry.addData("Front 1 argb",robot.glyphColor1.green()); //30-29-34     18-24-24

            telemetry.addLine();

            telemetry.addData("Back 2 Red",robot.glyphColor4.red());
            telemetry.addData("Back 2 Blue",robot.glyphColor4.blue());
            telemetry.addData("Back 2 Green",robot.glyphColor4.blue());
            telemetry.addData("Back 2 argb",robot.glyphColor4.green());

            telemetry.addLine();

            telemetry.addLine("---Current Draw---");
            telemetry.addData("Intake Current Draw",robot.IntakeMotor.getCurrentDraw());

            previousMilli = runtime.milliseconds();

            log();

            if (gamepad1.b) {
                log.log("monkeySeeRobot");
                telemetry.addData("Written","");
            }

            telemetry.update();
        }
    }

    public void log() {
        count += 1;

        log.storeValueInt(0, count, count);
        log.storeValueInt(1, count, runtime.milliseconds());
    }
}
