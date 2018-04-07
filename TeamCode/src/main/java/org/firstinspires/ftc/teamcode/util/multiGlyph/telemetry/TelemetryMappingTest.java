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

@TeleOp(name="Telemetry Mapping", group="Auto Testing")
public class TelemetryMappingTest extends LinearOpMode {
    String[][] output = new String[24][108];
    String[][] outputOrgin = new String[][] {
            {"|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|"},
            {"|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|"},
            {"|",":",":",":",":",":",":",":",":","|", ""," "," "," "," "," "," "," "," "," "," "," "," "," "," "," ","" ,"|",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":","|", ""," "," "," "," "," "," "," "," "," "," "," "," "," "," "," ","" ,"|",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|"},
            {"|",":",":",":",":",":",":",":",":","|", ""," "," "," "," "," "," "," "," "," "," "," "," "," "," "," ","" ,"|",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":","|", ""," "," "," "," "," "," "," "," "," "," "," "," "," "," "," ","" ,"|",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|"},
            {"|",":",":",":",":",":",":",":",":","|", ""," "," "," "," "," "," "," "," "," "," "," "," "," "," "," ","" ,"|",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":","|", ""," "," "," "," "," "," "," "," "," "," "," "," "," "," "," ","" ,"|",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|"},
            {"|",":",":",":",":",":",":",":",":","|", ""," "," "," "," "," "," "," "," "," "," "," "," "," "," "," ","" ,"|",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":","|", ""," "," "," "," "," "," "," "," "," "," "," "," "," "," "," ","" ,"|",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|"},
            {"|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|"},
            {"|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|"},
            {"|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|"},
            {"|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|"},
            {"|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|"},
            {"|","" ,"" ,"" ,"" ,"" ,"" ,"" ,"" ," ","_","_","_","_","_","_","_","_","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|","" ,"" ,"" ,"" ,"" ,"" ,"" ,"" ," ","_","_","_","_","_","_","_","_","|","" ,"" ,"" ,"" ,"" ,"" ,"" ,"" ," ","_","_","_","_","_","_","_","_","|"},
            {"|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",".",".",".",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|"},
            {"|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",".",".",".",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|"},
            {"|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",".",".","" ,"_","" ,"_","" ,"_","" ,"_","" ,"|","" ,"" ,"" ,"" ,"" ,"" ,"" ,"" ," ","_","_","_","_","_","_","_","_","|","" ,"" ,"" ,"" ,"" ,"" ,"" ,"" ," ","_","_","_","_","_","_","_","_","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|"},
            {"|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|"},
            {"|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|"},
            {"|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|"},
            {"|",":",":",":",":",":",":",":",":","|", ""," "," "," "," "," "," "," "," "," "," "," "," "," "," "," ","" ,"|",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":","|", ""," "," "," "," "," "," "," "," "," "," "," "," "," "," "," ","" ,"|",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|"},
            {"|",":",":",":",":",":",":",":",":","|", ""," "," "," "," "," "," "," "," "," "," "," "," "," "," "," ","" ,"|",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":","|", ""," "," "," "," "," "," "," "," "," "," "," "," "," "," "," ","" ,"|",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|"},
            {"|",":",":",":",":",":",":",":",":","|", ""," "," "," "," "," "," "," "," "," "," "," "," "," "," "," ","" ,"|",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":","|", ""," "," "," "," "," "," "," "," "," "," "," "," "," "," "," ","" ,"|",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|"},
            {"|",":",":",":",":",":",":",":",":","|", ""," "," "," "," "," "," "," "," "," "," "," "," "," "," "," ","" ,"|",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":","|", ""," "," "," "," "," "," "," "," "," "," "," "," "," "," "," ","" ,"|",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|"},
            {"|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|"},
            {"|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":",":","|"}
    };

    String[][] robot = new String[][] {
            {"(","","","","",".",".",".",".",".",".",".",".",".",".",".",".",".",".",".",".",")"},
            {"(","","","","",".",".",".",".",".",".",".",".",".",".",".",".",".",".",".",".",")"},
            {"(","","","","",".",".",".",".",".",".",".",".",".",".",".",".",".",".",".",".",")"}
    };

    int xSize = 0;
    int xVar = 60, yVar = 15;
    boolean leftPrev = false, rightPrev = false, upPrev = false, downPrev;

    @Override
    public void runOpMode() {

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.setMsTransmissionInterval(50);
        telemetry.setItemSeparator("");
        telemetry.setCaptionValueSeparator("");
        correction();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.dpad_right&&!rightPrev) {
                rightPrev = true;
                xVar += 1;
            }
            if (!gamepad1.dpad_right) {
                rightPrev = false;
            }

            if (gamepad1.dpad_left&&!leftPrev) {
                leftPrev = true;
                xVar -= 1;
            }
            if (!gamepad1.dpad_left) {
                leftPrev = false;
            }

            if (gamepad1.dpad_down&&!downPrev) {
                downPrev = true;
                yVar += 1;
            }
            if (!gamepad1.dpad_down) {
                downPrev = false;
            }

            if (gamepad1.dpad_up&&!upPrev) {
                upPrev = true;
                yVar -= 1;
            }
            if (!gamepad1.dpad_up) {
                upPrev = false;
            }

            correction();
            //modify(output,yVar,xVar,":");
            robot1(output,yVar,xVar,")");
            telemetryLoop(output);
        }
    }

    private void modify(String[][] input, int y, int x, String value) {
        input[y][x] = value;
    }

    private void robot1(String[][] input, int yCenter, int xCenter, String value) {
        for (int y = 0; y < robot.length; y++) {
            for (int x = 0; x < robot[0].length; x++) {
                input[yCenter-1+y][xCenter-13+x] = robot[y][x];
            }
        }
    }

    private void robot(String[][] input, int y, int x, String value) {
        input[y+1][x-2] = value;
        input[y][x-2] = value;
        input[y-1][x-2] = value;
        input[y+1][x+2] = value;
        input[y][x+2] = value;
        input[y-1][x+2] = value;
    }

    private void correction() {
        for (int x1 = 0; x1 < 108; x1++) {
            for (int y1 = 0; y1 < 24; y1++) {
                output[y1][x1] = outputOrgin[y1][x1];
            }
        }
    }

    private void telemetryLoop(String[][] input) {
        for (String[] anInput : input) {
            telemetry.addLine(line(anInput));
        }
        telemetry.addData("Size",xSize);
        telemetry.update();
    }

    private String line(String[] input) {
        StringBuilder output = new StringBuilder();
        for (String anInput : input) {
            output.append(anInput);
        }
        xSize = input.length;
        return output.toString();
    }
}
