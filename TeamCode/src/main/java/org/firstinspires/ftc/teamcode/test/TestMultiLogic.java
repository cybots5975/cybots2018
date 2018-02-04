package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.AutoTransitioner;

import java.util.Objects;

/**
 * Created by kskrueger on 10/22/17.
 */

@Autonomous(name="MultiGlyph Test Auto", group="Test")
public class TestMultiLogic extends LinearOpMode{
    private Robot robot = new Robot(this);
    private boolean loop = true;

    @Override
    public void runOpMode() {
        robot.init();
        robot.drive.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.drive.zeroEncoders();

        if (Objects.equals(robot.prefs.read("testMode"), "false")) {
            AutoTransitioner.transitionOnStop(this, "Teleop V1");
        }
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()&&loop&&!isStopRequested()) {
            telemetry.addData("Position:", robot.prefs.read("postion"));
            telemetry.update();


            loop = false;
        }
    }

    private void scorePosition() {
        switch (robot.prefs.read("position")) {
            case "RED CLOSE":
                redClose();
                break;
            case "RED FAR":
                redFar();
                break;
            case "BLUE CLOSE":
                blueClose();
                break;
            case "BLUE FAR":
                blueFar();
                break;
        }
    }

    private void redClose(){

    }

    private void redFar(){

    }

    private void blueClose(){

    }

    private void blueFar(){

    }

    private void multiGlyph() {
        int glyphCount = 0;
        ElapsedTime runtime  = new ElapsedTime();
        ElapsedTime stallTime  = new ElapsedTime();

        //run drive code to get to pile here

        while (glyphCount<2 && runtime.seconds()<26) {
            if (jamDetected(1000)) {
                stallTime.reset();
                //switch state to jammed state
                //or reverse intake and such right here
            } else {
                //count number of glyphs
                if (frontGlyph(5, 12) && backGlyph(5, 12)) {
                    //do stuff for glyph count 2 here
                    //program will exit after the glyphCount is set to 2
                    glyphCount = 2;

                } else {
                    if (frontGlyph(5, 12) || backGlyph(5, 12)) {
                        //do glyph count 1 stuff here
                        glyphCount = 1;

                    } else {
                        //glyph count 0 stuff here
                        glyphCount = 0;

                    }
                }
            }

        }
    }

    private boolean frontGlyph(int minDistance, int maxDistance) {
        int sensorValueHere = 0;
        return (sensorValueHere >= minDistance && sensorValueHere <= maxDistance);
    }

    private boolean backGlyph(int minDistance, int maxDistance) {
        int sensorValueHere = 0;
        return (sensorValueHere >= minDistance && sensorValueHere <= maxDistance);
    }

    private boolean jamDetected(int jamCurrentDraw) {
        int currentSensorValueHere = 750; //750 mA when not jammed
        return (currentSensorValueHere < jamCurrentDraw) && (currentSensorValueHere < jamCurrentDraw);
    }

}
