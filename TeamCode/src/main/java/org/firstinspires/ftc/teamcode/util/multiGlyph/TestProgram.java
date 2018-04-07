package org.firstinspires.ftc.teamcode.util.multiGlyph;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.multiGlyph.cipher.CipherMatch;

/**
 * Created by kskrueger for Cybots Robotics on 3/10/18.
 */

@Autonomous(name="Test Cipher Auton", group="Test")
public class TestProgram extends LinearOpMode{
    private RelicRecoveryVuMark VuMark;
    private static CipherMatch cipherTest = new CipherMatch();

    private Robot robot = new Robot(this);

    private static int[][] load2 = new int[][]{
            {0}, //fake bottom glyph to simulate solid surface below
            {1}, //1st glyph intaked (bottom when placing)
            {1}  //2nd glyph (top when placing)
    };

    private String frontGlyphColor = "", backGlyphColor = "";
    private int frontGlyph = 0, backGlyph = 0;
    private boolean frontGlyphPresent = false, backGlyphPresent = false;
    private int glyphCount = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.Vuforia = true;
        robot.init();

        while (!isStarted()&&!isStopRequested()) {
            telemetry.addData("VuMark",robot.VuMark1.scan().toString());
            VuMark = robot.VuMark1.scan();
            robot.jewelVision.vumarkFrames(robot.VuMark1.capture()); //run the vuforia frames through the jewel detector
            robot.jewelOrder = robot.jewelVision.jewelOrder(); //set the current order to the jewelOrder enum
            telemetry.addData("Status","Initialized");
            telemetry.addData("Position",robot.prefs.read("position"));
            telemetry.addData("Mode",robot.prefs.read("testMode"));
            detectGlyphOne();
            telemetry.addData("Glyph Color",backGlyph);
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()&&!isStopRequested()) {

        }

    }

    private void detectGlyphOne () {
        if ((robot.glyphDistance2.getDistance(DistanceUnit.CM) >= 4
                && robot.glyphDistance2.getDistance(DistanceUnit.CM) <= 40)||

                (robot.glyphDistance4.getDistance(DistanceUnit.CM) >= 4
                        && robot.glyphDistance4.getDistance(DistanceUnit.CM) <= 40)){

            if (((robot.glyphColor2.alpha()+robot.glyphColor4.alpha())/2)>40) {
                backGlyphColor = "Grey";
                backGlyph = 1;
                backGlyphPresent = true;
            } else {
                backGlyphColor = "Brown";
                backGlyph = 2;
                backGlyphPresent = true;
            }
        } else {
            backGlyphColor = "None";
            backGlyph = 3;
            backGlyphPresent = false;
        }
    }
}
