package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * Created by kskrueger on 10/22/17.
 */

@Autonomous(name="MultiGlyph Test Auto", group="Test")
public class TestMultiLogic extends LinearOpMode{
    private Robot robot = new Robot(this);
    private boolean loop = true;
    private int angle = 0;
    private int distanceCounts = 350;

    private String frontGlyphColor = "", backGlyphColor = "";
    private int frontGlyph = 0, backGlyph = 0;
    private boolean frontGlyphPresent = false, backGlyphPresent = false;
    private int glyphCount = 0;

    @Override
    public void runOpMode() {
        robot.init();
        robot.drive.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.drive.zeroEncoders();
        robot.intake.multiGlyph();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()&&loop&&!isStopRequested()) {
            //go into pit
            robot.drive.zeroEncoders();
            robot.intake.setSpeed(1);
            while (!glyphsIn()&&!isStopRequested()&&opModeIsActive()) {
                while (!jammed(4500)&&!isStopRequested()&&opModeIsActive()) {
                    robot.drive.encoderFwd(.15,300,0,30);
                    robot.pause(1);
                    robot.drive.encoderFwd(.15,50,0,30);
                    robot.pause(1);
                }
                robot.drive.robotCentric(0,0,0);
                robot.intake.setSpeed(-1);
                robot.pause(.5);
            }
            robot.drive.robotCentric(0,0,0);
            robot.glyphMech.grab();
            robot.pause(.5);
            robot.intake.setSpeed(-.5);

            robot.drive.encoderFwd(.2,-300,0,50);
//            loop = false;
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
            if (jammed(2500)) {
                stallTime.reset();
                //switch state to jammed state
                //or reverse intake and such right here
            } else {
                //count number of glyphs
                if (frontGlyph(4, 20) && backGlyph(4, 20)) {
                    //do stuff for glyph count 2 here
                    //program will exit after the glyphCount is set to 2
                    glyphCount = 2;

                } else {
                    if (frontGlyph(4, 20) || backGlyph(4, 20)) {
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
        return (robot.glyphDistance1.getDistance(DistanceUnit.CM) >= minDistance
                && robot.glyphDistance1.getDistance(DistanceUnit.CM) <= maxDistance);
    }

    private boolean backGlyph(int minDistance, int maxDistance) {
        int sensorValueHere = 0;
        return (robot.glyphDistance2.getDistance(DistanceUnit.CM) >= minDistance
                && robot.glyphDistance2.getDistance(DistanceUnit.CM) <= maxDistance);
    }

    private boolean jammed(int jamCurrentDraw) {
        return (robot.IntakeMotor.getCurrentDraw() > jamCurrentDraw);
    }

    private boolean glyphsIn () {
        if ((robot.glyphDistance1.getDistance(DistanceUnit.CM) >= 4
                && robot.glyphDistance1.getDistance(DistanceUnit.CM) <= 20)&&

                (robot.glyphDistance3.getDistance(DistanceUnit.CM) >= 4
                        && robot.glyphDistance3.getDistance(DistanceUnit.CM) <= 20)) {

            if (((robot.glyphColor1.alpha()+robot.glyphColor3.alpha())/2)>75) {
                frontGlyphColor = "Grey";
                frontGlyph = 1;
                frontGlyphPresent = true;
            } else {
                frontGlyphColor = "Brown";
                frontGlyph = 2;
                frontGlyphPresent = true;
            }
        } else {
            frontGlyphColor = "None";
            frontGlyph = 3;
            frontGlyphPresent = false;
        }

        if ((robot.glyphDistance2.getDistance(DistanceUnit.CM) >= 4
                && robot.glyphDistance2.getDistance(DistanceUnit.CM) <= 40)&&

                (robot.glyphDistance4.getDistance(DistanceUnit.CM) >= 4
                        && robot.glyphDistance4.getDistance(DistanceUnit.CM) <= 40)){

            if (((robot.glyphColor2.alpha()+robot.glyphColor4.alpha())/2)>47) {
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

        telemetry.addData("Order (front, back): ",frontGlyph+", "+backGlyph);

        if (frontGlyphPresent&&backGlyphPresent) {
            glyphCount = 2;
            robot.intake.setSpeed(0);
        } else if (frontGlyphPresent||backGlyphPresent) {
            glyphCount = 1;
        } else {
            glyphCount = 0;
        }

        telemetry.update();

        if (glyphCount==2) {
            return true;
        } else {
            return false;
        }
    }

    public void countGlyphs() {
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                while(!isStopRequested()) {
                    if ((robot.glyphDistance1.getDistance(DistanceUnit.CM) >= 4
                            && robot.glyphDistance1.getDistance(DistanceUnit.CM) <= 20)||

                            (robot.glyphDistance3.getDistance(DistanceUnit.CM) >= 4
                                    && robot.glyphDistance3.getDistance(DistanceUnit.CM) <= 20)) {

                        if (((robot.glyphColor1.alpha()+robot.glyphColor3.alpha())/2)>50) {
                            frontGlyphColor = "Grey";
                            frontGlyph = 1;
                            frontGlyphPresent = true;
                        } else {
                            frontGlyphColor = "Brown";
                            frontGlyph = 2;
                            frontGlyphPresent = true;
                        }
                    } else {
                        frontGlyphColor = "None";
                        frontGlyph = 3;
                        frontGlyphPresent = false;
                    }

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

                    telemetry.addData("Order (front, back): ",frontGlyph+", "+backGlyph);

                    if (frontGlyphPresent&&backGlyphPresent) {
                        glyphCount = 2;
                        robot.intake.setSpeed(0);
                    } else if (frontGlyphPresent||backGlyphPresent) {
                        glyphCount = 1;
                    } else {
                        glyphCount = 0;
                    }

                    telemetry.update();

                }
            }
        }).start();
    }

}


/*

telemetry.addData("Position:", robot.prefs.read("postion"));

            telemetry.update();
            robot.intake.setSpeed(1);
            robot.intake.multiGlyph();
            while ((!frontGlyph(2,20))
                    &&!jammed(3000)
                    &&!isStopRequested()) {
                robot.drive.encoderFwd(.3, distanceCounts);
                robot.pause(.25);
                robot.drive.encoderFwd(-.4, 0);
                angle += 7;
                distanceCounts += 100;
                robot.drive.gyroTurn(.55,angle,1);
                telemetry.addData("Back Sensor",robot.glyphDistance2.getDistance(DistanceUnit.CM));
                telemetry.addData("Front Sensor",robot.glyphDistance1.getDistance(DistanceUnit.CM));
                telemetry.addData("Intake Current Draw mA",robot.IntakeMotor.getCurrentDraw());
                telemetry.update();
            }
            if (jammed(3000)) {
                robot.intake.setSpeed(-1);
                robot.pause(.5);
                robot.intake.setSpeed(1);
            }
            while (glyphCount!=2
                    &&!jammed(2500)
                    &&!isStopRequested()) {
                    robot.drive.encoderFwd(.3, distanceCounts);
                    robot.pause(.75);
                    robot.drive.zeroEncoders();
                    robot.drive.encoderFwd(-.4, -200);
                    //angle += 7;
                    distanceCounts += 100;
                    //robot.drive.gyroTurn(.55,angle,1);
                    telemetry.addData("Intake Current Draw mA",robot.IntakeMotor.getCurrentDraw());
                    telemetry.update();
                    }
                    //CYCLE BACK TO REPEAT THIS STEP
                    robot.pause(.5);
                    robot.intake.setSpeed(0);
                    robot.drive.gyroTurn(.3,0,1);*/
