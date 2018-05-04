package org.firstinspires.ftc.teamcode.matchCode.Autonomous;

import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.subsystems.GlyphMech;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.AutoTransitioner;
import org.firstinspires.ftc.teamcode.util.multiGlyph.cipher.CipherMatch;

import java.util.Objects;

import static org.firstinspires.ftc.teamcode.subsystems.GlyphMech.height.LOW;
import static org.firstinspires.ftc.teamcode.subsystems.GlyphMech.height.STORE;

/**
 * Created by kskrueger on 10/22/17.
 */

@Autonomous(name="Master Auto", group="State")
public class MasterAuto extends LinearOpMode{
    private RelicRecoveryVuMark VuMark;
    private int encoderCounts;
    private int turnAngle, turnAngleSearch;
    private int distanceSearch;
    private Robot robot = new Robot(this);
    private boolean loop = true;
    public ElapsedTime runtime  = new ElapsedTime();
    private ElapsedTime jamTime  = new ElapsedTime();
    private boolean intakeJammed = false;

    private ElapsedTime fullTime  = new ElapsedTime();
    private boolean firstFull = false;
    private boolean glyphCountOn = false;

    //multi glyph vars

    private String frontGlyphColor = "", backGlyphColor = "";
    private int frontGlyph = 0, backGlyph = 0;
    private boolean frontGlyphPresent = false, backGlyphPresent = false;
    private int glyphCount = 0;
    private boolean first = false;
    private double intakeCurrentDraw = 0;
    private boolean intakeOn = false;

    private static CipherMatch cipherTest = new CipherMatch();

    private boolean tryFor5 = true;

    @Override
    public void runOpMode() {
        robot.Vuforia = true;
        robot.JewelVision = true;
        robot.init();
        robot.drive.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.drive.zeroEncoders();
        robot.oldTracking.wheelsUp();


        robot.VuMark1.activate();

        while (!isStarted()&&!isStopRequested()) {
            telemetry.addData("VuMark",robot.VuMark1.scan().toString());
            VuMark = robot.VuMark1.scan();
            robot.jewelVision.vumarkFrames(robot.VuMark1.capture()); //run the vuforia frames through the jewel detector
            robot.jewelOrder = robot.jewelVision.jewelOrder(); //set the current order to the jewelOrder enum
            telemetry.addData("Jewel Order",robot.jewelOrder);
            telemetry.addData("Status","Initialized");
            telemetry.addData("Position",robot.prefs.read("position"));
            telemetry.addData("Mode",robot.prefs.read("testMode"));
            telemetry.update();
        }

        robot.VuMark1.close();

        if (Objects.equals(robot.prefs.read("testMode"), "false")) {
            AutoTransitioner.transitionOnStop(this, "Teleop V1");
        }
        waitForStart();

        while (opModeIsActive()&&loop&&!isStopRequested()) {
            if (VuMark.equals(RelicRecoveryVuMark.UNKNOWN)) {
                VuMark = RelicRecoveryVuMark.CENTER;
            }
            robot.intake.setSpeed(1);
            runtime.reset();
            robot.glyphMech.setDumpSpeed(1);
            robot.glyphMech.grab();
            robot.pause(.1);
            flipBallColor(robot.jewelOrder);

            //lift box INSIDE of scoreJewel code

            robot.intake.auton();

            scoreJewel(robot.jewelOrder);

            robot.intake.setSpeed(0);
            //testMode();
            scorePosition();

            robot.glyphMech.disable();

            robot.speak("ALL DONE!");
            while(!isStopRequested()&&opModeIsActive()) {
                robot.prefs.save("angle",Double.toString(robot.imu.getHeading()));
                //waiting for 30 seconds
            }

            loop = false;
        }
        robot.stop();
        robot.drive.logging.log("autonLOGGINGGGGGGGG");
        telemetry.addData("Written","");
        telemetry.update();
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

    private void testMode() {
        glyphColors();
        cipherTest.vumarkGlyph(backGlyph,1);

        switch (1) {
            case 0:
                robot.drive.gyroTurn(.07,360-30,3);
                break;
            case 1:
                robot.drive.gyroTurn(.07,0,3);
                break;
            case 2:
                robot.drive.gyroTurn(.07,30,3);
                break;
        }

        robot.glyphMech.setPosition(LOW);
        while (!robot.glyphMech.inPosition()&&opModeIsActive()&&!isStopRequested()) {/*wait*/}
        robot.glyphMech.drop();
        robot.pause(.25);
        robot.drive.zeroEncoders();
        robot.drive.encoderFwd(.1,200);

        robot.intake.setSpeed(1);

        robot.glyphMech.setPosition(STORE);
        while (!robot.glyphMech.inPosition()&&opModeIsActive()&&!isStopRequested()) {/*wait*/}

        robot.drive.encoderFwd(.7,450,0);
        intakeCurrentDraw = robot.IntakeMotor.getCurrentDraw();
        intakeWithCurrent(intakeCurrentDraw+2250);
        monitorGlyphCount();
        robot.drive.zeroEncoders();

        while (!glyphFull()&&runtime.seconds()<22&&opModeIsActive()&&!isStopRequested()) {
            robot.drive.encoderFwd(.15,300,0); //goes 200 more than previously
            robot.pause(1);
            robot.drive.encoderFwd(-.5,-50,0);
        }

        robot.glyphMech.grab();
        robot.drive.encoderFwd(-.6,-275,0);
        /*glyphColors();
        telemetry.addData("Order (front, back): ",frontGlyph+", "+backGlyph);
        telemetry.update();*/
        robot.pause(.25);
        glyphColors();
        cipherTest.setHopperGlyphs(backGlyph,frontGlyph);
        cipherTest.search(cipherTest.hopper);
        telemetry.addData("Order (front, back): ",frontGlyph+", "+backGlyph);
        telemetry.addData("Selected Cipher",cipherTest.cipherNumber(cipherTest.cipherChosen));
        telemetry.addData("Column",cipherTest.getColumn());
        telemetry.addData("Height",cipherTest.getHeight());
        telemetry.update();
        switch (cipherTest.getColumn()) {
            case 0:
                robot.drive.gyroTurn(.07,360-30,3);
                break;
            case 1:
                robot.drive.gyroTurn(.07,0,3);
                break;
            case 2:
                robot.drive.gyroTurn(.07,30,3);
                break;
        }

        robot.glyphMech.setPosition(cipherTest.getHeight());
        while (!robot.glyphMech.inPosition()&&opModeIsActive()&&!isStopRequested()) {
            //wait
        }
        robot.glyphMech.drop();
        robot.pause(.5);
        robot.drive.encoderFwd(.1,200);
        robot.glyphMech.setPosition(STORE);
    }

    private void redClose(){
        robot.drive.zeroEncoders();
        setVuMarkColumn(1500,1175,1500);
        robot.glyphMech.grab();
        robot.pause(.1);
        robot.glyphMech.setPosition(GlyphMech.height.STONE);

        robot.drive.encoderStrafe(.5,encoderCounts,0);

        setColumnAngle(24,32,360-25);
        robot.drive.gyroTurn(.25,turnAngle,3);

        placeGlyph();

        robot.intake.multiGlyph();

        robot.drive.gyroTurn(.25,0,3);

        robot.intake.setSpeed(1);

        robot.drive.zeroEncoders();
        robot.drive.encoderFwd(.7,450,0);
        robot.drive.zeroEncoders();

        turnAngle = 0;
        intakeSequence(19);
        robot.glyphMech.setPosition(LOW);

        robot.drive.encoderFwd(-.4,-400,0);
        robot.glyphMech.grab();
        robot.pause(.25);
        robot.glyphMech.setPosition(LOW);

        setColumnAngle(360-18,18,25);
        robot.drive.gyroTurn(.25,turnAngle,3);
        placeGlyph2();

        if (runtime.seconds()<28) {
            robot.drive.zeroEncoders();
            robot.drive.encoderFwd(-.5, -500);
            robot.drive.zeroEncoders();
            robot.drive.encoderFwd(.5, 200);
        }

        if (tryFor5&&runtime.seconds()<22&&!VuMark.equals(RelicRecoveryVuMark.CENTER)) {
            robot.drive.gyroTurn(.25,0,3);
            robot.intake.setSpeed(1);
            robot.drive.zeroEncoders();
            robot.drive.encoderFwd(.7,450,0);
            robot.drive.zeroEncoders();

            turnAngle = 0;
            intakeSequence(25);
            robot.glyphMech.grab();
            robot.pause(.2);
            robot.intake.setSpeed(-.5);
            robot.glyphMech.setPosition(LOW);
            robot.drive.encoderFwd(-.9,-250);

            if (runtime.seconds()<28) {
                robot.drive.zeroEncoders();
                robot.drive.encoderFwd(-.5, -400);
                robot.drive.zeroEncoders();
                robot.drive.encoderFwd(.5, 150);
            }
        }
    }

    private void redFar(){
        robot.glyphMech.grab();
        glyphColors();
        cipherTest.vumarkGlyph(backGlyph,getVumarkColumn());
        robot.pause(.1);
        robot.glyphMech.setPosition(GlyphMech.height.STONE);
        robot.drive.encoderStrafe(.4,1025,0);

        robot.drive.gyroTurn(.15,90-2,3); //turn 90 degrees to the right

        setVuMarkColumn(1100,800,550);
        robot.drive.zeroEncoders();
        robot.drive.encoderStrafe(.5,encoderCounts,90-2);

        robot.pause(.25);

        setColumnAngle(65,60,60);
        robot.drive.gyroTurn(.2,turnAngle,3);

        placeGlyph();

        robot.intake.multiGlyph();

        setColumnAngle(70,55,55);
        robot.drive.gyroTurn(.25,turnAngle,3);

        robot.intake.setSpeed(1);

        setColumnAngle(70,63,55);
        setVuMarkColumn(800,1050,1050);
        robot.drive.encoderFwd(.9,encoderCounts,turnAngle);

       intakeSequence(21);

        robot.glyphMech.grab();
        setColumnAngle(40,turnAngleSearch,turnAngleSearch);
        robot.drive.gyroTurn(.2,turnAngle,3);
        robot.pause(.5);
        robot.glyphMech.setPosition(LOW);
        setVuMarkColumn(-875,-750,-775);
        setColumnAngle(40,70,80);
        robot.drive.encoderFwd(-.6,encoderCounts,turnAngle);
        robot.intake.auton();

        setColumnAngle(55,75,80);
        robot.drive.gyroTurn(.2,turnAngle,3);

        placeGlyph2();

        robot.drive.zeroEncoders();
        if (runtime.seconds()<27) {
            robot.drive.encoderFwd(-.4,-600);
            robot.drive.zeroEncoders();
            robot.drive.encoderFwd(.5,150);
        }
    }

    private void blueClose(){
        robot.glyphMech.grab();

        robot.drive.zeroEncoders();
        setVuMarkColumn(-1450,-1150,-1450);
        robot.drive.encoderStrafe(-.35,encoderCounts);

        setColumnAngle(30,340,340);
        robot.drive.gyroTurn(.25,turnAngle,3);

        placeGlyph();

        telemetry.addData("1st glyph","placed");
        telemetry.update();

        robot.intake.multiGlyph();

        turnAngle = 0;
        robot.drive.gyroTurn(.25,0,3);

        robot.intake.setSpeed(1);
        telemetry.addData("Intake On","");
        telemetry.update();
        robot.drive.zeroEncoders();
        telemetry.addData("Encoders reset","");
        telemetry.update();
        robot.drive.encoderFwd(.7,550,0);
        telemetry.addData("Drive forward completed","");
        telemetry.update();

        intakeSequence(19);

        telemetry.addData("Intake sequence","completed");
        telemetry.update();

        robot.pause(.5);

        robot.glyphMech.setPosition(LOW);

        robot.drive.encoderFwd(-.6,-425);

        //robot.drive.gyroTurn(.1,20,1);
        setColumnAngle(360-10,7,25);
        robot.drive.gyroTurn(.25,turnAngle,3);

        placeGlyph2();

        robot.drive.zeroEncoders();
        if (runtime.seconds()<28) {
            robot.drive.encoderFwd(-.7,-600);
            robot.drive.zeroEncoders();
            robot.drive.encoderFwd(.6,150);
        }

        if (tryFor5&&runtime.seconds()<22&&!VuMark.equals(RelicRecoveryVuMark.CENTER)) {
            robot.drive.gyroTurn(.25,0,3);
            robot.intake.setSpeed(1);
            robot.drive.zeroEncoders();
            robot.drive.encoderFwd(.7,450,0);
            robot.drive.zeroEncoders();

            turnAngle = 0;
            intakeSequence(25);
            robot.glyphMech.grab();
            robot.pause(.2);
            robot.intake.setSpeed(-.5);
            robot.glyphMech.setPosition(LOW);
            robot.drive.encoderFwd(-.9,-250);

            if (runtime.seconds()<28) {
                robot.drive.zeroEncoders();
                robot.drive.encoderFwd(-.6, -400);
                robot.drive.zeroEncoders();
                robot.drive.encoderFwd(.6, 150);
            }
        }
    }

    private void blueFar() {
        robot.glyphMech.grab();
        glyphColors();
        cipherTest.vumarkGlyph(backGlyph,getVumarkColumn());
        robot.pause(.1);
        robot.glyphMech.setPosition(GlyphMech.height.STONE);
        robot.drive.encoderStrafe(-.4,-925,0);

        robot.drive.gyroTurn(.15,-90,2); //turn 90 degrees to the right

        setVuMarkColumn1(-1025,-800,-450);
        robot.drive.zeroEncoders();
        robot.drive.encoderStrafe(-.5,encoderCounts,-90-2);

        robot.pause(.25);

        setColumnAngle1(-65,-60,-60);
        robot.drive.gyroTurn(.2,turnAngle,2);

        placeGlyph();

        robot.intake.multiGlyph();

        setColumnAngle1(-70,-55,-55);
        robot.drive.gyroTurn(.25,turnAngle,2);

        robot.intake.setSpeed(1);

        setColumnAngle1(-70,-65,-60);
        setVuMarkColumn1(800,1050,1100);
        robot.drive.encoderFwd(.9,encoderCounts,turnAngle);


        intakeCurrentDraw = robot.IntakeMotor.getCurrentDraw();
        intakeWithCurrent(intakeCurrentDraw+3250);
        monitorGlyphCount();

        robot.drive.zeroEncoders();
        turnAngleSearch = turnAngle;
        distanceSearch = 350;
        moveOnTime(19);
        while (!glyphFull()&&runtime.seconds()<19&&!isStopRequested()&&opModeIsActive()) {
            robot.drive.encoderFwd(.2,distanceSearch,turnAngleSearch); //goes 200 more than previously
            robot.pause(.5);
            robot.drive.encoderFwd(-.4,-100,turnAngleSearch);
            turnAngleSearch -= 3;
            distanceSearch += 100;
            telemetry.addData("Inside loop","");
            telemetry.update();
        }
        telemetry.addData("Exited search loop","");
        telemetry.update();
        robot.glyphMech.grab();
        //glyphColors();
        cipherTest.setHopperGlyphs(backGlyph,frontGlyph);
        cipherTest.search(cipherTest.hopper);
        telemetry.addData("Order (front, back): ",frontGlyph+", "+backGlyph);
        telemetry.addData("Selected Cipher",cipherTest.cipherNumber(cipherTest.cipherChosen));
        telemetry.addData("Column",cipherTest.getColumn());
        telemetry.addData("Height",cipherTest.getHeight());
        telemetry.update();
        robot.glyphMech.grab();
        setColumnAngle1(-40,turnAngleSearch,turnAngleSearch);
        robot.drive.gyroTurn(.2,turnAngle,2);
        robot.pause(.5);
        robot.glyphMech.setPosition(LOW);
        setVuMarkColumn1(-925,-650,-625);
        setColumnAngle1(-50,-70,-80);
        robot.drive.encoderFwd(-.9,encoderCounts,turnAngle);
        robot.intake.setSpeed(0);
        robot.intake.auton();

        setColumnAngle1(-60,-65,-80);
        robot.drive.gyroTurn(.3,turnAngle,1);

        placeGlyph2();

        robot.drive.zeroEncoders();
        if (runtime.seconds()<28) {
            robot.drive.encoderFwd(-.4,-500);
            robot.drive.zeroEncoders();
            robot.drive.encoderFwd(.5,150);
        }
    }

    private void scoreJewel(JewelDetector.JewelOrder jewelOrder){
        if (jewelOrder!= JewelDetector.JewelOrder.UNKNOWN) {
            //don't run routine if unknown case
            robot.JewelKick.setPosition(robot.kickLow);
            robot.JewelArm.setPosition(robot.armLow);
            robot.pause(.6);
            robot.glyphMech.setPosition(GlyphMech.height.STONE);
            switch (jewelOrder) {
                case BLUE_RED:
                    robot.JewelKick.setPosition(robot.kickRight);
                    robot.pause(.4);
                    break;
                case RED_BLUE:
                    robot.JewelKick.setPosition(robot.kickLeft);
                    robot.pause(.5);
                    break;
                case UNKNOWN:
                    //nothing
                    //better safe than sorry
                    break;
            }
            robot.JewelArm.setPosition(robot.armInit);
            robot.pause(.2);
            robot.JewelKick.setPosition(robot.kickRight);
            robot.pause(.2);
        }
    }

    private void flipBallColor (JewelDetector.JewelOrder jewelOrder) {
        if (Objects.equals(robot.prefs.read("color"), "blue")) {
            switch (jewelOrder) {
                case RED_BLUE:
                    robot.jewelOrder = JewelDetector.JewelOrder.BLUE_RED;
                    robot.speak("LEFT BALL OFF!...FOUR!");
                    break;
                case BLUE_RED:
                    robot.jewelOrder = JewelDetector.JewelOrder.RED_BLUE;
                    robot.speak("RIGHT BALL OFF!...FOUR");
                    break;
                case UNKNOWN:
                    if (Objects.equals(robot.prefs.read("testMode"), "true")) {
                        robot.speak("Thinking...thinking...thinking...Unknown order!");
                    } else {
                        robot.speak("Unknown order!");
                    }
                    break;
            }
        } else {
            switch (jewelOrder) {
                case BLUE_RED:
                    robot.speak("LEFT BALL OFF!...FOUR!");
                    break;
                case RED_BLUE:
                    robot.speak("RIGHT BALL OFF!...FOUR!");
                    break;
                case UNKNOWN:
                    if (Objects.equals(robot.prefs.read("testMode"), "true")) {
                        robot.speak("Thinking...thinking...thinking...Unknown order!");
                    } else {
                        robot.speak("Unknown order!");
                    }
                    break;
            }
        }
    }

    private void setVuMarkColumn(int left, int center, int right){
        switch (VuMark) {
            case LEFT:
                encoderCounts = left;
                robot.speak("Going to Left Column");
                break;
            case CENTER:
                encoderCounts = center;
                robot.speak("Going to Center Column");
                break;
            case RIGHT:
                encoderCounts = right;
                robot.speak("Going to Right Column");
                break;
        }
    }

    private void setVuMarkColumn1 (int right, int center, int left){
        switch (VuMark) {
            case LEFT:
                encoderCounts = left;
                robot.speak("Going to Left Column");
                break;
            case CENTER:
                encoderCounts = center;
                robot.speak("Going to Center Column");
                break;
            case RIGHT:
                encoderCounts = right;
                robot.speak("Going to Right Column");
                break;
        }
    }

    private void setColumnAngle(int left, int center, int right){
        switch (VuMark) {
            case LEFT:
                turnAngle = left;
                break;
            case CENTER:
                turnAngle = center;
                break;
            case RIGHT:
                turnAngle = right;
                break;
        }
    }

    private void setColumnAngle1(int right, int center, int left){
        switch (VuMark) {
            case LEFT:
                turnAngle = left;
                break;
            case CENTER:
                turnAngle = center;
                break;
            case RIGHT:
                turnAngle = right;
                break;
        }
    }

    private void placeGlyph(){
        robot.pause(.1);
        robot.glyphMech.setPosition(GlyphMech.height.LOW);
        robot.pause(1.5);
        robot.glyphMech.drop();
        robot.pause(.1);

        robot.drive.zeroEncoders();
        robot.drive.encoderFwd(-.2,-100);
        robot.pause(.25);
        robot.drive.encoderFwd(.2,100);
        robot.glyphMech.setPosition(STORE);
        robot.pause(.75);
        //robot.glyphMech.disable();
    }

    private void placeGlyph2(){
        robot.glyphMech.grab();
        robot.pause(.25);
        robot.glyphMech.setPosition(GlyphMech.height.LOW);
        //robot.pause(1);
        while (!robot.glyphMech.inPosition()&&opModeIsActive()&&!isStopRequested()) {
        }
        robot.glyphMech.drop();
        robot.pause(.1);

        robot.drive.zeroEncoders();
        robot.drive.encoderFwd(-.2,-100);
        robot.pause(.25);
        robot.drive.encoderFwd(.2,100);
        robot.glyphMech.setPosition(STORE);
        robot.pause(.75);
        //robot.glyphMech.disable();
    }

    private void intakeWithCurrent(double maxCurrent) {
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                intakeOn = true;
                while (opModeIsActive()&&!isStopRequested()&&intakeOn) {
                    if (robot.IntakeMotor.getCurrentDraw()<maxCurrent) {
                        robot.intake.setSpeed(1);
                    } else {
                        robot.intake.setSpeed(-.65);
                    }
                }
                robot.intake.setSpeed(0);
            }
        }).start();
    }

    //attempted new intake sequence
    /**private void intakeWithCurrent(double maxCurrent) {
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                intakeOn = true;
                jamTime.reset();
                while (opModeIsActive()&&!isStopRequested()&&intakeOn) {
                    if (robot.IntakeMotor.getCurrentDraw()<maxCurrent) {
                        robot.intake.setSpeed(1);
                        jamTime.reset();
                        intakeJammed = false;
                    } else if (jamTime.seconds()<.75&&intakeJammed){
                        intakeJammed = true;
                        robot.intake.setSpeed(-1);
                    } else {
                        robot.intake.setSpeed(0);
                    }

                }
                robot.intake.setSpeed(0);
            }
        }).start();
    }*/

    private void setIntakeOff() {
        intakeOn = false;
    }

    private void monitorGlyphCount() {
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                glyphCountOn = true;
                while (opModeIsActive()&&!isStopRequested()&&glyphCountOn) {
                    if (glyphFull()&&firstFull) {
                        firstFull = false;
                        fullTime.reset();
                    }

                    if (!glyphFull()) {
                        firstFull = true;
                    }

                    if (glyphFull()&&fullTime.seconds()>.5) {
                        robot.pause(.5);
                        robot.glyphMech.grab();
                        robot.pause(.25);
                        setIntakeOff();
                        robot.pause(.15);
                        robot.intake.setSpeed(-.5);
                        glyphCountOn = false;
                    }
                }
            }
        }).start();
    }

    private boolean glyphFull () {
        return ((robot.glyphDistance2.getDistance(DistanceUnit.CM) >= 4 && robot.glyphDistance2.getDistance(DistanceUnit.CM) <= 40)
                &&
                (robot.glyphDistance4.getDistance(DistanceUnit.CM) >= 4 && robot.glyphDistance4.getDistance(DistanceUnit.CM) <= 40)
                &&
                (robot.glyphDistance1.getDistance(DistanceUnit.CM) >= 4 && robot.glyphDistance1.getDistance(DistanceUnit.CM) <= 20)
                &&
                (robot.glyphDistance3.getDistance(DistanceUnit.CM) >= 4 && robot.glyphDistance3.getDistance(DistanceUnit.CM) <= 20));
    }

    private void glyphColors () {
        if ((robot.glyphDistance2.getDistance(DistanceUnit.CM) >= 4
                && robot.glyphDistance2.getDistance(DistanceUnit.CM) <= 40)&&

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

        if ((robot.glyphDistance1.getDistance(DistanceUnit.CM) >= 4
                && robot.glyphDistance1.getDistance(DistanceUnit.CM) <= 30)&&

                (robot.glyphDistance3.getDistance(DistanceUnit.CM) >= 4
                        && robot.glyphDistance3.getDistance(DistanceUnit.CM) <= 30)) {

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

        telemetry.addData("Order (front, back): ",frontGlyph+", "+backGlyph);
        telemetry.update();
    }

    private void moveOnTime(int timeLimit) {
        new Thread(new Runnable()
        {
            boolean moveOnRun = true;
            @Override
            public void run()
            {
                while (opModeIsActive()&&!isStopRequested()&&moveOnRun) {
                    if (runtime.seconds() < timeLimit) {
                        robot.drive.moveOn = false;
                    } else {
                        robot.drive.moveOn = true;
                        moveOnRun = false;
                        sleep(50);
                        robot.drive.moveOn = false;
                    }
                }
            }
        }).start();
    }

    private void intakeSequence(int timeAllowed) {
        telemetry.addData("Intake Sequence started","");
        intakeCurrentDraw = robot.IntakeMotor.getCurrentDraw();
        intakeWithCurrent(intakeCurrentDraw+3100);
        monitorGlyphCount();

        robot.drive.zeroEncoders();
        turnAngleSearch = turnAngle;
        distanceSearch = 350;
        moveOnTime(timeAllowed);
        while (!glyphFull()&&runtime.seconds()<timeAllowed&&!isStopRequested()&&opModeIsActive()) {
            robot.drive.encoderFwd(.2,distanceSearch,turnAngleSearch); //goes 200 more than previously
            telemetry.update();
            robot.pause(.25);
            robot.drive.encoderFwd(-.4,-50,turnAngleSearch);
            telemetry.update();
            turnAngleSearch -= 4;
            distanceSearch += 100;
        }
        //glyphColors();
        cipherTest.setHopperGlyphs(backGlyph,frontGlyph);
        cipherTest.search(cipherTest.hopper);
        telemetry.addData("Order (front, back): ",frontGlyph+", "+backGlyph);
        telemetry.addData("Selected Cipher",cipherTest.cipherNumber(cipherTest.cipherChosen));
        telemetry.addData("Column",cipherTest.getColumn());
        telemetry.addData("Height",cipherTest.getHeight());
        telemetry.update();
    }

    private int getVumarkColumn() {
        int out = 1;
        switch (VuMark) {
            case LEFT:
                out = 0;
                break;
            case CENTER:
                out = 1;
                break;
            case RIGHT:
                out = 2;
                break;
        }
        return out;
    }



    void smooth () {
        robot.drive.encoderFwd(.5,200,0);
        robot.drive.encoderFwd(.5,500,90);
    }

}