package org.firstinspires.ftc.teamcode.matchCode.Autonomous;

import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.subsystems.GlyphMech;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.AutoTransitioner;

import java.util.Objects;

/**
 * Created by kskrueger on 10/22/17.
 */

@Autonomous(name="Master Auto", group="State")
public class MasterAuto extends LinearOpMode{
    private RelicRecoveryVuMark VuMark;
    private int encoderCounts;
    private int turnAngle;
    private Robot robot = new Robot(this);
    private boolean loop = true;
    public ElapsedTime runtime  = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.Vuforia = true;
        robot.JewelVision = true;
        robot.init();
        robot.drive.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.drive.zeroEncoders();

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
            /*if (robot.VuMark1.scan().equals(RelicRecoveryVuMark.UNKNOWN)) {
                VuMark = RelicRecoveryVuMark.CENTER;
            }*/
            runtime.reset();
            robot.glyphMech.setDumpSpeed(.6);
            robot.glyphMech.grab();
            robot.pause(.1);
            robot.glyphMech.setPosition(GlyphMech.height.STONE);
            flipBallColor(robot.jewelOrder);

            robot.intake.auton();

            scoreJewel(robot.jewelOrder);

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
        robot.drive.zeroEncoders();
        setVuMarkColumn(1500,1175,1500);
        robot.glyphMech.grab();
        robot.pause(.1);
        robot.glyphMech.setPosition(GlyphMech.height.STONE);

        robot.drive.encoderStrafe(.6,encoderCounts,0);
        robot.pause(1);

        setColumnAngle(27,30,360-30);
        robot.drive.gyroTurn(.15,turnAngle,1);

        placeGlyph();

        robot.intake.multiGlyph();

        robot.drive.gyroTurn(.25,0,1);

        robot.intake.setSpeed(1);
        robot.drive.zeroEncoders();
        robot.drive.encoderFwd(.7,550,0);
        robot.drive.zeroEncoders();
        robot.drive.encoderFwd(.15,300,0); //goes 200 more than previously
        robot.pause(1.5);
        robot.drive.encoderFwd(-.5,-200,0);
        robot.drive.encoderFwd(.2,200,5);
        robot.pause(1.5);
        robot.drive.encoderFwd(-.6,-475,0);

        setColumnAngle(360-25,11,25);
        robot.drive.gyroTurn(.1,turnAngle,1);
        placeGlyph2();
        robot.drive.zeroEncoders();
        robot.drive.encoderFwd(-.4,-500);
        robot.drive.zeroEncoders();
        robot.drive.encoderFwd(.3,150);
    }

    private void redFar(){
        robot.glyphMech.grab();
        robot.pause(.1);
        robot.glyphMech.setPosition(GlyphMech.height.STONE);
        robot.drive.encoderStrafe(.4,1025,0);

        robot.drive.gyroTurn(.15,90-2,1); //turn 90 degrees to the right

        setVuMarkColumn(550,800,550);
        robot.drive.zeroEncoders();
        robot.drive.encoderStrafe(.5,encoderCounts,90-2);

        robot.pause(.25);

        setColumnAngle(120-5,60,60);
        robot.drive.gyroTurn(.2,turnAngle,1);

        placeGlyph();

        robot.intake.multiGlyph();

        robot.drive.gyroTurn(.25,90-35,1);

        intakeWithCurrent(6500);
        robot.drive.zeroEncoders();

        setColumnAngle(55,70,55);
        robot.drive.encoderFwd(.9,1050,turnAngle);
        robot.drive.zeroEncoders();
        robot.drive.encoderFwd(.2,300,turnAngle); //goes 200 more than previously
        robot.pause(1.25);
        robot.drive.encoderFwd(-.6,-50,turnAngle);
        robot.drive.encoderFwd(.2,350,turnAngle+5);
        robot.pause(1);

        setVuMarkColumn(-800,-1100,-800);
        setColumnAngle(61,70,76);
        robot.drive.encoderFwd(-.9,encoderCounts,turnAngle);
        robot.intake.auton();

        setColumnAngle(62,70,70);
        robot.drive.gyroTurn(.2,turnAngle,1);
        placeGlyph2();
        robot.drive.zeroEncoders();
        if (runtime.seconds()<27) {
            robot.drive.encoderFwd(-.4,-400);
            robot.drive.zeroEncoders();
            robot.drive.encoderFwd(.3,25);
        }
    }

    private void blueClose(){
        robot.glyphMech.grab();

        robot.drive.zeroEncoders();
        setVuMarkColumn(-1650,-1200,-1550);
        robot.drive.encoderStrafe(-.6,encoderCounts);

        setColumnAngle(30,330,330);
        robot.drive.gyroTurn(.1,turnAngle,1);

        placeGlyph();

        robot.intake.multiGlyph();

        robot.drive.gyroTurn(.1,0,1);

        robot.intake.setSpeed(1);
        robot.drive.zeroEncoders();
        robot.drive.encoderFwd(.7,550);
        robot.drive.zeroEncoders();
        robot.drive.encoderFwd(.15,300); //goes 200 more than previously
        robot.pause(1.5);
        robot.drive.encoderFwd(-.5,-200);
        robot.drive.encoderFwd(.2,200);
        robot.pause(1.5);
        robot.drive.encoderFwd(-.6,-475);

        robot.drive.gyroTurn(.1,20,1);
        placeGlyph2();
        robot.drive.zeroEncoders();
        robot.drive.encoderFwd(-.3,-200);
        robot.drive.encoderFwd(.2,100);
    }

    private void blueFar() {
        robot.glyphMech.grab();
        robot.pause(.1);
        robot.glyphMech.setPosition(GlyphMech.height.STONE);
        robot.drive.encoderStrafe(-.4, -950);

        robot.drive.gyroTurn(.15, 270 + 2, 1); //turn 90 degrees to the right

        robot.drive.zeroEncoders();
        setVuMarkColumn(-500, -800, -500);
        robot.pause(.5);
        robot.drive.encoderStrafe(-.5, encoderCounts);

        robot.pause(.25);

        setColumnAngle(360-60,360-60,360-120);
        robot.drive.gyroTurn(.15, turnAngle, 1);

        placeGlyph();

        robot.intake.multiGlyph();

        robot.drive.gyroTurn(.15,295,2);

        intakeWithCurrent(6500);
        robot.drive.zeroEncoders();

        setColumnAngle(295,290,295);
        robot.drive.encoderFwd(.9,1050,turnAngle);
        robot.drive.zeroEncoders();
        robot.drive.encoderFwd(.2,300,turnAngle); //goes 200 more than previously
        robot.pause(1.25);
        robot.drive.encoderFwd(-.6,-50,turnAngle);
        robot.drive.encoderFwd(.2,350,turnAngle-5);
        robot.pause(1);

        setVuMarkColumn(-625,-625,-750);
        setColumnAngle(282,287,305);
        robot.drive.encoderFwd(-.9,encoderCounts,turnAngle);
        robot.intake.auton();

        setColumnAngle(290,290,290);
        robot.drive.gyroTurn(.25,turnAngle,2);

        placeGlyph2();

        robot.drive.zeroEncoders();
        if (runtime.seconds()<27) {
            robot.drive.encoderFwd(-.4,-400);
            robot.drive.zeroEncoders();
            robot.drive.encoderFwd(.4,75);
        } else {
            robot.drive.encoderFwd(-.2,-100);
        }
    }

    private void scoreJewel(JewelDetector.JewelOrder jewelOrder){
        if (jewelOrder!= JewelDetector.JewelOrder.UNKNOWN) {
            //don't run routine if unknown case
            robot.JewelKick.setPosition(robot.kickLow);
            robot.JewelArm.setPosition(robot.armLow);
            robot.pause(1);
            switch (jewelOrder) {
                case BLUE_RED:
                    robot.JewelKick.setPosition(robot.kickRight);
                    robot.pause(.75);
                    break;
                case RED_BLUE:
                    robot.JewelKick.setPosition(robot.kickLeft);
                    robot.pause(.75);
                    break;
                case UNKNOWN:
                    //nothing
                    //better safe than sorry
                    break;
            }
            robot.JewelArm.setPosition(robot.armInit);
            robot.pause(.4);
            robot.JewelKick.setPosition(robot.kickRight);
            robot.pause(.5);
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
        robot.glyphMech.setPosition(GlyphMech.height.STORE);
        robot.pause(.75);
        //robot.glyphMech.disable();
    }

    private void placeGlyph2(){
        robot.glyphMech.grab();
        robot.pause(.25);
        robot.glyphMech.setPosition(GlyphMech.height.LOW);
        robot.pause(1.5);
        robot.glyphMech.drop();
        robot.pause(.1);

        robot.drive.zeroEncoders();
        robot.drive.encoderFwd(-.2,-100);
        robot.pause(.25);
        robot.drive.encoderFwd(.2,100);
        robot.glyphMech.setPosition(GlyphMech.height.STORE);
        robot.pause(.75);
        //robot.glyphMech.disable();
    }

    public void intakeWithCurrent(double maxCurrent) {
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                while (opModeIsActive()&&!isStopRequested()) {
                    if (robot.IntakeMotor.getCurrentDraw()<maxCurrent) {
                        robot.intake.setSpeed(1);
                    } else {
                        robot.intake.setSpeed(-1);
                    }
                }
                robot.intake.setSpeed(0);
            }
        }).start();
    }
}