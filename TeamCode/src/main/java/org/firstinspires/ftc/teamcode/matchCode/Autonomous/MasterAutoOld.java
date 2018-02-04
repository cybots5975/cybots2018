package org.firstinspires.ftc.teamcode.matchCode.Autonomous;

import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.AutoTransitioner;

import java.util.Objects;

/**
 * Created by kskrueger on 10/22/17.
 */

@Autonomous(name="Master Auto", group="League Champ")
public class MasterAuto extends LinearOpMode{
    private RelicRecoveryVuMark VuMark;
    private int encoderCounts;
    private Robot robot = new Robot(this);
    private boolean loop = true;

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
            if (robot.VuMark1.scan().equals(RelicRecoveryVuMark.UNKNOWN)) {
                VuMark = RelicRecoveryVuMark.CENTER;
            } else {
                VuMark = robot.VuMark1.scan();
            }
            telemetry.addData("Status","Initialized");
            telemetry.addData("Position",robot.prefs.read("position"));
            telemetry.addData("Mode",robot.prefs.read("testMode"));
            telemetry.update();
        }

        robot.VuMark1.close();
        robot.jewelVision.enable();

        if (Objects.equals(robot.prefs.read("testMode"), "false")) {
            AutoTransitioner.transitionOnStop(this, "Teleop V1");
        }
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()&&loop&&!isStopRequested()) {
            telemetry.addData("Position:", robot.prefs.read("postion"));
            telemetry.update();
            robot.JewelKick.setPosition(robot.kickRight);
            robot.pause(3.5);
            robot.jewelOrder = robot.jewelVision.jewelOrder(); //set the current order to the jewelOrder enum
            robot.jewelVision.disable(); //disable the jewel detector after

            flipBallColor(robot.jewelOrder);

            if (Objects.equals(robot.prefs.read("color"), "red")) {
                switch (robot.jewelOrder) {
                    case BLUE_RED:
                        robot.speak("RIGHT BALL OFF!");
                        break;
                    case RED_BLUE:
                        robot.speak("LEFT BALL OFF!");
                        break;
                    case UNKNOWN:
                        if (Objects.equals(robot.prefs.read("testMode"), "true")) {
                            robot.speak("Thinking...thinking...thinking...");
                        } else {
                            robot.speak("Unknown order!");
                        }
                        break;
                }
            }

            scoreJewel(robot.jewelOrder);

            scorePosition();

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
        setVuMarkColumn(-1650,-1400,-1125);

        robot.drive.encoderStrafe(-.25,encoderCounts);
        robot.pause(1);
        robot.intake.setSpeed(-1);
        robot.drive.encoderFwd(.25,300);

        robot.pause(1);
        robot.drive.encoderFwd(-.25,0);

        robot.intake.setSpeed(0);

        robot.pause(.5);
        robot.speak("ALL DONE!.....I'm a good swervy boi.");
        while(!isStopRequested()) {
            //waiting for 30 seconds
        }
    }

    private void redFar(){
        setVuMarkColumn(250,550,950);

        robot.drive.encoderFwd(-.25,-960); //drive backwards off stone

        robot.speak("I'LL BE BACK!");

        robot.drive.gyroTurn(.07,87,1); //turn 90 degrees to the left

        robot.drive.zeroEncoders();
        robot.drive.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.drive.encoderFwd(.25,1000);

        robot.drive.encoderStrafe(.25,encoderCounts);
        switch (VuMark) {
            case LEFT:
                robot.speak("Left Column");
                break;
            case CENTER:
                robot.speak("Center Column");
                break;
            case RIGHT:
                robot.speak("Right Column");
                break;
        }
        robot.pause(1);
        robot.intake.setSpeed(-1);
        robot.drive.zeroEncoders();
        robot.drive.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.drive.encoderFwd(.25,300);
        robot.pause(1);
        robot.drive.encoderFwd(-.25,0);

        robot.intake.setSpeed(0);

        robot.pause(.5);
        robot.speak("ALL DONE!");

        while(!isStopRequested()) {
            //waiting for 30 seconds
        }
    }

    private void blueClose(){
        setVuMarkColumn(1175,1460,1785);

        robot.drive.encoderStrafe(.25,encoderCounts);
        robot.pause(1);
        robot.intake.setSpeed(-1);
        robot.drive.encoderFwd(.25,300);

        robot.pause(1);
        robot.drive.encoderFwd(-.25,0);

        robot.intake.setSpeed(0);
        robot.speak("ALL DONE!.....I'm a good swervy boi.");
        while(!isStopRequested()) {
            //waiting for 30 seconds
        }
    }

    private void blueFar(){
        setVuMarkColumn(-750,-550,-250);

        robot.drive.encoderFwd(-.25,-960); //drive backwards off stone

        robot.drive.gyroTurn(.1,270+2,1); //turn 90 degrees to the right

        robot.drive.zeroEncoders();
        robot.drive.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.drive.encoderFwd(.25,1000);

        robot.drive.zeroEncoders();
        robot.drive.encoderStrafe(-.25,encoderCounts);
        robot.pause(1);
        robot.intake.setSpeed(-1);
        robot.drive.zeroEncoders();
        robot.drive.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.drive.encoderFwd(.25,300);
        robot.pause(1);
        robot.drive.encoderFwd(-.25,0);

        robot.intake.setSpeed(0);
        robot.speak("ALL DONE!.....I'm a good swervy boi.");

        while(!isStopRequested()) {
            //waiting for 30 seconds
        }
    }

    private void scoreJewel(JewelDetector.JewelOrder jewelOrder){
        if (jewelOrder!= JewelDetector.JewelOrder.UNKNOWN) {
            //don't run routine if unknown case
            robot.JewelArm.setPosition(robot.middleArm);
            robot.pause(1.5);
            robot.JewelKick.setPosition(robot.kickCenter);
            robot.pause(.5);
            robot.JewelArm.setPosition(robot.loweredArm);
            robot.pause(.5);
            switch (jewelOrder) {
                case RED_BLUE:
                    robot.JewelKick.setPosition(robot.kickRight);
                    robot.pause(1);
                    break;
                case BLUE_RED:
                    robot.JewelKick.setPosition(robot.kickLeft);
                    robot.pause(1);
                    robot.JewelKick.setPosition(robot.kickCenter);
                    robot.pause(.5);
                    robot.JewelArm.setPosition(robot.middleArm);
                    robot.pause(.5);
                    robot.JewelKick.setPosition(robot.kickRight);
                    robot.pause(.5);
                    break;
                case UNKNOWN:
                    //nothing
                    //better safe than sorry
                    break;
            }
            robot.JewelArm.setPosition(robot.raisedArm);
            robot.pause(1.5);
        }
    }

    private void flipBallColor (JewelDetector.JewelOrder jewelOrder) {
        if (Objects.equals(robot.prefs.read("color"), "blue")) {
            switch (jewelOrder) {
                case RED_BLUE:
                    robot.jewelOrder = JewelDetector.JewelOrder.BLUE_RED;
                    robot.speak("LEFT BALL OFF!");
                    break;
                case BLUE_RED:
                    robot.jewelOrder = JewelDetector.JewelOrder.RED_BLUE;
                    robot.speak("RIGHT BALL OFF!");
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
}
