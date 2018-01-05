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
            robot.pause(4);
            robot.jewelOrder = robot.jewelVision.jewelOrder(); //set the current order to the jewelOrder enum
            robot.jewelVision.disable(); //disable the jewel detector after

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
        setVuMarkColumn(-1785,-1400,-1160);

        robot.drive.encoderStrafe(-.25,encoderCounts);
        robot.pause(1);
        robot.intake.setSpeed(-1);
        robot.drive.encoderFwd(.25,300);

        robot.pause(1);
        robot.drive.encoderFwd(-.25,0);

        robot.intake.setSpeed(0);
    }

    private void redFar(){
        setVuMarkColumn(325,700,1025);

        robot.drive.encoderFwd(-.25,-960); //drive backwards off stone

        robot.drive.gyroTurn(.1,90,1); //turn 90 degrees to the left

        robot.drive.zeroEncoders();
        robot.drive.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.drive.encoderFwd(.25,1000);

        robot.drive.encoderStrafe(.25,encoderCounts);
        robot.pause(1);
        robot.intake.setSpeed(-1);
        robot.drive.zeroEncoders();
        robot.drive.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.drive.encoderFwd(.25,300);
        robot.pause(1);
        robot.drive.encoderFwd(-.25,0);

        robot.intake.setSpeed(0);
    }

    private void blueClose(){
        setVuMarkColumn(1060,1460,1785);

        robot.drive.encoderStrafe(.25,encoderCounts);
        robot.pause(1);
        robot.intake.setSpeed(-1);
        robot.drive.encoderFwd(.25,300);

        robot.pause(1);
        robot.drive.encoderFwd(-.25,0);

        robot.intake.setSpeed(0);
    }

    private void blueFar(){
        setVuMarkColumn(-325,-700,-1025);

        robot.drive.encoderFwd(-.25,-960); //drive backwards off stone

        robot.drive.gyroTurn(-.1,-90,1); //turn 90 degrees to the right

        robot.drive.zeroEncoders();
        robot.drive.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.drive.encoderFwd(.25,1000);

        robot.drive.encoderStrafe(-.25,encoderCounts);
        robot.pause(1);
        robot.intake.setSpeed(-1);
        robot.drive.zeroEncoders();
        robot.drive.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.drive.encoderFwd(.25,300);
        robot.pause(1);
        robot.drive.encoderFwd(-.25,0);

        robot.intake.setSpeed(0);
    }

    private void scoreJewel(JewelDetector.JewelOrder jewelOrder){
        robot.JewelArm.setPosition(robot.middleArm);
        robot.pause(2);
        robot.JewelKick.setPosition(robot.kickCenter);
        robot.pause(1.5);
        robot.JewelArm.setPosition(robot.loweredArm);
        robot.pause(2);
        switch (jewelOrder) {
            case RED_BLUE:
                robot.JewelKick.setPosition(robot.kickRight);
                robot.pause(1.5);
                break;
            case BLUE_RED:
                robot.JewelKick.setPosition(robot.kickLeft);
                robot.pause(1.5);
                robot.JewelKick.setPosition(robot.kickRight);
                robot.pause(.75);
                break;
            case UNKNOWN:
                //nothing
                //better be safe than sorry
                break;
        }
        robot.JewelArm.setPosition(robot.raisedArm);
        robot.pause(1.5);
    }

    private void setVuMarkColumn(int left, int center, int right){
        switch (VuMark) {
            case LEFT:
                encoderCounts = left;
                break;
            case CENTER:
                encoderCounts = center;
                break;
            case RIGHT:
                encoderCounts = right;
                break;
        }
    }
}
