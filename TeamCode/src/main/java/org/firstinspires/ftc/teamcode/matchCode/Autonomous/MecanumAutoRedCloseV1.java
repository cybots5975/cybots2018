package org.firstinspires.ftc.teamcode.matchCode.Autonomous;

import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.general.Robot;

/**
 * Created by kskrueger on 12/20/17.
 */

@Autonomous(name="Red Close V1", group="Mecanum")
//@Disabled
public class MecanumAutoRedCloseV1 extends LinearOpMode {
    private RelicRecoveryVuMark VuMark;
    private int encoderCounts;
    private double kickCenter = .45, raisedArm = .02;
    private Robot robot = new Robot(this);
    private boolean loop = true;

    @Override
    public void runOpMode() {
        robot.Vuforia = true;
        //robot.init(hardwareMap);
        robot.setOpMode(this);
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
            telemetry.addData("Status", "Initialized");
            telemetry.update();
        }

        robot.VuMark1.close();
        robot.jewelVision.enable();

        //AutoTransitioner.transitionOnStop(this, "Teleop V1");
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()&&loop&&!isStopRequested()) {
            //jewelDetector.enable(); //enable jewel detection
            robot.intake.init(); //initialize robot
            robot.JewelArm.setPosition(1); //set jewel arm to start position
            robot.pause(5);
            robot.jewelOrder = robot.jewelVision.jewelOrder(); //set the current order to the jewelOrder enum
            robot.jewelVision.disable(); //diable the jewel detector after

            robot.pause(.5); //wait .5 seconds to diable, then score the jewel

            scoreJewel(robot.jewelOrder);

            robot.pause(1.5);

            setVuMarkColumn();

            robot.drive.encoderStrafe(-.25,encoderCounts);
            robot.pause(1);
            robot.intake.setSpeed(-1);
            robot.drive.encoderFwd(.25,300);
            robot.pause(1);
            robot.drive.encoderFwd(-.25,0);

            robot.intake.setSpeed(0);

            loop = false;
        }
    }

    private void scoreJewel(JewelDetector.JewelOrder jewelOrder){
        switch (jewelOrder) {
            case RED_BLUE:
                robot.runtime.reset();
                while (robot.runtime.seconds() < 2) {
                    robot.JewelKick.setPosition(0);
                    telemetry.addData("Red jewel","stop");
                    telemetry.update();
                }
                robot.JewelKick.setPosition(kickCenter);
                robot.JewelArm.setPosition(raisedArm);
                break;
            case BLUE_RED:
                robot.runtime.reset();
                while (robot.runtime.seconds() < 2) {
                    robot.JewelKick.setPosition(1);
                    telemetry.addData("Blue jewel","stop");
                    telemetry.update();
                }
                robot.JewelKick.setPosition(kickCenter);
                robot.JewelArm.setPosition(raisedArm);
                break;
            case UNKNOWN:

                break;
        }
    }

    private void setVuMarkColumn(){
        switch (VuMark) {
            case LEFT:
                encoderCounts = -1460-325;
                break;
            case CENTER:
                encoderCounts = -1460;
                break;
            case RIGHT:
                encoderCounts = -1460+400;
                break;
        }
    }
}
