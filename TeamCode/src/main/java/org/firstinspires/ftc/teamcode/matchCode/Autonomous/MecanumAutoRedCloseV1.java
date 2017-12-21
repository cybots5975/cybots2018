package org.firstinspires.ftc.teamcode.matchCode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.general.Robot;
import org.firstinspires.ftc.teamcode.test.ExampleBlueVision;

import static org.firstinspires.ftc.teamcode.test.ExampleBlueVision.jewelsOrder;
import static org.firstinspires.ftc.teamcode.test.ExampleBlueVision.order.unknown;

/**
 * Created by kskrueger on 10/22/17.
 */

@Autonomous(name="Red Close V1", group="Mecanum")
//@Disabled
public class MecanumAutoRedCloseV1 extends LinearOpMode {
    private RelicRecoveryVuMark VuMark;
    private int encoderCounts;
    private double kickCenter = .45, raisedArm = .02;

    private ExampleBlueVision blueVision;
    private Robot robot = new Robot(); //use the SwerveV1 hardware file to configure
    private boolean loop = true;

    @Override
    public void runOpMode() {
        robot.Vuforia = true;
        robot.init(hardwareMap);
        robot.setOpMode(this);
        robot.drive.zeroEncoders();
        robot.drive.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        //AutoTransitioner.transitionOnStop(this, "Teleop V1");
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()&&loop&&!isStopRequested()) {
            robot.intake.init();
            robot.JewelArm.setPosition(1);
            ExampleBlueVision.jewelsOrder = unknown;
            blueVision = new ExampleBlueVision();
            // can replace with ActivityViewDisplay.getInstance() for fullscreen
            blueVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
            blueVision.setShowBlue(false);
            // start the vision system
            blueVision.enable();

            blueVision.setShowBlue(true);

            robot.pause(2);

            scoreJewel();

            robot.pause(1.5);

            setVuMarkColumn();

            robot.drive.encoderStrafe(-.25,encoderCounts);
            robot.pause(1);
            robot.intake.setSpeed(-1);
            robot.drive.encoderFwd(.25,300);
            robot.pause(1);
            robot.drive.encoderFwd(-.25,0);

            robot.intake.setSpeed(0);

            blueVision.setShowBlue(false);
            blueVision.disable();

            loop = false;
        }
    }

    private void scoreJewel(){
        switch (jewelsOrder) {
            case blueFirst:
                robot.runtime.reset();
                while (robot.runtime.seconds() < 2) {
                    robot.JewelKick.setPosition(0);
                    telemetry.addData("Red jewel","stop");
                    telemetry.update();
                }
                robot.JewelKick.setPosition(kickCenter);
                robot.JewelArm.setPosition(raisedArm);
                break;
            case redFirst:
                robot.runtime.reset();
                while (robot.runtime.seconds() < 2) {
                    robot.JewelKick.setPosition(1);
                    telemetry.addData("Blue jewel","stop");
                    telemetry.update();
                }
                robot.JewelKick.setPosition(kickCenter);
                robot.JewelArm.setPosition(raisedArm);
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
