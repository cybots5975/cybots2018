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

@Autonomous(name="Red Far V1", group="Mecanum")
//@Disabled
public class MecanumAutoRedFarV1 extends LinearOpMode {
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

            robot.pause(2,isStopRequested());

            //jewelsOrder = jewelsOrder.blueFirst;
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

            robot.pause(1.5,isStopRequested());

            switch (VuMark) {
                case LEFT:
                    encoderCounts = 700+325;
                    break;
                case CENTER:
                    encoderCounts = 700;
                    break;
                case RIGHT:
                    encoderCounts = 700-325;
                    break;
            }

            robot.drive.encoderFwd(-.25,-960,isStopRequested()); //drive backwards off stone

            gyroTurn(.1,90,1); //turn 90 degrees to the left

            robot.drive.zeroEncoders();
            robot.drive.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.drive.encoderFwd(.25,1000,isStopRequested());

            robot.drive.encoderStrafe(.25,encoderCounts,isStopRequested());
            robot.pause(1,isStopRequested());
            robot.intake.setSpeed(-1);
            robot.drive.zeroEncoders();
            robot.drive.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.drive.encoderFwd(.25,300,isStopRequested());
            robot.pause(1,isStopRequested());
            robot.drive.encoderFwd(-.25,0,isStopRequested());

            robot.intake.setSpeed(0);

            blueVision.setShowBlue(false);
            blueVision.disable();

            loop = false;
        }
    }

    private double kP = .1, kD = .08, kI = 0;
    private double dt, lastTime;
    public void gyroTurn (double turnSpeed, int targetAngle, int error) {
        while (Math.abs(robot.drive.getAvgHeading()-targetAngle)>error) {

            dt = (System.currentTimeMillis() - lastTime);

            double pidOffset = robot.drive.PID(kP, kI, kD, 10, targetAngle, (int) robot.drive.getAvgHeading());

            double power = -pidOffset * turnSpeed;
            //set power to motors
            robot.drive.driveMecanum(0, 0, power);
            lastTime = System.currentTimeMillis();
            sleep(10);
        }
        robot.drive.driveMecanum(0,0,0);
    }
}
