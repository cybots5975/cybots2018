package org.firstinspires.ftc.teamcode.matchCode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.general.AutoTransitioner;
import org.firstinspires.ftc.teamcode.general.Robot;
import org.firstinspires.ftc.teamcode.test.ExampleBlueVision;

import static org.firstinspires.ftc.teamcode.test.ExampleBlueVision.jewelsOrder;
import static org.firstinspires.ftc.teamcode.test.ExampleBlueVision.order.unknown;

/**
 * Created by kskrueger on 10/22/17.
 */

@Autonomous(name="Test Glyph Auto RED", group="Swerve")
public class GlyphAutoV1_opp extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();

    private ExampleBlueVision blueVision;

    private RelicRecoveryVuMark VuMark;
    private double driveTime, distance;
    private double kickCenter = .45, raisedArm = .02;
    private boolean loop = true;


    private Robot robot = new Robot(); //use the SwerveV1 hardware file to configure

    @Override
    public void runOpMode() {
        robot.Vuforia = true;
        robot.init(hardwareMap);

        gamepad1.setJoystickDeadzone(.01F); //Set joystick deadzone to a lower number

        robot.VuMark1.activate();

        while (!isStarted()&&!isStopRequested()) {
            robot.JewelArm.setPosition(raisedArm);
            robot.JewelKick.setPosition(kickCenter);
            robot.drive.setEfficiency(false);
            robot.drive.holdModuleAngle(90+180);
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
        AutoTransitioner.transitionOnStop(this, "Teleop V1");
        waitForStart();
        if (isStarted()) {
            while(opModeIsActive()&&loop&&!isStopRequested()) {
                jewelsOrder = unknown;
                robot.JewelKick.setPosition(kickCenter);
                robot.JewelArm.setPosition(1);
                blueVision = new ExampleBlueVision();
                // can replace with ActivityViewDisplay.getInstance() for fullscreen
                blueVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
                blueVision.setShowBlue(false);
                // start the vision system
                blueVision.enable();

                blueVision.setShowBlue(true);

                pause(4,false);

                //jewelsOrder = jewelsOrder.blueFirst;
                switch (jewelsOrder) {
                    case blueFirst:
                        runtime.reset();
                        while (runtime.seconds() < 2) {
                            robot.JewelKick.setPosition(0);
                            robot.drive.holdModuleAngle(90+180);
                            telemetry.addData("Red jewel","stop");
                            telemetry.update();
                        }
                        robot.JewelKick.setPosition(kickCenter);
                        robot.JewelArm.setPosition(raisedArm);
                        break;
                    case redFirst:
                        runtime.reset();
                        while (runtime.seconds() < 2) {
                            robot.JewelKick.setPosition(1);
                            robot.drive.holdModuleAngle(90+180);
                            telemetry.addData("Blue jewel","stop");
                            telemetry.update();
                        }
                        robot.JewelKick.setPosition(kickCenter);
                        robot.JewelArm.setPosition(raisedArm);
                        break;
                }

                robot.drive.setEfficiency(false);
                pause(1.5, false);

                switch (VuMark) {
                    case LEFT:
                        driveTime = 2;
                        distance = 24;
                        break;
                    case CENTER:
                        driveTime = 2.5;
                        distance = 24 + 6;
                        break;
                    case RIGHT:
                        driveTime = 3;
                        distance = 24 + 6 + 6;
                        break;
                }

                runtime.reset();
                while (runtime.seconds() < driveTime) {
                    robot.drive.RobotCentric(.15, 0, 0, false);
                }

                pause(3, true);

                robot.intake.setSpeed(-.5);

                pause(2,true);

                runtime.reset();
                while (runtime.seconds() < .5) {
                    robot.drive.RobotCentric(0, .15, 0, false);
                }

                pause(2, true);

                robot.drive.setEfficiency(true);

                runtime.reset();
                while (runtime.seconds() < 1) {
                    robot.drive.RobotCentric(0, -.15, 0, false);
                }

                robot.intake.setSpeed(0);

                blueVision.setShowBlue(false);
                blueVision.disable();

                loop = false;
            }
        }
    }

    public void pause(double seconds, boolean fwd) {
        runtime.reset();
        while (runtime.seconds()<seconds){
            if (fwd) {
                robot.drive.RobotCentric(0,.001,0,false);
                //robot.drive.setEfficiency(false);
                //robot.drive.holdModuleAngle(0);
            } else {
                robot.drive.RobotCentric(.001,0,0,false);
                //robot.drive.setEfficiency(false);
                //robot.drive.holdModuleAngle(90+180);
            }
            //wait
            telemetry.addData("Waiting",seconds+" seconds");
            telemetry.addData("Time",runtime.seconds());
            telemetry.update();
        }
    }
}