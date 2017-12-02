package org.firstinspires.ftc.teamcode.matchCode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.general.Robot;
import org.firstinspires.ftc.teamcode.test.ExampleBlueVision;

/**
 * Created by kskrueger on 10/22/17.
 */

@Autonomous(name="Test Glyph Auto RED", group="Swerve")
public class GlyphAutoV1_opp extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();

    ExampleBlueVision blueVision;

    public RelicRecoveryVuMark VuMark;
    double driveTime, distance;
    boolean loop = true;

    Robot robot = new Robot(); //use the SwerveV1 hardware file to configure
    //SwerveDrive drive;

    enum order{
        blueFirst,
        redFirst;
    }

    @Override
    public void runOpMode() {
        robot.Vuforia = true;
        robot.init(hardwareMap);

        gamepad1.setJoystickDeadzone(.01F); //Set joystick deadzone to a lower number

        robot.VuMark1.activate();

        while (!isStarted()&&!isStopRequested()) {
            robot.JewelArm.setPosition(0);
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
        waitForStart();
        if (isStarted()) {
            while(opModeIsActive()&&loop&&!isStopRequested()) {
                robot.JewelArm.setPosition(1);
                blueVision = new ExampleBlueVision();
                // can replace with ActivityViewDisplay.getInstance() for fullscreen
                blueVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
                blueVision.setShowBlue(false);
                // start the vision system
                blueVision.enable();

                blueVision.setShowBlue(true);

                order jewelOrder;

                /*if (blueVision.order.equals("Blue,Red")) {
                    jewelOrder = order.blueFirst;
                    telemetry.addData("Order",jewelOrder.toString());
                } else {
                    jewelOrder = order.redFirst;
                    telemetry.addData("Order",jewelOrder.toString());
                }*/

                double jewelTime = 0;

                robot.drive.setEfficiency(false);
                pause(1.5,false);

                switch (ExampleBlueVision.jewelsOrder) {
                    case redFirst:
                        robot.drive.setEfficiency(true);
                        runtime.reset();
                        jewelTime = .4;
                        while (runtime.seconds() < .5) {
                            //////////////
                            robot.drive.RobotCentric(-.1, 0, 0, false);
                            telemetry.addData("Red jewel","stop");
                            telemetry.update();
                        }
                        robot.drive.RobotCentric(0,.001,0,false);
                        robot.drive.setEfficiency(false);
                        robot.JewelArm.setPosition(0);
                        break;
                    case blueFirst:
                        robot.drive.setEfficiency(true);
                        runtime.reset();
                        jewelTime = -.5;
                        while (runtime.seconds() < .3) {
                            /////////////
                            robot.drive.RobotCentric(.1, 0, 0, false);
                            telemetry.addData("Blue jewel","stop");
                            telemetry.update();
                        }
                        robot.drive.RobotCentric(0,.001,0,false);
                        robot.drive.setEfficiency(false);
                        robot.JewelArm.setPosition(0);
                        break;
                }

                robot.drive.setEfficiency(false);
                pause(1.5, false);

                //robot.drive.resetEncoders();

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
                while (runtime.seconds() < driveTime+jewelTime) {
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
                robot.drive.RobotCentric(0,.01,0,false);
            } else {
                robot.drive.RobotCentric(.001,0,0,false);
            }
            //wait
            telemetry.addData("Waiting",seconds+" seconds");
            telemetry.addData("Time",runtime.seconds());
            telemetry.update();
        }
    }
}
