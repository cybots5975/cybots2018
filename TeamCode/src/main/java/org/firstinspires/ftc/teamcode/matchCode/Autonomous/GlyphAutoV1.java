package org.firstinspires.ftc.teamcode.matchCode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.disnodeteam.dogecv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.util.AutoTransitioner;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.test.zzz_old.ExampleBlueVision;

import static org.firstinspires.ftc.teamcode.test.zzz_old.ExampleBlueVision.jewelsOrder;
import static org.firstinspires.ftc.teamcode.test.zzz_old.ExampleBlueVision.order.unknown;

/**
 * Created by kskrueger on 10/22/17.
 */

@Autonomous(name="Test Glyph Auto BLUE", group="Swerve")
@Disabled
public class GlyphAutoV1 extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();

    private RelicRecoveryVuMark VuMark;
    double driveTime, distance;
    private double kickCenter = .45, raisedArm = .02;
    boolean loop = true;

    ExampleBlueVision blueVision;

    Robot robot = new Robot(this);
    //SwerveDrive drive;

    @Override
    public void runOpMode() {
        robot.Vuforia = true;
        robot.init();

        gamepad1.setJoystickDeadzone(.01F); //Set joystick deadzone to a lower number

        robot.VuMark1.activate();

        while (!isStarted()&&!isStopRequested()) {
            robot.drive.setEfficiency(false);
            robot.drive.holdModuleAngle(90);
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
                robot.JewelArm.setPosition(1);
                ExampleBlueVision.jewelsOrder = unknown;
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
                            robot.drive.holdModuleAngle(90);
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
                            robot.drive.holdModuleAngle(90);
                            telemetry.addData("Blue jewel","stop");
                            telemetry.update();
                        }
                        robot.JewelKick.setPosition(kickCenter);
                        robot.JewelArm.setPosition(raisedArm);
                        break;
                }

                robot.drive.setEfficiency(false);
                pause(1.5,false);

                switch (VuMark) {
                    case LEFT:
                        driveTime = 3;
                        distance = 24 + 6 + 6;
                        break;
                    case CENTER:
                        driveTime = 2.5;
                        distance = 24 + 6;
                        break;
                    case RIGHT:
                        driveTime = 2;
                        distance = 24;
                        break;
                }

                runtime.reset();
                while (runtime.seconds() < driveTime) {
                    robot.drive.robotCentric(-.15, 0, 0);
                }

                //robot.drive.holdModuleAngle(90); todo try this later and remove timer hack
                pause(3, true);

                robot.intake.setSpeed(-.5);

                //robot.drive.holdModuleAngle(0); todo here too
                pause(2,true);

                runtime.reset();
                while (runtime.seconds() < .5) {
                    robot.drive.robotCentric(0, .15, 0);
                }

                pause(2, true);

                robot.drive.setEfficiency(true);

                runtime.reset();
                while (runtime.seconds() < 1) {
                    robot.drive.robotCentric(0, -.15, 0);
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
        while (runtime.seconds()<seconds&&!isStopRequested()){
            if (fwd) {
                robot.drive.robotCentric(0,.001,0);
            } else {
                robot.drive.robotCentric(-.001,0,0);
            }
            //wait
            telemetry.addData("Waiting",seconds+" seconds");
            telemetry.addData("Time",runtime.seconds());
            telemetry.update();
        }
    }
}