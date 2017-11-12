package org.firstinspires.ftc.teamcode.matchCode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.general.Robot;

/**
 * Created by kskrueger on 10/22/17.
 */

@Autonomous(name="Test Glyph Auto OPP", group="Swerve")
public class GlyphAutoV1_opp extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();

    public RelicRecoveryVuMark VuMark;
    double driveTime, distance;
    boolean loop = true;

    Robot robot = new Robot(); //use the SwerveV1 hardware file to configure
    //SwerveDrive drive;

    @Override
    public void runOpMode() {
        robot.Vuforia = true;
        robot.init(hardwareMap);

        gamepad1.setJoystickDeadzone(.01F); //Set joystick deadzone to a lower number

        robot.VuMark1.activate();

        while (!isStarted()&&!isStopRequested()) {
            robot.drive.setEfficiency(false);
            robot.drive.setModuleAngle(-90);
            telemetry.addData("VuMark",robot.VuMark1.scan());
            if (robot.VuMark1.scan()==RelicRecoveryVuMark.UNKNOWN) {
                VuMark = RelicRecoveryVuMark.CENTER;
            } else {
                VuMark = robot.VuMark1.scan();
            }
            telemetry.addData("Status", "Initialized");
            telemetry.update();
        }

        waitForStart();
        if (isStarted()) {
            while(opModeIsActive()&&loop&&!isStopRequested()) {
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

                pause(5, true);

                robot.intake.setSpeed(0);

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
