package org.firstinspires.ftc.teamcode.matchCode.Autonomous;

import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.subsystems.drivebase.VectorDrive;
import org.firstinspires.ftc.teamcode.general.Robot;

/**
 * Created by kskrueger on 12/20/17.
 */

@Autonomous(name="Red Far V1", group="Mecanum")
public class MecanumAutoRedFarV1 extends LinearOpMode{
    private RelicRecoveryVuMark VuMark;
    private int encoderCounts;
    private Robot robot = new Robot(this);
    private boolean loop = true;

    @Override
    public void runOpMode() {
        robot.Vuforia = true;
        //robot.init(hardwareMap);
        //robot.setOpMode(this);
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

            robot.pause(2);

            scoreJewel(robot.jewelOrder);

            robot.pause(1.5);

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

            loop = false;
        }
    }

    private void scoreJewel(JewelDetector.JewelOrder jewelOrder){
        switch (jewelOrder) {
            case RED_BLUE:
                robot.runtime.reset();
                while (robot.runtime.seconds() < 2) {
                    robot.JewelKick.setPosition(robot.kickLeft);
                    telemetry.addData("Red jewel","stop");
                    telemetry.update();
                }
                robot.JewelKick.setPosition(robot.kickCenter);
                robot.JewelArm.setPosition(robot.raisedArm);
                break;
            case BLUE_RED:
                robot.runtime.reset();
                while (robot.runtime.seconds() < 2) {
                    robot.JewelKick.setPosition(robot.kickRight);
                    telemetry.addData("Blue jewel","stop");
                    telemetry.update();
                }
                robot.JewelKick.setPosition(robot.kickCenter);
                robot.JewelArm.setPosition(robot.raisedArm);
                break;
            case UNKNOWN:

                break;
        }
    }

    /*private double kP = .1, kD = .08, kI = 0;
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
    }*/
}
