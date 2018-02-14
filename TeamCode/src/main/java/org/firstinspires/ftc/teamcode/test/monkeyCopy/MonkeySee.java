package org.firstinspires.ftc.teamcode.test.monkeyCopy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.logging.ArrayLogging;

import java.io.IOException;

@TeleOp(name="Record Robot Drive Test", group="Monkey Copy")
//@Disabled
public class MonkeySee extends LinearOpMode {
    private Robot robot = new Robot(this);
    private ArrayLogging log = new ArrayLogging(16,10000);
    public ElapsedTime runtime = new ElapsedTime();
    private double[] position = new double[4];
    private double[] velocity = new double[4];
    private int count = 0;

    double previousMilli = 0;

    @Override
    public void runOpMode() {
        robot.init();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        initializeLogging();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.drive.robotCentric(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);

            position[0] = robot.DMotor1.getCurrentPosition();
            position[1] = robot.DMotor2.getCurrentPosition();
            position[2] = robot.PMotor1.getCurrentPosition();
            position[3] = robot.PMotor2.getCurrentPosition();

            velocity[0] = robot.DMotor1.getVelocity(AngleUnit.DEGREES);
            velocity[1] = robot.DMotor2.getVelocity(AngleUnit.DEGREES);
            velocity[2] = robot.PMotor1.getVelocity(AngleUnit.DEGREES);
            velocity[3] = robot.PMotor2.getVelocity(AngleUnit.DEGREES);

            log(runtime.milliseconds(),
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    robot.imu.getHeading(),
                    robot.drive.getFwdEncoderAverage(),
                    robot.drive.getStrafeEncoderAverage(),
                    position,
                    velocity);

            while(runtime.milliseconds()-previousMilli < 40) {
                //wait
            }
            previousMilli = runtime.milliseconds();

            if (gamepad1.b) {
                try {
                    log.log("monkeySeeRobot");
                } catch (IOException e) {
                    e.printStackTrace();
                }
                telemetry.addData("Written","");
            }

            telemetry.update();
        }
    }

    public void initializeLogging() {
        log.storeValue(0, 0, "Count #");
        log.storeValue(1, 0, "Time");
        log.storeValue(2, 0, "LeftY Joystick");
        log.storeValue(3, 0, "LeftX Joystick");
        log.storeValue(4, 0, "RightX Joystick");

        log.storeValue(5, 0, "Gyro Heading");
        log.storeValue(6, 0, "Forward Encoder Counts");
        log.storeValue(7, 0, "Strafe Encoder Counts");

        log.storeValue(8, 0, "D1 Position");
        log.storeValue(9, 0, "D1 Velocity");

        log.storeValue(10, 0, "D2 Position");
        log.storeValue(11, 0, "D2 Velocity");

        log.storeValue(12, 0, "P1 Position");
        log.storeValue(13, 0, "P1 Velocity");

        log.storeValue(14, 0, "P1 Position");
        log.storeValue(15, 0, "P2 Velocity");

        log.storeValue(16, 0, "Intake Speed");

        log.storeValue(17, 0, "Left Intake Position");
        log.storeValue(18, 0, "Right Intake Position");
        log.storeValue(19, 0, "Close arm button T/F");
    }

    public void log(double time, double leftY, double leftX, double rightX, double gyroHeading, int fwdCount, int strafeCounts, double[] position, double[] velocity) {
        count += 1;

        log.storeValueInt(0, count, count);
        log.storeValueInt(1, count, time);
        log.storeValueInt(2, count, leftY);
        log.storeValueInt(3, count, leftX);
        log.storeValueInt(4, count, rightX);

        log.storeValueInt(5, count, gyroHeading);
        log.storeValueInt(6, count, fwdCount);
        log.storeValueInt(7, count, strafeCounts);

        log.storeValueInt(8, count, position[0]);
        log.storeValueInt(9, count, velocity[0]);

        log.storeValueInt(10, count, position[1]);
        log.storeValueInt(11, count, velocity[1]);

        log.storeValueInt(12, count, position[2]);
        log.storeValueInt(13, count, velocity[2]);

        log.storeValueInt(14, count, position[3]);
        log.storeValueInt(15, count, velocity[3]);
    }

}