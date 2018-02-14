package org.firstinspires.ftc.teamcode.matchCode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.GlyphMech;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.logging.ArrayLogging;

import java.io.IOException;

import static org.firstinspires.ftc.teamcode.subsystems.GlyphMech.height.HIGH;
import static org.firstinspires.ftc.teamcode.subsystems.GlyphMech.height.LOW;
import static org.firstinspires.ftc.teamcode.subsystems.GlyphMech.height.MID;
import static org.firstinspires.ftc.teamcode.subsystems.GlyphMech.height.STORE;

/**
 * Created by kskrueger on 10/18/17.
 */

@TeleOp(name="Teleop V1 LOG", group="State TEST")
//@Disabled
public class TeleopLOGGING extends  LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    GlyphMech.height height = STORE;

    boolean intakereverse = false;
    boolean positionMode = true;
    boolean relicClaw = false;

    Robot robot = new Robot(this);

    private ArrayLogging log = new ArrayLogging(20,10000);
    //public ElapsedTime runtime = new ElapsedTime();
    private double[] position = new double[4];
    private double[] velocity = new double[4];
    private int count = 0;

    @Override
    public void runOpMode() {
        robot.Vuforia = false;
        robot.init();

        robot.JewelArm.setPwmDisable();

        robot.drive.setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gamepad1.setJoystickDeadzone(.01F); //Set joystick deadzone to a lower number

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //wait for the program to start (operator presses PLAY)
        waitForStart();
        runtime.reset();
        robot.drive.setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        initializeLogging();
        //run until the end (operator presses STOP)
        while (opModeIsActive()) {
            RunLog();
            gamepad1.setJoystickDeadzone(.01F); //Set joystick deadzone to a lower number

            double leftY = -gamepad1.left_stick_y;
            double leftX = -gamepad1.left_stick_x;
            double rightX = gamepad1.right_stick_x;

            if (gamepad1.right_bumper) {
                leftX = leftX/2;
                rightX = rightX/2.5;
            }

            if (gamepad1.left_bumper) {
                leftY = leftY/2;
            }

            robot.drive.setEfficiency(true); //used for swerve
            robot.drive.zeroReset(gamepad1.a); //used for swerve
            robot.drive.robotCentric(leftY,leftX,rightX);

            glyph();

            intake();

            relic();

            telemetry.addData("Left Intake",leftPosition);
            telemetry.addData("Right Intake",rightPosition);
            telemetry.addData("Running","");

            if (gamepad1.start) {
                try {
                    log.save("hungryHippo");
                } catch (IOException e) {
                    e.printStackTrace();
                }
                telemetry.addData("File Written","");
            }

            telemetry.update();
        }
    }

    private void glyph(){
        //robot.intake.open();

        if (gamepad1.right_bumper) {
            robot.glyphMech.grab();
        } else {
            robot.glyphMech.drop();
        }

        if (gamepad1.a) {
            height = HIGH;
            intakereverse = true;
        } else if (gamepad1.y) {
            height = MID;
            intakereverse = true;
        } else if (gamepad1.x) {
            height = LOW;
            intakereverse = true;
        } else if (gamepad1.b) {
            height = STORE;
            intakereverse = false;
        }

        robot.glyphMech.runProcess(height);
    }

    double leftPosition = .225; //.156 old
    double rightPosition = 1; //.978 old

    private void intake(){
        if (intakereverse) {
            robot.intake.setSpeed(.2);
        } else if (gamepad1.right_trigger>.1) {
            robot.intake.setSpeed(gamepad1.right_trigger);
        } else {
            robot.intake.setSpeed(-gamepad1.left_trigger);
        }

        if (gamepad1.dpad_up) {
            rightPosition += .05;
            //was .001
        } else if (gamepad1.dpad_down) {
            rightPosition -= .05;
        } else if (gamepad1.dpad_left) {
            leftPosition += .05;
        } else if (gamepad1.dpad_right) {
            leftPosition -= .05;
        }

        if (gamepad1.left_bumper) {
            rightPosition = .825;
        } else if (gamepad1.start) {
            rightPosition = 0;
        } else {
            rightPosition = 1;
        }

        if (gamepad1.right_stick_button) {
            positionMode = false;
        } else if (gamepad1.left_stick_button) {
            positionMode = true;
        }
        if (positionMode) {
            robot.intake.setAngle(leftPosition,rightPosition);
        } else {
            robot.intake.auton();
        }
    }

    private void relic() {
        /*if (gamepad2.dpad_up) {
            robot.relicArm.pivotUp();
        } else if (gamepad2.dpad_down) {
            robot.relicArm.pivotDown();
        }*/
        if (gamepad2.left_bumper) {
            robot.relicArm.pivotUp();
        } else {
            robot.relicArm.pivotDown();
        }

        if (gamepad2.right_bumper) {
            robot.relicArm.grab();
            relicClaw = true;
        } /*else if (gamepad2.left_bumper) {
            robot.relicArm.release();
        } */else {
            if (relicClaw) {
                robot.relicArm.release();
            } else {
                robot.relicArm.clawInit();
            }
        }

        robot.relicArm.extendArm(-gamepad2.right_stick_y);
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

        log.storeValueInt(16, count, robot.IntakeMotor.getPower());

        log.storeValueInt(17, count, leftPosition);
        log.storeValueInt(18, count, rightPosition);

        //log.storeValueInt(19, count, );
    }

    public void RunLog() {
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
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
            }
        }).start();
    }
}
