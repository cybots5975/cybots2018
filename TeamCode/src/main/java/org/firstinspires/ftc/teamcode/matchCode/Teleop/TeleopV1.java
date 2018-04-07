package org.firstinspires.ftc.teamcode.matchCode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.GlyphMech;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.logging.ArrayLogging;

import static org.firstinspires.ftc.teamcode.subsystems.GlyphMech.height.HIGH;
import static org.firstinspires.ftc.teamcode.subsystems.GlyphMech.height.LOW;
import static org.firstinspires.ftc.teamcode.subsystems.GlyphMech.height.MID;
import static org.firstinspires.ftc.teamcode.subsystems.GlyphMech.height.STORE;

/**
 * Created by kskrueger on 10/18/17.
 */
@TeleOp(name="Teleop V1", group="Super Qual")
//@Disabled
public class TeleopV1 extends  LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ArrayLogging log = new ArrayLogging(2,5000);
    private int count = 0;

    GlyphMech.height height = STORE;

    boolean glyphOff = false;
    boolean positionMode = true;
    boolean relicClaw = false;

    boolean initRelic = false;

    Robot robot = new Robot(this);

    boolean pressed = false;

    private ElapsedTime glyphResetTime = new ElapsedTime();

    private ElapsedTime glyphGrabTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.Vuforia = false;
        robot.isTeleop = true;
        robot.init();

        robot.JewelArm.setPwmDisable();

        robot.drive.setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gamepad1.setJoystickDeadzone(.01F); //Set joystick deadzone to a lower number

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initializeLogging();

        //wait for the program to start (operator presses PLAY)
        waitForStart();
        runtime.reset();

        robot.positionTracking.wheelsUp();

        //run until the end (operator presses STOP)
        while (opModeIsActive()&&!isStopRequested()) {
            gamepad1.setJoystickDeadzone(.01F); //Set joystick deadzone to a lower number

            double leftY = Math.signum(-gamepad1.left_stick_y) * Math.pow(-gamepad1.left_stick_y, 2);
            double leftX = Math.signum(-gamepad1.left_stick_x) * Math.pow(-gamepad1.left_stick_x, 2);
            double rightX = Math.signum(gamepad1.right_stick_x) * Math.pow(gamepad1.right_stick_x, 2);

            if (gamepad1.right_bumper||!height.equals(STORE)) {
                leftX = leftX/2;
                rightX = rightX/2.5;
            }

            if (gamepad1.left_bumper) {
                leftY = leftY/2;
            }

            //robot.drive.setEfficiency(true); //used for swerve
            //robot.drive.zeroReset(gamepad1.a); //used for swerve

           /*if (gamepad2.a) {
                rightX /= 2;
                robot.drive.robotCentric(leftY,leftX,rightX);
            } else {
            }*/

           if (gamepad2.dpad_up&&!initRelic) {
               robot.initRelicVision();
               initRelic = true;
           }

           if (gamepad2.dpad_down&&initRelic&&robot.relicDetector.getFound()) {
               robot.drive.turnPID.setVariables(.08,0,.1);
               robot.drive.turnDrivePID(leftY,leftX, (int) robot.relicDetector.getPosX(),260,.05);
           } else {
               robot.drive.robotCentric(leftY,leftX,rightX);
           }


            glyph();

            intake();

            relic();

            log();

            telemetry.addData("Left Intake",leftPosition);
            telemetry.addData("Right Intake",rightPosition);
            telemetry.addData("Running","");
            telemetry.update();

            if (gamepad1.start) {
                log.log("teleopV1");
            }
        }
        robot.disableRelicVision();
    }

    private void glyph(){
        //robot.intake.open();

        if (gamepad1.a||gamepad2.y) {
            height = HIGH;
            robot.glyphMech.setDumpSpeed(1);
            glyphGrabTime.reset();
            glyphOff = false;

        } else if (gamepad1.y||gamepad2.x) {
            height = MID;
            robot.glyphMech.setDumpSpeed(1);
            glyphGrabTime.reset();
            glyphOff = false;

        } else if (gamepad1.x||gamepad2.a) {
            height = LOW;
            robot.glyphMech.setDumpSpeed(1);
            glyphGrabTime.reset();
            glyphOff = false;

        } else if (gamepad1.b||gamepad2.b) {
            height = STORE;
            robot.glyphMech.setDumpSpeed(0);
            glyphGrabTime.reset();
            glyphOff = true;

        } else if (gamepad2.left_stick_button) {
            robot.ArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (robot.boxLimit.getVoltage()<.5) {
                robot.ArmMotor.setPower(.5);
            } else {
                robot.ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }

        if (gamepad1.right_bumper||!height.equals(STORE)) {
            robot.glyphMech.grab();
        } else {
            robot.glyphMech.drop();
        }

        if (height==STORE  &&  !pressed  &&  robot.boxLimit.getVoltage()>1) {
            robot.glyphMech.reset();
            glyphResetTime.reset();
            pressed = true;
        }

        telemetry.addData("Time since reset",glyphResetTime.seconds());

        if (robot.boxLimit.getVoltage()<1) {
            pressed = false;
        }

        boolean inPosition = Math.abs(Math.abs(robot.ArmMotor.getCurrentPosition())-Math.abs(robot.ArmMotor.getTargetPosition())) < 25;

        if (glyphOff&&robot.boxLimit.getVoltage()>.5) {
            robot.glyphMech.setDumpSpeed(0);
            telemetry.addData("Dump","off - Position:"+inPosition);
        } else {
            //robot.glyphMech.setDumpSpeed(1);
            telemetry.addData("Dump","on - Position:"+inPosition);
        }

        /*if (gamepad2.left_stick_button&&!robot.glyphMech.zeroBusy) {
            robot.glyphMech.zero();
        }*/

        if (gamepad2.left_stick_button) {

        } else {
            if (glyphResetTime.milliseconds()>50) {
                if (height==STORE&&!(gamepad2.b||gamepad1.b)&&robot.boxLimit.getVoltage()<.5) {
                    robot.glyphMech.setDumpSpeed(1);
                }
                robot.glyphMech.runProcess(height);
            }
        }

        telemetry.addData("Limit",robot.boxLimit.getVoltage());
    }

    double leftPosition = .225; //.156 old
    double rightPosition = 1; //.978 old

    private void intake() {
        if (gamepad1.right_trigger>.1) {
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

        if (gamepad2.a) {
            positionMode = false;
        } else /*if (gamepad1.left_stick_button)*/ {
            positionMode = true;
        }

        if (positionMode) {
            //robot.intake.setAngle(leftPosition,rightPosition);
            robot.intake.multiGlyph();
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

        if (!gamepad2.right_bumper) {
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
    }

    public void log() {
        count += 1;

        log.storeValueInt(0, count, count);
        log.storeValueInt(1, count, runtime.milliseconds());
    }
}
