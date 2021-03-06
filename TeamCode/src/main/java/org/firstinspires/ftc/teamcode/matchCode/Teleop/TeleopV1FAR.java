package org.firstinspires.ftc.teamcode.matchCode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.GlyphMech;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import static org.firstinspires.ftc.teamcode.subsystems.GlyphMech.height.HIGH;
import static org.firstinspires.ftc.teamcode.subsystems.GlyphMech.height.LOW;
import static org.firstinspires.ftc.teamcode.subsystems.GlyphMech.height.MID;
import static org.firstinspires.ftc.teamcode.subsystems.GlyphMech.height.STORE;

/**
 * Created by kskrueger on 10/18/17.
 */

@TeleOp(name="Teleop FAR", group="Super Qual")
//@Disabled
public class TeleopV1FAR extends  LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    GlyphMech.height height = STORE;

    boolean intakereverse = false;

    Robot robot = new Robot(this);

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

        //run until the end (operator presses STOP)
        while (opModeIsActive()) {
            gamepad1.setJoystickDeadzone(.01F); //Set joystick deadzone to a lower number

            double leftX = -gamepad1.left_stick_y;
            double leftY = -gamepad1.left_stick_x;
            double rightX = gamepad1.right_stick_x;

            if (gamepad1.left_bumper) {
                leftX = -leftX;
                leftY = -leftY;
            }

            if (gamepad1.right_bumper) {
                leftX = leftX/2;
            }

            if (gamepad1.right_bumper) {
                rightX = rightX/2;
            }

            robot.drive.setEfficiency(true); //used for swerve
            robot.drive.zeroReset(gamepad1.a); //used for swerve
            robot.drive.robotCentric(leftY,leftX,rightX);

            glyph();

            intake();

            telemetry.addData("Left Intake",leftPosition);
            telemetry.addData("Right Intake",rightPosition);
            telemetry.addData("Running","");
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

    double leftPosition = .156;
    double rightPosition = .978;

    private void intake(){
        if (intakereverse) {
            robot.intake.setSpeed(.2);
        } else if (gamepad1.right_trigger>.1) {
            robot.intake.setSpeed(gamepad1.right_trigger);
        } else {
            robot.intake.setSpeed(-gamepad1.left_trigger);
        }

        if (gamepad1.dpad_up) {
            rightPosition += .001;
        } else if (gamepad1.dpad_down) {
            rightPosition -= .001;
        } else if (gamepad1.dpad_left) {
            leftPosition += .001;
        } else if (gamepad1.dpad_right) {
            leftPosition -= .001;
        }

        robot.intake.setAngle(leftPosition,rightPosition);
    }
}
