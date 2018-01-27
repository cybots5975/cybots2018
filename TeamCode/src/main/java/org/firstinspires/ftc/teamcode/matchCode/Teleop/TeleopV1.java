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

@TeleOp(name="Teleop V1", group="1League Champ")
//@Disabled
public class TeleopV1 extends  LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    GlyphMech.height height = STORE;

    boolean intakereverse = false;

    Robot robot = new Robot(this);

    @Override
    public void runOpMode() {
        robot.Vuforia = false;
        robot.init();

        robot.drive.setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gamepad1.setJoystickDeadzone(.01F); //Set joystick deadzone to a lower number

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //wait for the program to start (operator presses PLAY)
        waitForStart();
        runtime.reset();

        //run until the end (operator presses STOP)
        while (opModeIsActive()) {
            gamepad1.setJoystickDeadzone(.01F); //Set joystick deadzone to a lower number

            double leftY = -gamepad1.left_stick_y;
            double leftX = -gamepad1.left_stick_x;
            double rightX = gamepad1.right_stick_x;

            if (gamepad1.right_bumper) {
                leftX = leftX/2;
            }

            if (gamepad1.right_bumper) {
                rightX = rightX/2;
            }

            robot.drive.setEfficiency(true);
            robot.drive.robotCentric(leftY,leftX,rightX);
            robot.drive.zeroReset(gamepad1.a);
            if (gamepad1.right_trigger>.1) {
                robot.intake.setSpeed(gamepad1.right_trigger);
            } else {
                robot.intake.setSpeed(-gamepad1.left_trigger);
            }

            glyph();

            telemetry.addData("Running","");
            telemetry.update();
        }
    }

    void glyph(){
        //robot.intake.open();

        if (gamepad1.right_bumper) {
            robot.glyphMech.grab();
        } else {
            robot.glyphMech.drop();
        }

        if (gamepad1.left_bumper) {
            robot.intake.pinch();
        } else {
            robot.intake.open();
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

        if (intakereverse) {
            robot.intake.setSpeed(-1);
        }

        robot.glyphMech.runProcess(height);
    }
}
