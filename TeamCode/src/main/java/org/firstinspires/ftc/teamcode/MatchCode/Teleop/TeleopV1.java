package org.firstinspires.ftc.teamcode.MatchCode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.General.Robot;

/**
 * Created by kskrueger on 10/18/17.
 */

@TeleOp(name="Teleop V1", group="1Swerve")
//@Disabled
public class TeleopV1 extends  LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    Robot robot = new Robot(); //use the SwerveV1 hardware file to configure
    //SwerveDrive drive;

    @Override
    public void runOpMode() {
        robot.Vuforia = false;
        robot.init(hardwareMap);

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
            double rightX = -gamepad1.right_stick_x/2;

            robot.drive.setEfficiency(true);
            robot.drive.RobotCentric(leftX,leftY,rightX,gamepad1.a);
            if (gamepad1.right_trigger>.1) {
                robot.intake.setSpeed(gamepad1.right_trigger);
            } else {
                robot.intake.setSpeed(-gamepad1.left_trigger);
            }

            telemetry.addData("Swerve Running","");
            telemetry.update();
        }
    }
}
