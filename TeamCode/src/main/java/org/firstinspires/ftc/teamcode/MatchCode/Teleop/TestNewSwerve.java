package org.firstinspires.ftc.teamcode.MatchCode.Teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Old_Swerve.HardwareSwerveV1;
import org.firstinspires.ftc.teamcode.drivebase.swerve.core.SwerveDrive;

/**
 * Created by kskrueger on 10/18/17.
 */

@TeleOp(name="Test New Swerve", group="Swerve")
//@Disabled
public class TestNewSwerve extends  LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor DMotor1, DMotor2, PMotor1, PMotor2;
    public Servo DServo1, DServo2, PServo1, PServo2;
    public AnalogInput DSensor1, DSensor2, PSensor1, PSensor2;
    public BNO055IMU imu, imu2;

    HardwareSwerveV1 robot = new HardwareSwerveV1(); //use the SwerveV1 hardware file to configure
    SwerveDrive drive;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        drive = new SwerveDrive(hardwareMap, imu, imu2,
                            DMotor1,DServo1,DSensor1,
                            DMotor2,DServo2,DSensor2,
                            PMotor1,PServo1,PSensor1,
                            PMotor2,PServo2,PSensor2);

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

            drive.setEfficiency(true);
            drive.RobotCentric(leftX,leftY,rightX,gamepad1.a);

            telemetry.addData("Swerve Running","");
            telemetry.update();
        }
    }
}
