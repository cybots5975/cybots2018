package org.firstinspires.ftc.teamcode.MatchCode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Old_Swerve.HardwareSwerveV1;
import org.firstinspires.ftc.teamcode.Sensors.IMU;
import org.firstinspires.ftc.teamcode.Sensors.MA3Encoder;
import org.firstinspires.ftc.teamcode.drivebase.swerve.core.SwerveDrive;

/**
 * Created by kskrueger on 10/22/17.
 */

@Autonomous(name="Test Auto", group="Swerve")
public class TestAuto extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor DMotor1, DMotor2, PMotor1, PMotor2;
    public Servo DServo1, DServo2, PServo1, PServo2;
    public AnalogInput DSensor1, DSensor2, PSensor1, PSensor2;
    public IMU imu, imu2;

    HardwareSwerveV1 robot = new HardwareSwerveV1(); //use the SwerveV1 hardware file to configure
    SwerveDrive drive;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        MA3Encoder DSEncoder = new MA3Encoder(hardwareMap,"EncoderTest");

        drive = new SwerveDrive(hardwareMap, imu, imu2,
                DMotor1,DServo1,DSensor1,
                DMotor2,DServo2,DSensor2,
                PMotor1,PServo1,PSensor1,
                PMotor2,PServo2,PSensor2);

        gamepad1.setJoystickDeadzone(.01F); //Set joystick deadzone to a lower number

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStarted()) {
            //drive.moveEncoder(.1,0,1000);
            while (opModeIsActive() ) {
                drive.gyroMove(.2,0,90);
            }
        }
    }
}
