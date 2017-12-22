package org.firstinspires.ftc.teamcode.matchCode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.general.Robot;
import org.firstinspires.ftc.teamcode.sensors.IMU;

/**
 * Created by kskrueger on 10/22/17.
 */

@Autonomous(name="Test VISION", group="Testing")
public class VisionAutoTesting extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor DMotor1, DMotor2, PMotor1, PMotor2;
    public Servo DServo1, DServo2, PServo1, PServo2;
    public AnalogInput DSensor1, DSensor2, PSensor1, PSensor2;
    public IMU imu, imu2;

    Robot robot = new Robot(); //use the SwerveV1 hardware file to configure
    //SwerveDrive drive;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        gamepad1.setJoystickDeadzone(.01F); //Set joystick deadzone to a lower number

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStarted()) {
            //drive.moveEncoder(.1,0,1000);
            while (opModeIsActive() ) {


                robot.drive.gyroMove(.2,0,90);

                robot.drive.gyroMove(.2,0,90);
                
            }
        }
    }
}
