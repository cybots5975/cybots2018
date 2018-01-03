package org.firstinspires.ftc.teamcode.matchCode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.sensors.IMU;

/**
 * Created by kskrueger on 10/22/17.
 */

@Autonomous(name="Test Auto", group="Swerve")
@Disabled
public class TestAuto extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor DMotor1, DMotor2, PMotor1, PMotor2;
    public Servo DServo1, DServo2, PServo1, PServo2;
    public AnalogInput DSensor1, DSensor2, PSensor1, PSensor2;
    public IMU imu, imu2;

    Robot robot = new Robot(this);

    @Override
    public void runOpMode() {
        robot.init();

        gamepad1.setJoystickDeadzone(.01F); //Set joystick deadzone to a lower number

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStarted()) {
            //drive.moveEncoder(.1,0,1000);
            while (opModeIsActive() ) {


                robot.drive.gyroDrive(.2,0,90);

                robot.drive.gyroDrive(.2,0,90);
                
            }
        }
    }
}
