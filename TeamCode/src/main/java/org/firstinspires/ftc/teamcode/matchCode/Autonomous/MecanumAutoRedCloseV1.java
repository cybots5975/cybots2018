package org.firstinspires.ftc.teamcode.matchCode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.general.Robot;

/**
 * Created by kskrueger on 10/22/17.
 */

@Autonomous(name="Red Close V1", group="Mecanum")
//@Disabled
public class MecanumAutoRedCloseV1 extends LinearOpMode {
    Robot robot = new Robot(); //use the SwerveV1 hardware file to configure
    private boolean loop = true;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.drive.zeroEncoders();
        robot.drive.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()&&loop&&!isStopRequested()) {

            encoderStrafe(.25,-1425);

            loop = false;
        }
    }

    public void encoderStrafe(double power, int encoder){
        while (robot.drive.getStrafeEncoderAverage()>encoder&&!isStopRequested()) {
            robot.drive.driveMecanum(0,power,0);
        }
        robot.drive.driveMecanum(0,0,0);
    }
}
