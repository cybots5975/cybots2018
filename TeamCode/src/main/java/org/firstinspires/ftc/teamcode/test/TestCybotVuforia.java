package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * Created by kskrueger on 10/22/17.
 */

@Autonomous(name="Test Vumark", group="Test")
//@Disabled
public class TestCybotVuforia extends LinearOpMode{
    private Robot robot = new Robot(this);
    private boolean loop = true;

    @Override
    public void runOpMode() {
        robot.Vuforia = true;
        robot.init();

        robot.VuMark1.activate();

        telemetry.addData("Position:", robot.prefs.read("postion"));
        telemetry.update();

        waitForStart();
        robot.startVumarkPositionThread();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()&&loop&&!isStopRequested()) {

            telemetry.addData("tX",robot.VuMark1.tX/25.4);
            telemetry.addData("tY",robot.VuMark1.tY/25.4);
            telemetry.addData("tZ",robot.VuMark1.tZ/25.4);
            telemetry.addData("rX",robot.VuMark1.rX);
            telemetry.addData("rY",robot.VuMark1.rY);
            telemetry.addData("rZ",robot.VuMark1.rZ);

            telemetry.update();

            //loop = false;
        }
        robot.stopVumarkPositionThread();
        robot.VuMark1.close();
    }
}
