package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.Objects;

/**
 * Created by kskrueger on 10/22/17.
 */

@Autonomous(name="Hybrid vuforia and jewel CV", group="Template")
public class TestHybridVuforiaCV extends LinearOpMode{
    private Robot robot = new Robot(this);
    private boolean loop = true;

    @Override
    public void runOpMode() {
        robot.Vuforia = true;
        robot.JewelVision = true;
        robot.init();

        robot.VuMark1.activate();

        if (Objects.equals(robot.prefs.read("testMode"), "false")) {
            AutoTransitioner.transitionOnStop(this, "Teleop V1");
        }
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()&&loop&&!isStopRequested()) {
            telemetry.addData("Position:", robot.prefs.read("postion"));
            telemetry.addData("VuMark",robot.VuMark1.scan().toString());

            robot.jewelVision.vumarkFrames(robot.VuMark1.capture());
            telemetry.addData("Order",robot.jewelVision.jewelOrder());

            telemetry.update();


        }
    }

    private void scorePosition() {
        switch (robot.prefs.read("position")) {
            case "RED CLOSE":
                redClose();
                break;
            case "RED FAR":
                redFar();
                break;
            case "BLUE CLOSE":
                blueClose();
                break;
            case "BLUE FAR":
                blueFar();
                break;
        }
    }

    private void redClose(){

    }

    private void redFar(){

    }

    private void blueClose(){

    }

    private void blueFar(){

    }

}
