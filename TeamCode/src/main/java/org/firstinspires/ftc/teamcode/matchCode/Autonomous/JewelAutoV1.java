package org.firstinspires.ftc.teamcode.matchCode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.disnodeteam.dogecv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.general.AutoTransitioner;
import org.firstinspires.ftc.teamcode.general.Robot;
import org.firstinspires.ftc.teamcode.test.zzz_old.ExampleBlueVision;

import static org.firstinspires.ftc.teamcode.test.zzz_old.ExampleBlueVision.jewelsOrder;
import static org.firstinspires.ftc.teamcode.test.zzz_old.ExampleBlueVision.order.unknown;

/**
 * Created by kskrueger on 10/22/17.
 */

@Autonomous(name="Jewel Auto Trial", group="AutoOnly")
public class JewelAutoV1 extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();

    private ExampleBlueVision blueVision;

    private double kickCenter = .45, raisedArm = .02;
    private boolean loop = true;


    private Robot robot = new Robot(); //use the SwerveV1 hardware file to configure

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        while (!isStarted()&&!isStopRequested()) {
            robot.JewelArm.setPosition(raisedArm);
            robot.JewelKick.setPosition(kickCenter);
            telemetry.addData("Status", "Initialized");
            telemetry.update();
        }
        AutoTransitioner.transitionOnStop(this, "Teleop V1");
        waitForStart();
        if (isStarted()) {
            while(opModeIsActive()&&loop&&!isStopRequested()) {
                jewelsOrder = unknown;
                robot.JewelKick.setPosition(kickCenter);
                robot.JewelArm.setPosition(1);
                blueVision = new ExampleBlueVision();
                // can replace with ActivityViewDisplay.getInstance() for fullscreen
                blueVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
                blueVision.setShowBlue(false);
                // start the vision system
                blueVision.enable();

                blueVision.setShowBlue(true);

                sleep(4000);

                //jewelsOrder = jewelsOrder.blueFirst;
                switch (jewelsOrder) {
                    case blueFirst:
                        runtime.reset();
                        while (runtime.seconds() < 2) {
                            robot.JewelKick.setPosition(0);
                            telemetry.addData("Red jewel","stop");
                            telemetry.update();
                        }
                        robot.JewelKick.setPosition(kickCenter);
                        robot.JewelArm.setPosition(raisedArm);
                        break;
                    case redFirst:
                        runtime.reset();
                        while (runtime.seconds() < 2) {
                            robot.JewelKick.setPosition(1);
                            telemetry.addData("Blue jewel","stop");
                            telemetry.update();
                        }
                        robot.JewelKick.setPosition(kickCenter);
                        robot.JewelArm.setPosition(raisedArm);
                        break;
                }

                sleep(3000);

                blueVision.setShowBlue(false);
                blueVision.disable();

                loop = false;
            }
        }
    }
}