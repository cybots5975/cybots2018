package org.firstinspires.ftc.teamcode.test.zzz_old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.disnodeteam.dogecv.CameraViewDisplay;

/**
 * Created by guinea on 10/5/17.
 * This is a sample opmode that demonstrates the use of an OpenCVPipeline with FTC code.
 * When the x button is pressed on controller one, the camera is set to show areas of the image
 * where a certain color is, in this case, blue.
 *
 */
@TeleOp(name="Example: Blue Vision Demo",group = "Testing")
@Disabled
public class ExampleBlueVisionDemo extends OpMode {
    ExampleBlueVision blueVision;
    @Override
    public void init() {
        blueVision = new ExampleBlueVision();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        blueVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        blueVision.setShowBlue(false);
        // start the vision system
        blueVision.enable();
    }

    @Override
    public void loop() {
        blueVision.setShowBlue(true);
        telemetry.addData("Running","");
        telemetry.addData("Order",blueVision.order);
        telemetry.update();
    }

    public void stop() {
        // stop the vision system
        blueVision.disable();
    }
}
