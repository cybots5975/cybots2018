package org.firstinspires.ftc.teamcode.test.examples;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="DogeCV Jewel Detector", group="DogeCV")
public class JewelOpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private JewelDetector jewelDetector = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        jewelDetector = new JewelDetector();
        jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        //Jewel Detector Settings
        jewelDetector.areaWeight       = 0.02;
        jewelDetector.detectionMode    = JewelDetector.JewelDetectionMode.MAX_AREA; // PERFECT_AREA
        jewelDetector.perfectRatio     = 2;
        jewelDetector.perfectArea      = 2800;
        jewelDetector.areaWeight       = 0.05; // Since we're dealing with 100's of pixels
        jewelDetector.minArea          = 1100;
        jewelDetector.ratioWeight      = 30; // Since most of the time the area diffrence is a decimal place
        jewelDetector.maxDiffrence     = 100; // Since most of the time the area diffrence is a decimal place

        jewelDetector.enable();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Initialized.");
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addData("Current Order", "Jewel Order: " + jewelDetector.getCurrentOrder().toString()); // Current Result
        telemetry.addData("Last Order", "Jewel Order: " + jewelDetector.getLastOrder().toString()); // Last Known Result
    }

    @Override
    public void stop() {
        jewelDetector.disable();
    }
}