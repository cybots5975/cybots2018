package org.firstinspires.ftc.teamcode.test.examples;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.GenericDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="DogeCV Relic Detector", group="DogeCV")
public class RelicOpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private GenericDetector relicDetector = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        relicDetector = new GenericDetector();
        relicDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        //Jewel Detector Settings
        relicDetector.areaWeight       = 0.01;
        relicDetector.detectionMode    = GenericDetector.GenericDetectionMode.MAX_AREA; // PERFECT_AREA
        relicDetector.perfectRatio     = 1;
        relicDetector.perfectArea      = 2800;
        relicDetector.areaWeight       = 0.05; // Since we're dealing with 100's of pixels
        relicDetector.minArea          = 200;

        relicDetector.enable();
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

        telemetry.addData("Relic Position", relicDetector.getLocation()); // Current Result
    }

    @Override
    public void stop() {
        relicDetector.disable();
    }
}