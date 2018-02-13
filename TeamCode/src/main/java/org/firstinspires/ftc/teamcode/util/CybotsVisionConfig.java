package org.firstinspires.ftc.teamcode.util;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Mat;

/**
 * Created by kskrueger for Cybots Robotics on 12/26/17.
 */

public class CybotsVisionConfig {
    private JewelDetector jewelDetector = null;
    //private JewelDetector.JewelOrder jewelOrder;

    public CybotsVisionConfig(HardwareMap hardwareMap, boolean vuforiaFrames) {
        jewelDetector = new JewelDetector();
        if (!vuforiaFrames) {
            jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        }

        //Jewel Detector Settings
        jewelDetector.areaWeight       = 0.02;
        jewelDetector.detectionMode    = JewelDetector.JewelDetectionMode.MAX_AREA; // PERFECT_AREA
        jewelDetector.perfectRatio     = 2;
        jewelDetector.perfectArea      = 2800;
        jewelDetector.areaWeight       = 0.05; // Since we're dealing with 100's of pixels
        jewelDetector.minArea          = 1100;
        jewelDetector.ratioWeight      = 30; // Since most of the time the area diffrence is a decimal place
        jewelDetector.maxDiffrence     = 100; // Since most of the time the area diffrence is a decimal place
    }

    public void vumarkFrames (Mat frame) {
        //Core.flip(frame,frame,+1);
        jewelDetector.processFrame(frame,frame);
    }

    public void enable(){
        jewelDetector.enable();
    }

    public void disable(){
        jewelDetector.disable();
    }

    public JewelDetector.JewelOrder jewelOrder() {
        return jewelDetector.getCurrentOrder();
        //return jewelDetector.getLastOrder(); should do last or current position? hmm
    }
}
