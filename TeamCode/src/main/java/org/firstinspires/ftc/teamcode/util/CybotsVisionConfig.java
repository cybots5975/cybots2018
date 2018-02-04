package org.firstinspires.ftc.teamcode.util;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Core;
import org.opencv.core.Mat;

/**
 * Created by kskrueger for Cybots Robotics on 12/26/17.
 */

public class CybotsVisionConfig {
    private JewelDetector jewelDetector = null;
    private JewelDetector.JewelOrder jewelOrder;

    public CybotsVisionConfig(HardwareMap hardwareMap, boolean vuforiaFrames) {
        jewelDetector = new JewelDetector();
        if (!vuforiaFrames) {
            jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        }

        //Jewel Detector Settings
        jewelDetector.areaWeight = 0.02;
        jewelDetector.detectionMode = JewelDetector.JewelDetectionMode.MAX_AREA; // PERFECT_AREA
        //jewelDetector.perfectArea = 6500; <- Needed for PERFECT_AREA
        jewelDetector.debugContours = true;
        jewelDetector.maxDiffrence = 200; //old 15
        jewelDetector.ratioWeight = 30; //old 15
        jewelDetector.minArea = 700;
    }

    public void vumarkFrames (Mat frame) {
        Core.flip(frame,frame,+1);
        jewelDetector.processFrame(frame,frame);
    }

    public void enable(){
        jewelDetector.enable();
    }

    public void disable(){
        jewelDetector.disable();
    }

    public JewelDetector.JewelOrder jewelOrder() {
        return jewelDetector.getLastOrder();
        //return jewelDetector.getCurrentOrder(); todo switch to current position?
    }
}
