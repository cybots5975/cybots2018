package org.firstinspires.ftc.teamcode.util.vuforia;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public interface VuMark {
    void init(HardwareMap hardwareMap, VuforiaLocalizer.CameraDirection direction, boolean showView);
    String getColumn();
}
