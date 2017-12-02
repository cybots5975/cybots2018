package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by kskrueger on 12/1/17.
 */

public class CRServoMA3 {
    public CRServoImplEx servo;

    public CRServoMA3 (HardwareMap hwMap, String configName) {
        servo = (CRServoImplEx)hwMap.crservo.get(configName);
    }

    public void holdPosition(double position) {
        
    }
}
