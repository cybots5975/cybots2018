package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by kskrueger on 11/4/17.
 */

public class AbsEncoder {
    private AnalogInput encoder;

    public AbsEncoder (HardwareMap hwMap, String name) {
        encoder = hwMap.analogInput.get(name);
    }

    public double voltage{
        return encoder.getVoltage();
    }

}
