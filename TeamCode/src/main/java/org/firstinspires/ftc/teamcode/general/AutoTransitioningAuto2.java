package org.firstinspires.ftc.teamcode.general;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "AutoTransitioningAuto2")
public class AutoTransitioningAuto2 extends OpMode {
    @Override
    public void init() {
        telemetry.addData("Initializing Here", true);

        AutoTransitioner.transitionOnStop(this, "Teleop V1");
        // AutoTransitioner used in init()
    }

    @Override
    public void loop() {
        telemetry.addData("Timer", getRuntime());
    }
}
