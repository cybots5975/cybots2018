package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Sensors.MA3Encoder;

/**
 * Created by kskrueger on 10/22/17.
 */

@Autonomous(name="Test Enc", group="Test")
public class TestEncoder extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();

    public AnalogInput DSensor1, DSensor2, PSensor1, PSensor2;

    //HardwareSwerveV1 robot = new HardwareSwerveV1(); //use the SwerveV1 hardware file to configure

    @Override
    public void runOpMode() {
        MA3Encoder DSEncoder = new MA3Encoder(hardwareMap,"EncoderTest");
        DSEncoder.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStarted()) {
            while (opModeIsActive() ) {


                telemetry.addData("CurrentBelow0",DSEncoder.currentBelow0);

                telemetry.addData("PreviousBelow0",DSEncoder.previousBelow0);

                telemetry.addData("Absolute",DSEncoder.getAbsolute());

                telemetry.addData("Incremental",DSEncoder.getIncremental());

                telemetry.update();
            }
        }
    }
}
