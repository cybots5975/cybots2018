package org.firstinspires.ftc.teamcode.subsystems.sensors.ADS1015;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Test the ADS1015 ADC driver
 */
@TeleOp(name = "ADS1015 Test jaxn")
public class ADS1015Test extends OpMode {
    ADS1015 voltageSensor;

    @Override
    public void init() {
        voltageSensor = hardwareMap.get(ADS1015.class, "vsens");
    }

    @Override
    public void loop() {
        telemetry.addData("Voltage on A0", voltageSensor.getVoltage(0));
        telemetry.addData("Voltage on A1", voltageSensor.getVoltage(1));
        telemetry.addData("Voltage on A2", voltageSensor.getVoltage(2));
        telemetry.addData("Voltage on A3", voltageSensor.getVoltage(3));
    }
}