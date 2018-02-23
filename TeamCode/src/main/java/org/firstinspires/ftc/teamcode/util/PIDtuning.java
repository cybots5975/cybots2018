package org.firstinspires.ftc.teamcode.util;

/**
 * Created by kskrueger for Cybots Robotics on 2/16/18.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.math.RoundingMode;
import java.text.DecimalFormat;

@TeleOp(name="PID Tuning", group="Test")
public class PIDtuning extends LinearOpMode
{
    private double p, i, d;
    private WirelessPID pidUdpReceiver;
    private Robot robot = new Robot(this);
    private int angle = 0;
    private double speed = .5;

    @Override
    public void runOpMode() throws InterruptedException
    {
        /*
         * Initialize the network receiver
         */
        pidUdpReceiver = new WirelessPID();
        pidUdpReceiver.beginListening();

        telemetry.setMsTransmissionInterval(50);
        robot.init();
        waitForStart();

        /*
         * Main loop
         */
        while (opModeIsActive())
        {
            updateCoefficients();

            if (gamepad1.a) {
                angle = 0;
            } else if (gamepad1.b) {
                angle = 90;
            } else if (gamepad1.y) {
                angle = 180;
            } else if (gamepad1.x) {
                angle = 270;
            }

            if (gamepad1.dpad_up) {
                speed += .005;
            } else if (gamepad1.dpad_down) {
                speed -= .005;
            }

            robot.drive.turnPID.setVariables(p,i,d);
            robot.drive.gyroTurn(.5,angle,1);

            telemetry.addData("Speed",speed);
            telemetry.addData("Angle",angle);
            telemetry.addData("P", formatVal(p));
            telemetry.addData("I", formatVal(i));
            telemetry.addData("D", formatVal(d));
            telemetry.update();
        }

        /*
         * Make sure to call this at the end of your OpMode or else
         * the receiver won't work again until the app is restarted
         */
        pidUdpReceiver.shutdown();
    }

    private void updateCoefficients()
    {
        p = pidUdpReceiver.getP();
        i = pidUdpReceiver.getI();
        d = pidUdpReceiver.getD();
    }

    /*
     * This method formats a raw double for nice display on the DS telemetry
     */
    private String formatVal(double val)
    {
        DecimalFormat df = new DecimalFormat("#.###");
        df.setRoundingMode(RoundingMode.CEILING);
        return df.format(val);
    }
}