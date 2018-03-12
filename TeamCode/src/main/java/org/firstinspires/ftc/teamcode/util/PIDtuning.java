package org.firstinspires.ftc.teamcode.util;

/**
 * Created by kskrueger for Cybots Robotics on 2/16/18.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.logging.ArrayLogging;

import java.io.IOException;
import java.math.RoundingMode;
import java.text.DecimalFormat;

@TeleOp(name="PID Tuning", group="Test")
public class PIDtuning extends LinearOpMode
{
    private double p, i, d;
    private WirelessPID pidUdpReceiver;
    private Robot robot = new Robot(this);
    private int angle = 0;
    private double speed = .07;

    private ArrayLogging log = new ArrayLogging(11,10000);
    public ElapsedTime runtime = new ElapsedTime();
    private int count = 0;

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

        log.storeValue(0,0,"Count");
        log.storeValue(1,0,"Time");
        log.storeValue(2,0,"P");
        log.storeValue(3,0,"I");
        log.storeValue(4,0,"D");
        log.storeValue(5,0,"LastTime");
        log.storeValue(6,0,"Integral");
        log.storeValue(7,0,"PreviousError");
        log.storeValue(8,0,"Error");
        log.storeValue(9,0,"Out");

        log();

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

            telemetry.addData("Time",runtime.seconds());

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


    public void log() {
        new Thread(new Runnable() {
            @Override
            public void run() {
                while (!isStopRequested()) {
                    robot.imu.getHeading();

                    count += 1;

                    log.storeValueInt(0, count, count);
                    log.storeValueInt(1, count, runtime.milliseconds());
                    log.storeValueInt(2, count, robot.drive.turnPID.kP);
                    log.storeValueInt(3, count, robot.drive.turnPID.kI);
                    log.storeValueInt(4, count, robot.drive.turnPID.kD);
                    log.storeValueInt(5, count, robot.drive.turnPID.lastTime);
                    log.storeValueInt(6, count, robot.drive.turnPID.integral);
                    log.storeValueInt(7, count, robot.drive.turnPID.previousError);
                    log.storeValueInt(8, count, robot.drive.turnPID.error);
                    log.storeValueInt(9, count, robot.drive.turnPID.out);
                    log.storeValueInt(10,count, robot.imu.getHeading());

                    if (runtime.seconds()>20) {
                        try {
                            log.log("pidTesting");
                        } catch (IOException e) {
                            e.printStackTrace();
                        }
                        telemetry.addData("Written","");
                    }
                    telemetry.update();
                }
            }
        }).start();
    }
}