package org.firstinspires.ftc.teamcode.test.monkeyCopy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.logging.ArrayLogging;

import java.io.IOException;

@TeleOp(name="Record Times", group="Test")
public class MonkeySeeTesting extends LinearOpMode {
    private ArrayLogging log = new ArrayLogging(16,10000);
    private int count = 0;
    public ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx Motor1, Motor2, Motor3, Motor4;
    private ServoImplEx S1, S2, S3, S4, S5, S6;

    @Override
    public void runOpMode() {
        //robot.init();
        Motor1 = (DcMotorEx) hardwareMap.dcMotor.get("m1");
        Motor2 = (DcMotorEx) hardwareMap.dcMotor.get("m2");
        Motor3 = (DcMotorEx) hardwareMap.dcMotor.get("m3");
        Motor4 = (DcMotorEx) hardwareMap.dcMotor.get("m4");

        S1 = (ServoImplEx) hardwareMap.servo.get("s1");
        S2 = (ServoImplEx) hardwareMap.servo.get("s2");
        S3 = (ServoImplEx) hardwareMap.servo.get("s3");
        S4 = (ServoImplEx) hardwareMap.servo.get("s4");
        S5 = (ServoImplEx) hardwareMap.servo.get("s5");
        S6 = (ServoImplEx) hardwareMap.servo.get("s6");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        initializeLogging();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            log(runtime.milliseconds());
            Motor1.setPower(gamepad1.left_stick_y);
            Motor2.setPower(gamepad1.left_stick_x);
            Motor2.setPower(gamepad1.right_stick_y);
            Motor2.setPower(gamepad1.right_stick_x);

            S1.setPosition(gamepad1.left_trigger);
            S2.setPosition(gamepad1.left_trigger);
            S3.setPosition(gamepad1.left_trigger);
            S4.setPosition(gamepad1.right_trigger);
            S5.setPosition(gamepad1.right_trigger);
            S6.setPosition(gamepad1.right_trigger);

            if (runtime.seconds()>10) {
                try {
                    log.log("times");
                } catch (IOException e) {
                    e.printStackTrace();
                }
                telemetry.addData("Written","");
            }

            telemetry.update();
        }
    }

    public void initializeLogging() {
        log.storeValue(0, 0, "Count #");
        log.storeValue(1, 0, "Time");
        log.storeValue(2, 0, "Position");
        log.storeValue(3, 0, "Speed");
        log.storeValue(4, 0, "Position2");
        log.storeValue(5, 0, "Speed2");
        log.storeValue(6, 0, "Position2");
        log.storeValue(7, 0, "Speed2");
        log.storeValue(8, 0, "Position2");
        log.storeValue(9, 0, "Speed2");
    }

    public void log(double time) {
        count += 1;

        log.storeValueInt(0, count, count);
        log.storeValueInt(1, count, time);
        log.storeValueInt(2, count, Motor1.getTargetPosition());
        log.storeValueInt(3, count, Motor1.getPower());
        log.storeValueInt(4, count, Motor2.getTargetPosition());
        log.storeValueInt(5, count, Motor2.getPower());
        log.storeValueInt(6, count, Motor3.getTargetPosition());
        log.storeValueInt(7, count, Motor3.getPower());
        log.storeValueInt(8, count, Motor4.getTargetPosition());
        log.storeValueInt(9, count, Motor4.getPower());

        log.storeValueInt(10, count, S1.getPosition());
        log.storeValueInt(11, count, S2.getPosition());
        log.storeValueInt(12, count, S3.getPosition());
        log.storeValueInt(13, count, S4.getPosition());
        log.storeValueInt(14, count, S5.getPosition());
        log.storeValueInt(15, count, S6.getPosition());
    }

}