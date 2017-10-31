package org.firstinspires.ftc.teamcode.OthersCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by kskrueger on 10/29/17.
 */
@TeleOp(name="Test Amperage",group="Test")
public class TestAmperage extends LinearOpMode{

        @Override
        public void runOpMode() {

            DcMotor motor = hardwareMap.dcMotor.get("test");
            Servo servo = hardwareMap.servo.get("testS");
            LynxDcMotorController2 controller = (LynxDcMotorController2)motor.getController();

            telemetry.addData("Status", "Initialized");
            telemetry.update();

            //wait for the program to start (operator presses PLAY)
            waitForStart();

            //run until the end (operator presses STOP)
            while (opModeIsActive()) {

                motor.setPower(gamepad1.left_stick_y);
                servo.setPosition(.5);

                double current = controller.getBatteryCurrent();
                telemetry.addLine("Current: " + current);
                telemetry.update();
            }
        }
    }