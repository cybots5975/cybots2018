package org.firstinspires.ftc.teamcode.test.othersCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Created by kskrueger on 10/29/17.
 */
@TeleOp(name="Test Amperage",group="Test")
@Disabled
public class TestAmperage extends LinearOpMode{

        @Override
        public void runOpMode() {

            DcMotor motor = hardwareMap.dcMotor.get("test");
            Servo servo = hardwareMap.servo.get("testS");
            LynxDcMotorController2 controller = (LynxDcMotorController2)motor.getController();

            DcMotorEx testEx = (DcMotorEx) hardwareMap.dcMotor.get("Test");

            telemetry.addData("Status", "Initialized");
            telemetry.update();

            //wait for the program to start (operator presses PLAY)
            waitForStart();

            //run until the end (operator presses STOP)
            while (opModeIsActive()) {

                //motor.setPower(gamepad1.left_stick_y);
                servo.setPosition(.5);

                testEx.setVelocity(50, AngleUnit.DEGREES);

                double current = controller.getBatteryCurrent();
                telemetry.addLine("Current: " + current);
                telemetry.update();
            }
        }
    }