package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by kskrueger on 10/10/17.
 */

public class Intake{
    DcMotor intakeMotor;
    Servo leftServo, rightServo;

    public Intake (DcMotor intakeMotor, Servo leftServo, Servo rightServo) {
        this.intakeMotor = intakeMotor;
        this.leftServo = leftServo;
        this.rightServo = rightServo;
    }

    public void init() {
        intakeMotor.setPower(0);
        leftServo.setPosition(1);
        rightServo.setPosition(1);
    }

    public void store() {
        intakeMotor.setPower(0);
        leftServo.setPosition(0);
        rightServo.setPosition(1);
        //setAngle(-90,leftServo);
        //setAngle(-90,rightServo);
    }

    public void open() {
        leftServo.setPosition(.93);
        rightServo.setPosition(.3);
    }

    public void pinch() {
        leftServo.setPosition(.1);
        rightServo.setPosition(.5);
    }


    public void setAngle(int angle, Servo servo) {

    }

    private void setSideAngle(double side, int angle) {

    }

    public void setSpeed(double speed) {
        intakeMotor.setPower(-speed);
    }

    public void autoCollect() {
        //use ODS sensors to auto adjust angle
    }

}
