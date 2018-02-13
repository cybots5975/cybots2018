package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * Created by kskrueger on 10/10/17.
 */

public class Intake{
    DcMotor intakeMotor;
    ServoImplEx leftServo, rightServo;

    public Intake (DcMotor intakeMotor, ServoImplEx leftServo, ServoImplEx rightServo) {
        this.intakeMotor = intakeMotor;
        this.leftServo = leftServo;
        this.rightServo = rightServo;
    }

    public void init() {
        leftServo.setPwmEnable();
        rightServo.setPwmEnable();
        intakeMotor.setPower(0);
        leftServo.setPosition(1);
        rightServo.setPosition(1);
    }

    public void store() {
        leftServo.setPwmEnable();
        rightServo.setPwmEnable();
        intakeMotor.setPower(0);
        leftServo.setPosition(.884);
        rightServo.setPosition(.455);
    }

    public void open() {
        leftServo.setPwmEnable();
        rightServo.setPwmEnable();
        leftServo.setPosition(.156);
        rightServo.setPosition(.978);
    }

    public void pinch() {
        leftServo.setPwmEnable();
        rightServo.setPwmEnable();
        leftServo.setPosition(.1);
        rightServo.setPosition(.5);
    }

    public void auton() {
        leftServo.setPwmEnable();
        rightServo.setPwmEnable();
        leftServo.setPosition(.658);
        rightServo.setPosition(.696);
    }

    public void setAngle(double leftPosition, double rightPosition) {
        leftServo.setPwmEnable();
        rightServo.setPwmEnable();
        leftServo.setPosition(leftPosition);
        rightServo.setPosition(rightPosition);
    }

    private void setSideAngle(double side, int angle) {

    }

    public void setSpeed(double speed) {
        intakeMotor.setPower(-speed);
    }

    public void autoCollect() {
        //use ODS sensors to auto adjust angle
    }

    public void disable() {
        leftServo.setPwmDisable();
        rightServo.setPwmDisable();
    }

}
