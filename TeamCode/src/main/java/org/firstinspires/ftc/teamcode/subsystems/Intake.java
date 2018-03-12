package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by kskrueger on 10/10/17.
 */

public class Intake{
    DcMotor intakeMotor;

    public Intake (DcMotor intakeMotor) {
        this.intakeMotor = intakeMotor;
        //this.leftServo = leftServo;
        //this.rightServo = rightServo;
    }

    public void init() {
        intakeMotor.setPower(0);
        //leftServo.setPosition(1);
        //rightServo.setPosition(1);
    }

    public void store() {
        intakeMotor.setPower(0);
        //leftServo.setPosition(.9);
        //rightServo.setPosition(.2);
    }

    public void open() {
        //leftServo.setPosition(.156);
        //rightServo.setPosition(.978);
    }

    public void multiGlyph() {
        //leftServo.setPosition(.215);
        //rightServo.setPosition(.875);
    }

    public void pinch() {
        //leftServo.setPosition(.1);
        //rightServo.setPosition(.5);
    }

    public void auton() {
        //leftServo.setPosition(.658);
        //rightServo.setPosition(.696);
    }

    public void setAngle(double leftPosition, double rightPosition) {
        //leftServo.setPosition(leftPosition);
        //rightServo.setPosition(rightPosition);
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
    }

}
