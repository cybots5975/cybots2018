package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by kskrueger on 12/1/17.
 */

public class Outtake {
    private DcMotor armMotor1, armMotor2;
    private CRServoImplEx clampLeft, clampRight, belt;

    public Outtake (DcMotor flipMotor1, DcMotor flipMotor2, CRServoImplEx clampLeft, CRServoImplEx clampRight, CRServoImplEx belt) {
        this.armMotor1 = flipMotor1;
        this.armMotor2 = flipMotor2;
        this.clampLeft = clampLeft;
        this.clampRight = clampRight;
        this.belt = belt;

        armMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        clampRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void init() {
        armMotor1.setPower(0);
        armMotor2.setPower(0);
        clampLeft.setPower(0);
        clampRight.setPower(0);
        belt.setPower(0);
    }

    public void stop() {
        armMotor1.setPower(0);
        armMotor2.setPower(0);
        clampLeft.setPower(0);
        clampRight.setPower(0);
        belt.setPower(0);
    }
}