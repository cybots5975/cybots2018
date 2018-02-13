package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * Created by kskrueger on 1/10/17.
 */

public class RelicArm {
    private DcMotor ExtendMotor;
    private ServoImplEx Grab, Pivot;

    private double clawGrab = 0;
    private double clawInit = .887;
    private double clawRelease = .65;
    private double pivotDown = .96;
    private double pivotUp = .536;

    RelicArm(DcMotor ExtendMotor, ServoImplEx RelicGrab, ServoImplEx RelicPivot) {
        this.ExtendMotor = ExtendMotor;
        this.Grab= RelicGrab;
        this.Pivot = RelicPivot;
    }

    public void init() {
        ExtendMotor.setPower(0);
        ExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Grab.setPosition(clawInit);
        Pivot.setPosition(pivotDown);
    }

    public void grab() {
        Grab.setPosition(clawGrab);
    }

    public void release() {
        Grab.setPosition(clawRelease);
    }

    public void clawInit() {
        Grab.setPosition(clawInit);
    }

    public void pivotUp() {
        Pivot.setPosition(pivotUp);
    }

    public void pivotDown() {
        Pivot.setPosition(pivotDown);
    }

    public void extendArm(double power) {
        ExtendMotor.setPower(power);
    }

    public void disable() {
        ExtendMotor.setPower(0);
        Grab.setPwmDisable();
        Pivot.setPwmDisable();
    }

}
