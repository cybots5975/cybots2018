package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by kskrueger on 1/10/17.
 */

public class GlyphMech {
    DcMotorEx ArmMotor;
    CRServo BoxBelt;
    public Servo DPinch, PPinch;
    int position;

    ElapsedTime runtime  = new ElapsedTime();
    boolean run = false;

    public enum height {HIGH, MID, LOW, STORE}

    double DGrab = .390;
    double DDrop = .309;
    double PGrab = .9; //was .832
    double PDrop = .754;

    public GlyphMech(DcMotorEx ArmMotor, CRServo BoxBelt, Servo DPinch, Servo PPinch) {
        this.ArmMotor = ArmMotor;
        this.BoxBelt = BoxBelt;
        this.DPinch = DPinch;
        this.PPinch = PPinch;
    }

    public void init() {
        ArmMotor.setPower(0);
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BoxBelt.setPower(0);
        DPinch.setPosition(DDrop);
        PPinch.setPosition(PDrop);
    }

    public void setAngle(int position, double power) {
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor.setTargetPosition(position);
        ArmMotor.setPower(power);
    }

    public void grab() {
        DPinch.setPosition(DGrab);
        PPinch.setPosition(PGrab);
    }

    public void drop() {
        DPinch.setPosition(DDrop);
        PPinch.setPosition(PDrop);
    }

    public void setPosition(height height){
        switch (height) {
            case HIGH:
                position = -1400;
                break;
            case MID:
                position = -1400-325;
                break;
            case LOW:
                position = -1400-325-325;
                break;
            case STORE:
                position = 10;
                break;
        }
        setAngle(position,.9);
    }

    public void runProcess(height height) {
        /*if (pressed) {
            run = true;
            runtime.reset();
        }

        if (run&&runtime.seconds()<1) {
            //grab();
        } else {
            run = false;
        }*/

        if (runtime.seconds()>1) {
            setPosition(height);
        }
    }

}
