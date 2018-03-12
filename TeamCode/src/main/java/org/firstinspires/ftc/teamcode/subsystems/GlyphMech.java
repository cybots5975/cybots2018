package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.subsystems.GlyphMech.height.HIGH;
import static org.firstinspires.ftc.teamcode.subsystems.GlyphMech.height.LOW;
import static org.firstinspires.ftc.teamcode.subsystems.GlyphMech.height.MID;
import static org.firstinspires.ftc.teamcode.subsystems.GlyphMech.height.STORE;

/**
 * Created by kskrueger on 1/10/17.
 */

public class GlyphMech {
    private DcMotorEx ArmMotor;
    private Servo PPinch, DPinch;
    private AnalogInput boxLimit;
    private boolean stopRequest;
    public boolean zeroBusy = false;
    private int position;
    private double dumpSpeed = .9;

    ElapsedTime runtime  = new ElapsedTime();
    boolean run = false;

    public enum height {HIGH, MID, LOW, STONE, STORE}

    private double DGrab = 1;
    private double DDrop = .6;
    private double PGrab = 1; //was .832
    private double PDrop = .75;
    height savedHeight;

    public GlyphMech(DcMotorEx ArmMotor, Servo PPinch, Servo DPinch, AnalogInput boxLimit, Boolean stopRequest) {
        this.ArmMotor = ArmMotor;
        this.PPinch = PPinch;
        this.DPinch = DPinch;
        this.boxLimit = boxLimit;
        this.stopRequest = stopRequest;
    }

    public void init() {
        ArmMotor.setPower(0);
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        savedHeight = height;
        switch (height) {
            case HIGH:
                position = -1425;
                break;
            case MID:
                position = -1800;
                break;
            case LOW:
                position = -2200;
                break;
            case STONE:
                position = -300;
                break;
            case STORE:
                position = 50;
                break;
        }
        setAngle(position,dumpSpeed);
    }

    public void setPosition(int height){
        height height1 = STORE;
        switch (height) {
            case 1:
                height1 = LOW;
                break;
            case 2:
                height1 = MID;
                break;
            case 3:
                height1 = HIGH;
                break;
        }

        setPosition(height1);
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

    public boolean inPosition () {
        return (Math.abs(ArmMotor.getTargetPosition()) - Math.abs(ArmMotor.getCurrentPosition())) < 50;
    }

    public void setDumpSpeed(double dumpSpeed) {
        this.dumpSpeed = dumpSpeed;
    }

    public void disable() {
        ArmMotor.setPower(0);
    }

    public void reset() {
        DcMotor.RunMode mode = ArmMotor.getMode();
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setMode(mode);
    }

    public void zero() {
        new Thread (new Runnable()
        {
            @Override
            public void run()
            {
                boolean loop = true;
                zeroBusy = true;
                while(loop&&!stopRequest&&savedHeight==STORE) {
                    DcMotor.RunMode mode = ArmMotor.getMode();
                    ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    while(!stopRequest&&!(boxLimit.getVoltage()>.5)) {
                        ArmMotor.setPower(.2);
                    }
                    ArmMotor.setPower(0);
                    ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    ArmMotor.setMode(mode);

                    zeroBusy = false;
                    loop = false;
                }

            }
        }).start();
    }

}
