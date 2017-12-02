package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by kskrueger on 12/1/17.
 */

public class JewelArm {
    Servo arm;

    public JewelArm (Servo arm) {
        this.arm = arm;
    }

    public void drop() {
        arm.setPosition(0);
    }

    public void store() {
        arm.setPosition(1);
    }
}
