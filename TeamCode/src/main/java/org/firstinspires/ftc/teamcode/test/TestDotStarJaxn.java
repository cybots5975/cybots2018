package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.DotStar;

/**
 * Created by kskrueger for Cybots Robotics on 2/17/18.
 */

@TeleOp (name="Test Dot Star",group="Test")
public class TestDotStarJaxn extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException
    {

        DotStar frogDotStar = new DotStar(hardwareMap, "ci", "di");

        waitForStart();

        frogDotStar.setEntireStrip((byte)0, (byte)180, (byte)0);
    }
}
