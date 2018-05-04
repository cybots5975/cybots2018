package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.multiGlyph.cipher.GlyphColorDetector;

/**
 * Created by kskrueger on 10/22/17.
 */

@Autonomous(name="Test Cipher Colors Auto", group="Template")
//@Disabled
public class TestCipherColorsAuto extends LinearOpMode{
    private Robot robot = new Robot(this);
    private boolean loop = true;

    private GlyphColorDetector front1, front2, back1, back2;

    @Override
    public void runOpMode() {
        robot.init();
        robot.drive.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.drive.zeroEncoders();

        front1 = new GlyphColorDetector(robot.glyphColor1);
        front2 = new GlyphColorDetector(robot.glyphColor3);
        back1 = new GlyphColorDetector(robot.glyphColor2);
        back2 = new GlyphColorDetector(robot.glyphColor2);

        telemetry.addData("Position:", robot.prefs.read("postion"));
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()&&loop&&!isStopRequested()) {

            telemetry.addData("Front1 (1)",front1.glyphColor().toString());
            telemetry.addData("Front2 (3)",front2.glyphColor().toString());
            telemetry.addData("Back1 (2)",back1.glyphColor().toString());
            telemetry.addData("Back2 (4)",back2.glyphColor().toString());

            telemetry.update();

            //loop = false;
        }
    }
}
