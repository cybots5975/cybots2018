package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * Created by kskrueger on 10/22/17.
 */

@Autonomous(name="MultiGlyph Test Auto", group="Test")
public class TestMultiLogic extends LinearOpMode{
    private Robot robot = new Robot(this);
    private boolean loop = true;
    private int angle = 0;
    private int distanceCounts = 350;

    @Override
    public void runOpMode() {
        robot.init();
        robot.drive.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.drive.zeroEncoders();
        robot.intake.multiGlyph();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()&&loop&&!isStopRequested()) {
            telemetry.addData("Position:", robot.prefs.read("postion"));
            telemetry.update();
            robot.intake.setSpeed(1);
            robot.intake.multiGlyph();
            while ((!frontGlyph(2,20))
                    &&!jammed(6000)
                    &&!isStopRequested()) {
                robot.drive.encoderFwd(.3, distanceCounts);
                robot.pause(.25);
                robot.drive.encoderFwd(-.4, 0);
                angle += 7;
                distanceCounts += 100;
                robot.drive.gyroTurn(.55,angle,1);
                telemetry.addData("Back Sensor",robot.glyphDistance2.getDistance(DistanceUnit.CM));
                telemetry.addData("Front Sensor",robot.glyphDistance1.getDistance(DistanceUnit.CM));
                telemetry.addData("Intake Current Draw mA",robot.IntakeMotor.getCurrentDraw());
                telemetry.update();
            }
            if (jammed(6000)) {
                robot.intake.setSpeed(-1);
                robot.pause(.5);
                robot.intake.setSpeed(1);
            }
            while ((!((backGlyph(2,20))
                    &&(frontGlyph(2,20))))
                    &&!jammed(6000)
                    &&!isStopRequested()) {
                robot.drive.encoderFwd(.3, distanceCounts);
                robot.pause(.25);
                robot.drive.encoderFwd(-.4, 0);
                angle += 7;
                distanceCounts += 100;
                robot.drive.gyroTurn(.55,angle,1);
                telemetry.addData("Intake Current Draw mA",robot.IntakeMotor.getCurrentDraw());
                telemetry.update();
            }if (jammed(6000)) {
                robot.intake.setSpeed(-1);
                robot.pause(.5);
            }
            //CYCLE BACK TO REPEAT THIS STEP
            robot.pause(.5);
            robot.intake.setSpeed(0);
            robot.drive.gyroTurn(.3,0,1);

            loop = false;
        }
    }

    private void scorePosition() {
        switch (robot.prefs.read("position")) {
            case "RED CLOSE":
                redClose();
                break;
            case "RED FAR":
                redFar();
                break;
            case "BLUE CLOSE":
                blueClose();
                break;
            case "BLUE FAR":
                blueFar();
                break;
        }
    }

    private void redClose(){

    }

    private void redFar(){

    }

    private void blueClose(){

    }

    private void blueFar(){

    }

    private void multiGlyph() {
        int glyphCount = 0;
        ElapsedTime runtime  = new ElapsedTime();
        ElapsedTime stallTime  = new ElapsedTime();

        //run drive code to get to pile here

        while (glyphCount<2 && runtime.seconds()<26) {
            if (jammed(5000)) {
                stallTime.reset();
                //switch state to jammed state
                //or reverse intake and such right here
            } else {
                //count number of glyphs
                if (frontGlyph(4, 20) && backGlyph(4, 20)) {
                    //do stuff for glyph count 2 here
                    //program will exit after the glyphCount is set to 2
                    glyphCount = 2;

                } else {
                    if (frontGlyph(4, 20) || backGlyph(4, 20)) {
                        //do glyph count 1 stuff here
                        glyphCount = 1;

                    } else {
                        //glyph count 0 stuff here
                        glyphCount = 0;

                    }
                }
            }

        }
    }

    private boolean frontGlyph(int minDistance, int maxDistance) {
        int sensorValueHere = 0;
        return (robot.glyphDistance1.getDistance(DistanceUnit.CM) >= minDistance
                && robot.glyphDistance1.getDistance(DistanceUnit.CM) <= maxDistance);
    }

    private boolean backGlyph(int minDistance, int maxDistance) {
        int sensorValueHere = 0;
        return (robot.glyphDistance2.getDistance(DistanceUnit.CM) >= minDistance
                && robot.glyphDistance2.getDistance(DistanceUnit.CM) <= maxDistance);
    }

    private boolean jammed(int jamCurrentDraw) {
        return (robot.IntakeMotor.getCurrentDraw() > jamCurrentDraw);
    }

}
