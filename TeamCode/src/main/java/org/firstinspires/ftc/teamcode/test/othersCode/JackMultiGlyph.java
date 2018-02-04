package org.firstinspires.ftc.teamcode.test.othersCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by kskrueger for Cybots Robotics on 2/1/18.
 */

public class JackMultiGlyph extends LinearOpMode {

    boolean grabbedGlyph;

    enum multiGlyph {GlyphGoTo, GlyphIntake, GlyphStuck}

    int i = 0;
    ElapsedTime et;


    @Override
    public void runOpMode() {
        /*

        while (true) {
            switch (multiGlyph) {
                case GlyphGoTo: {
                    grabbedGlyph = false;
                    robot.in1.setPower(-.2); //need to ramp up in
                    robot.in2.setPower(-.2);
                    sleep(100);
                    robot.in1.setPower(-.4); //need to ramp up in
                    robot.in2.setPower(-.4);
                    sleep(100);
                    robot.in1.setPower(-.8); //need to ramp up in
                    robot.in2.setPower(-.8);
                    sleep(100);
                    robot.in1.setPower(-1); //need to ramp up in
                    robot.in2.setPower(-1);
                    sleep(100);
                    robot.belt.setPower(-1);
                    powerDrive(.2);
                    multiGlyph = MultiGlyph.GlyphIntake;
                }
                break;

                case GlyphIntake: {
                    double maxCurrent = Math.max(robot.in1.getCurrentDraw(), robot.in2.getCurrentDraw());

                    double maxRange = Math.max((robot.glyph1.getDistance(DistanceUnit.CM)), robot.glyph2.getDistance(DistanceUnit.CM));

                    double minRange = Math.min((robot.glyph1.getDistance(DistanceUnit.CM)), robot.glyph2.getDistance(DistanceUnit.CM));

                    double glyph1Range = robot.glyph1.getDistance(DistanceUnit.CM);

                    double glyph2Range = robot.glyph2.getDistance(DistanceUnit.CM);

                    double glyphThreshold = 1000;

                    double stuckThreshold = glyphThreshold + 0; //and time is also used

                    double stuckTime = 1200;

                    if (maxCurrent < glyphThreshold && opModeIsActive() || i >= 2) {
                        et.reset(); //reset time
                        grabbedGlyph = false; //
                        multiGlyph = MultiGlyph.GlyphGoTo;
                        if (i >= 2) {
                            state = State.Stop;
                        }
                        // already moving forward
                    }
                    if (maxCurrent >= stuckThreshold && et.milliseconds() < stuckTime && i < 2 && opModeIsActive()) {
                        powerDrive(0);
                        if (!grabbedGlyph && glyphRange1() || glyphRange2() && i < 1) {
                            i++; //If this triggers the one below can not trigger, cause grabbedGlyph = true
                            grabbedGlyph = true;
                        }
                        if (!grabbedGlyph && glyphRange1() && glyphRange2() && i >= 1) {
                            i++;
                            grabbedGlyph = true;
                        }
                        if (grabbedGlyph && i == 1) {
                            gyroDriveReverse(-.2, 0, 100); //Reverse and try again?
                            sleep(200);
                            grabbedGlyph = false;
                        }
                        if (minRange >= 6 && maxRange <= 11) {
                            i = 2; //glyphs have been gotten?
                        }
                        if (i >= 2) {
                            state = State.Stop; //Stoped so should exit
                        }

                    }
                    //Stuck so should exit.
                    if (maxCurrent >= stuckThreshold && et.milliseconds() >= stuckTime) {//glyph is stuck, changing state
                        multiGlyph = MultiGlyph.GlyphStuck;
                    }


                }
                break;

                case GlyphStuck: {
                    double unstuckThresh = 800;
                    if (grabbedGlyph) {
                        et.reset();
                        grabbedGlyph = false;
                        i--;
                    }
                    robot.in1.setPower(1);
                    robot.in2.setPower(1);
                    //robot.belt.setPower(1);
                    powerDrive(-.2);
                    if (et.milliseconds() > unstuckThresh) {
                        multiGlyph = MultiGlyph.GlyphGoTo;
                    }

                }
                break;
            }
        }
    }

    public boolean glyphRange1 () {
        double maxRange = Math.max((robot.glyph1.getDistance(DistanceUnit.CM)), robot.glyph2.getDistance(DistanceUnit.CM));

        double minRange = Math.min((robot.glyph1.getDistance(DistanceUnit.CM)), robot.glyph2.getDistance(DistanceUnit.CM));

        double glyph1Range = robot.glyph1.getDistance(DistanceUnit.CM);

        double glyph2Range = robot.glyph2.getDistance(DistanceUnit.CM);

        return (glyph1Range <= 11 && glyph1Range >= 5);
    }

    public boolean glyphRange2 () {
        double maxRange = Math.max((robot.glyph1.getDistance(DistanceUnit.CM)), robot.glyph2.getDistance(DistanceUnit.CM));

        double minRange = Math.min((robot.glyph1.getDistance(DistanceUnit.CM)), robot.glyph2.getDistance(DistanceUnit.CM));

        double glyph1Range = robot.glyph1.getDistance(DistanceUnit.CM);

        double glyph2Range = robot.glyph2.getDistance(DistanceUnit.CM);

        return (glyph2Range >= 5 && glyph2Range <= 11);
    }

    */
    }
}
