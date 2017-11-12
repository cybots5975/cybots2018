/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.oldRobots;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.Range;


/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B but
 * tons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TOM: Teleop", group="TOM")
//@Disabled
public class TOMteleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareTOM robot           = new HardwareTOM();              // Use a K9'shardware
    double          armPosition     = robot.ARM_HOME;                   // Servo safe position
    double          clawPosition    = robot.CLAW_HOME;                  // Servo safe position
    final double    CLAW_SPEED      = 0.01 ;                            // sets rate to move servo
    final double    ARM_SPEED       = 0.01 ;                            // sets rate to move servo
    double  pos1 = 0;
    double  pos2 = 0;
    double  speed = 0;
    double  count = 0;
    double power = 1;

    int turretPosition = 0;

    double chutePosition = .299;

    public double dsRack1_Min = 129;
    public double dsRack1_Max = 22; //y goes out
    public double dsRack2_Min = 99;
    public double dsRack2_Max = 207; //a goes out

    public double psRack1_Min = 109;
    public double psRack1_Max = 222; //a goes out
    public double psRack2_Min = 119;
    public double psRack2_Max = 233; //a goes out

    public double xTrack;
    public double yTrack;

    ModernRoboticsI2cGyro gyro            = null; // Gyro sensor used for turning
    ModernRoboticsI2cColorSensor lineColor   = null; // Bottom facing color sensor to detect lines
    ModernRoboticsAnalogOpticalDistanceSensor dsODS1 = null;
    ModernRoboticsI2cRangeSensor dsRange1 = null;
    ModernRoboticsI2cRangeSensor psRange1 = null;
    ModernRoboticsI2cColorSensor dsColor1 = null;
    AnalogInput ballLoad;

    @Override //init code
    public void runOpMode() {
        double left;
        double right;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        lineColor = (ModernRoboticsI2cColorSensor)hardwareMap.colorSensor.get("lineColor");
        lineColor.setI2cAddress(I2cAddr.create8bit(0x3c));
        lineColor.enableLed(true); // Turn the LED on
        dsODS1 = (ModernRoboticsAnalogOpticalDistanceSensor)hardwareMap.opticalDistanceSensor.get("dsODS1");

        //psRange1.setI2cAddress(I2cAddr.create8bit(0x28));
        psRange1 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "psRange1");
        psRange1.setI2cAddress(I2cAddr.create8bit(0x28));

        //dsRange1.setI2cAddress(I2cAddr.create8bit(0x30));
        dsRange1 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "dsRange1");
        dsRange1.setI2cAddress(I2cAddr.create8bit(0x30));

        dsColor1 = (ModernRoboticsI2cColorSensor)hardwareMap.colorSensor.get("dsColor1");
        dsColor1.setI2cAddress(I2cAddr.create8bit(0x10));
        dsColor1.enableLed(false);

        ballLoad = hardwareMap.analogInput.get("ballLoad");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        robot.turretMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.driveMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.passMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.driveMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.passMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.sweepMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.turretMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shootMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shootMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.turretMotor1.setTargetPosition(turretPosition);
        robot.turretMotor1.setPower(.25);

        moveRack(1,0);
        moveRack(2,0);
        moveRack(3,0);
        moveRack(4,0);

        robot.ballLift.setPosition(.5);
        robot.chute.setPosition(.84);

        /*
        robot.driveMotor1.setMaxSpeed(2500); //sets top speed to 3.5 ft/sec based on 4in wheel and 3:1 ratio
        robot.passMotor1.setMaxSpeed(2500);
        robot.driveMotor2.setMaxSpeed(2500);
        robot.passMotor2.setMaxSpeed(2500);
*/

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;

            //clip the right/left values so that the values never exceed +/- 1
            right = Range.clip(right, -1, 1);   //clip right driver power input to maximum of 1 and minimum of -1
            left = Range.clip(left, -1, 1);     //clip left driver power input to maximum of 1 and minimum of -1

            //scale the joystick value to make it easier to control
            //the robot moves more precisely at slower speeds.
            right = (float)scaleInput(right);   //scale right drive power
            left =  (float)scaleInput(left);    //scale left drive power

            robot.driveMotor1.setPower(left);
            robot.passMotor1.setPower(right);
            robot.driveMotor2.setPower(left);
            robot.passMotor2.setPower(right);

            if (gamepad1.dpad_up) {
                power = power + .01;
            } else if (gamepad1.dpad_down ) {
                power = power - .01;
            }

            if(gamepad1.a) {
                robot.shootMotor1.setPower(power);
                robot.shootMotor2.setPower(power);


                //moveRack(1,10);
                //moveRack(3,10);

                /*dsRack1_Min = dsRack1_Min + 1;
                dsRack2_Min = dsRack2_Min + 1;
                psRack1_Min = psRack1_Min + 1;
                psRack2_Min = psRack2_Min + 1;*/

            }
            if(gamepad1.b)  {
                robot.shootMotor1.setPower(0);
                robot.shootMotor2.setPower(0);

                //moveRack(2,10);
                //moveRack(4,10);

                /*dsRack1_Min = dsRack1_Min - 1;
                dsRack2_Min = dsRack2_Min - 1;
                psRack1_Min = psRack1_Min - 1;
                psRack2_Min = psRack2_Min - 1;*/
            }


            if(gamepad1.y)  {
                robot.sweepMotor1.setPower(1);

                /*moveRack(1,0);
                moveRack(2,0);
                moveRack(3,0);
                moveRack(4,0);
*/
                /*dsRack1_Min = dsRack1_Min - 1;
                dsRack2_Min = dsRack2_Min - 1;
                psRack1_Min = psRack1_Min - 1;
                psRack2_Min = psRack2_Min - 1;*/
            } else {
                robot.sweepMotor1.setPower(0);
            }

            if (gamepad1.x) {
                robot.ballLift.setPosition(0);
            } else {
                if (ballLoad.getVoltage()>3) {
                    robot.ballLift.setPosition(.5);
                } else {
                    robot.ballLift.setPosition(0);
                }
            }

            if (gamepad1.dpad_left) {
                chutePosition = chutePosition + .01;
            }
            if (gamepad1.dpad_right) {
                chutePosition = chutePosition - .01;
            }

            if (gamepad1.left_bumper) {
                moveRack(1,10);
                moveRack(3,10);
            } else {
                moveRack(1,0);
                moveRack(3,0);
            }
            if (gamepad1.right_bumper) {
                moveRack(2,10);
                moveRack(4,10);
            } else {
                moveRack(2,0);
                moveRack(4,0);
            }

            if (gamepad1.left_trigger>.25) {
                turretPosition = turretPosition+10;
            }
            if (gamepad1.right_trigger>.25) {
                turretPosition = turretPosition-10;
            }



            /*robot.rackDS1.setPosition(1 - (dsRack1_Min / 250));
            robot.rackDS2.setPosition(1 - (dsRack2_Min / 250));
            robot.rackPS1.setPosition(1 - (psRack1_Min / 250));
            robot.rackPS2.setPosition(1 - (psRack2_Min / 250));*/

/*
            // Use gamepad Y & A raise and lower the arm
            if (gamepad1.a)
                armPosition += ARM_SPEED;
            else if (gamepad1.y)
                armPosition -= ARM_SPEED;

            // Use gamepad X & B to open and close the claw
            if (gamepad1.x)
                clawPosition += CLAW_SPEED;
            else if (gamepad1.b)
                clawPosition -= CLAW_SPEED;

            // Move both servos to new position.
            armPosition  = Range.clip(armPosition, robot.ARM_MIN_RANGE, robot.ARM_MAX_RANGE);
            robot.arm.setPosition(armPosition);
            clawPosition = Range.clip(clawPosition, robot.CLAW_MIN_RANGE, robot.CLAW_MAX_RANGE);
            robot.claw.setPosition(clawPosition);
*/

            //what the heck is happening

            robot.chute.setPosition(chutePosition);

            robot.turretMotor1.setTargetPosition(turretPosition);
            robot.turretMotor1.setPower(.375);

            // Send telemetry message to signify robot running;
            telemetry.addData("BallLoad",ballLoad.getVoltage());
            telemetry.addData("Turret Position", turretPosition);
            telemetry.addData("Turret Current Position", robot.turretMotor1.getCurrentPosition());
            telemetry.addData("Turret Current Power", robot.turretMotor1.getPower());
            telemetry.addData("Chute", chutePosition);
            telemetry.addData("Shooter Speed", power);
            telemetry.addData("PS Range", psRange1.cmUltrasonic());
            telemetry.addData("DS Range", dsRange1.cmUltrasonic());
            telemetry.addData("Color", lineColor.alpha());
            telemetry.addData("Color2", dsColor1.alpha());
            telemetry.addData("Color2-Red", dsColor1.red());
            telemetry.addData("Color2-Green", dsColor1.green());
            telemetry.addData("Color2-Blue", dsColor1.blue());
            telemetry.addData("ODS Light", dsODS1.getLightDetected());
            telemetry.addData("ODS-Raw Light", dsODS1.getRawLightDetected());
            telemetry.addData("Range", dsRange1.cmUltrasonic());
            telemetry.addData("ds1", dsRack1_Min);
            telemetry.addData("ds2", dsRack2_Min);
            telemetry.addData("ps1", psRack1_Min);
            telemetry.addData("ps2", psRack2_Min);
            telemetry.addData("arm",   "%.2f", armPosition);
            telemetry.addData("claw",  "%.2f", clawPosition);
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.addData("X", "%.2f", xTrack);
            telemetry.addData("Y", "%.2f", yTrack);
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }

    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
    public void moveRack(double rack, double positionCM) {
        double position = positionCM * 9.65/2; //9.65 counts per 1 cm movement of rack / 2 for ratio
        if (rack == 1) {
            double positionFinal = dsRack1_Min - position;
            if (positionFinal > dsRack1_Max) {
                robot.rackDS1.setPosition(1 - (positionFinal / 250));
            }
        } else if (rack == 2) {
            double positionFinal = dsRack2_Min + position;
            if (positionFinal < dsRack2_Max) {
                robot.rackDS2.setPosition(1 - (positionFinal / 250));
            }
        } else if (rack == 3) {
            double positionFinal = psRack1_Min + position;
            if (positionFinal < psRack1_Max) {
                robot.rackPS1.setPosition(1 - (positionFinal / 250));
            }
        } else if (rack == 4) {
            double positionFinal = psRack2_Min + position;
            if (positionFinal < psRack2_Max) {
                robot.rackPS2.setPosition(1 - (positionFinal / 250));
            }
        }

    }

}
