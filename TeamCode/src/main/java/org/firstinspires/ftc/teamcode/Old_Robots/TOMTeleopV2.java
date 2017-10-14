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

/*  Created by Karter Krueger for FTC Cybots 5975 on 3/13/17
    Cleaned up, organized, and updated comments to previous program
    Started new naming method, starting at V1
*/

package org.firstinspires.ftc.teamcode.Old_Robots;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TOM: TeleopV2", group="TOM Teleop")
//@Disabled
public class TOMTeleopV2 extends LinearOpMode {

//Declare OpMode members.
    HardwareTOM robot = new HardwareTOM(); // Use TOM hardware file

//assorted variables
    double  ballLiftCycle = 0; //keeps track of position of ball feed
    double  ballReverseCycle = 0; //start the reverse cycle count at 0
    double  reverseCycleCount = 8; //number of times to loop for ball lift reverse function
    double  allianceColor = 0;
    int blueBallValue = 0; //blue value must be greater than 0 to reject the ball during driving
    int redBallValue = 1; //red value must be greater than 1 to reject ball during driving
    double  ballRejectTime = 2;
    double currentTime = 0;
    double detectTime = 0;
    double  shooterBoost = 0;

//speed variables
    double  pos1 = 0;
    double  pos2 = 0;
    double  speed = 0;
    double  count = 0;
    double  power = 1;

//shifter variables
    double          rearInitial = .5; //this is the servo position for the rear shifter at the start of the match (sweep)

    double          rearShifted = 0; //this is the servo position for the rear shifter when shifted (lift)

    double          middleInitial = .5; //this is the servo position for the middle shifter at the start of the match (4 motor drive)
    double          middleShifted = 0; //this is the servo position for the middle shifter when shifted (lift)

//pixy track variables
    double          pixyCenter = 1.7; //this is the starting center value of our pixy (we can adjust this during the match if needed
    double          deadband = .02; //this variable adjusts how much the pixy can float back and forth for accuracy
    double          pixyMin = 0.22; //this is the minimum value that the pixy can return for range
    double          pixyMax = 3.8;  //this it the maximum value that the pixy can return for range
    int             xPosition; //this is the current commanded position of the turret to match pixy

//turret varibles
    int turretOffset = 1100; //starting offset of the turret position from center (autonomous ends at 1100
    int turretLow = -3000-turretOffset; //this limits the turrets rotation in the counter-clockwise direction
    //-600
    int turretHigh = 2100-turretOffset; //this limits the turrets rotation in the clockwise direction
    //2200
    int turretPosition = 0-turretOffset; //this is the original value of the turret offset
    int turretInitPosition = 0; //has turret position been initialized

//initial servo values
    double pixyDown = .95; //pixy servo value for the down position to fit in 18 at start of match
    double pixyUp   = .5; //pixy servo value for the up position to be pointed at the goal
    double sensorAngle = 34.2; //was 25 previously; actually is 28.6
    double capBallPosition = .5; //position of cap ball servo

//shooter distance variables
    double velocity; //velocity of the shooting wheels
    double goalHypDistance; //hypotenuse distance to center goal (used for calculating trajectory)
    double angle; //angle of chute
    double height = .85; //height of goal relative to the robot top in meters
    double distanceToGoal; //distance to goal (calculated using hypotenuse measured by sensor)
    double totalDistance; //calulates target position for ball to go through the goal (calculated from distanceToGoal)
    double servoCounts = .0066; //servo counts per 1 degree of chute angle movement
    double chutePosition = .299; //starting position of the chute
    double position90 = .310; //was previously .302 (servo value of chute base at 90 degrees to robot)
    double shooterConstant = .575;
    //double chuteMin = .3; //minimum position of the chute
    //double chuteMax = .85; //maximum position of the chute

//servo rack starting positions
    public double dsRack1_Min = 129;
    public double dsRack1_Max = 22; //y goes out
    public double dsRack2_Min = 174;
    public double dsRack2_Max = 71; //y goes out
    public double psRack1_Min = 109;
    public double psRack1_Max = 222; //a goes out
    public double psRack2_Min = 157;
    public double psRack2_Max = 233; //a goes out

//define sensors
    ModernRoboticsI2cGyro gyro = null; //gyro sensor used for turning
    ModernRoboticsI2cColorSensor lineColor = null; //bottom facing color sensor to detect lines
    ModernRoboticsI2cColorSensor lineColor2 = null; //used to detect ball color TODO: 3/12/17 update name to match function
    ModernRoboticsI2cColorSensor dsColor1 = null; //color sensor 1 for detecting beacon TODO: change name
    ModernRoboticsI2cColorSensor psColor1 = null; //color sensor 2 for detecting beacon TODO: change name
    ModernRoboticsI2cRangeSensor dsRange1 = null;
    ModernRoboticsI2cRangeSensor psRange1 = null;
    ModernRoboticsAnalogOpticalDistanceSensor dsODS1 = null; //mounted on bottom to detect line // TODO: 3/12/17 update name

//time and loop variables
    double loopCount; //counts number of loops of program for lift switch
    private ElapsedTime runtime = new ElapsedTime(); //time

    @Override //init code
    public void runOpMode() {
    //joystick varibles
        double left; //used for joystick to control driver side wheels
        double right; //used for joystick to control passenger side wheels
        double turn; //used for joystick to control robot turn
        double lift; //used for joystick to control lift

    //Initialize the hardware variables.
    //The init() method of the hardware class does all the work here
        robot.init(hardwareMap);

    //Define Sensors
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        lineColor = (ModernRoboticsI2cColorSensor)hardwareMap.colorSensor.get("lineColor");
        lineColor.setI2cAddress(I2cAddr.create8bit(0x3c));
        lineColor.enableLed(true); // Turn the LED on

        dsColor1 = (ModernRoboticsI2cColorSensor)hardwareMap.colorSensor.get("dsColor1");
        dsColor1.setI2cAddress(I2cAddr.create8bit(0x10));
        dsColor1.enableLed(false);

        psRange1 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "psRange1");
        psRange1.setI2cAddress(I2cAddr.create8bit(0x28));

        dsRange1 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "dsRange1");
        dsRange1.setI2cAddress(I2cAddr.create8bit(0x30));

        dsODS1 = (ModernRoboticsAnalogOpticalDistanceSensor)hardwareMap.opticalDistanceSensor.get("dsODS1");

        AnalogInput pixySensor;
        pixySensor = hardwareMap.analogInput.get("pixy");

        AnalogInput goalRange;
        goalRange = hardwareMap.analogInput.get("goalRange");

        AnalogInput ballLoad;
        ballLoad = hardwareMap.analogInput.get("ballLoad");

    //Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        moveRack(1,0);
        moveRack(2,0);
        moveRack(3,0);
        moveRack(4,0);

        robot.capBall.setPosition(.5);
        robot.dSweep.setPosition(.5);
        robot.pSweep.setPosition(.5);

        //set motors to correct modes
        robot.turretMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.passMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.driveMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.passMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.sweepMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.turretMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shootMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shootMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("-","Motor modes initialized");
        telemetry.update();

    //Initialize options
        int turretZeroed = 0;
        while (!isStopRequested() && turretZeroed==0) {
            if (gamepad1.left_stick_x > .5 || gamepad2.left_stick_x > .5) {
                turretInitPosition = turretInitPosition + 20;
                robot.turretMotor1.setTargetPosition(turretInitPosition);
                robot.turretMotor1.setPower(1);
                telemetry.addData("Turret Position",turretInitPosition);
                telemetry.addData(">","Turning turret right");
                telemetry.update();
            }
            if (gamepad1.left_stick_x <-.5 || gamepad2.left_stick_x < -.5) {
                turretInitPosition = turretInitPosition - 20;
                robot.turretMotor1.setTargetPosition(turretInitPosition);
                robot.turretMotor1.setPower(1);
                telemetry.addData("Turret Position",turretInitPosition);
                telemetry.addData("<","Turning turret left");
                telemetry.update();
            }
            robot.turretMotor1.setTargetPosition(turretInitPosition);
            robot.turretMotor1.setPower(1);
            telemetry.addData("Turret Position",turretInitPosition);

            if (gamepad1.left_stick_button || gamepad2.left_stick_button) {

                telemetry.addData("-","Turret initialized");
                telemetry.update();
                turretZeroed = 1;
            }
            telemetry.addData("-","Looping turret align");
            telemetry.update();
        }

        /*int turretCentered = 0;
        while (!isStopRequested() && turretCentered==0) {
            telemetry.addData(">", "Waiting for Turret Position");
            telemetry.addData("-", "Press A for Auto");
            telemetry.addData("-", "Press B for Center");
            telemetry.update();
            if (gamepad1.a || gamepad2.a) {
                turretCentered = 1;
                turretOffset = 1100;
            } else if (gamepad1.b || gamepad2.b) {
                turretCentered = 1;
                turretOffset = 0;
            }
        }*/

        robot.turretMotor1.setTargetPosition(0-turretOffset+turretInitPosition);
        robot.turretMotor1.setPower(.3);

        /*int initConfirm = 0;
        while (!isStopRequested() && initConfirm==0) {
            if (gamepad1.a||gamepad2.a) {
                initConfirm = 1;
            }
        }*/

        //if driver pressed a during the earlier loop, then initialize the robot during init period, otherwise, wait until teleop
        //initConfirm==1
        while (!isStopRequested() && allianceColor==0) {
            if (gamepad1.x||gamepad2.x) {
                allianceColor = 2; //set alliance color to blue value

            } else if (gamepad1.b||gamepad2.b) {
                allianceColor = 1; //set alliance color to red value

            } else if (gamepad1.y||gamepad2.y) {
                allianceColor = 3; //set alliance color to none value

            }
        }
        telemetry.addData("Alliance Side", allianceColor);

        if (!isStopRequested()) {
            telemetry.addData(">","Initializing");
            telemetry.update();
            moveRack(1,0);
            moveRack(2,0);
            moveRack(3,0);
            moveRack(4,0);

            robot.ballLift.setPosition(0); //set ball lift servo to off
            robot.chute.setPosition(.84); //set chute to down position to fit in 18

            robot.pixyServo.setPosition(pixyDown); //put pixy servo down to fit in 18 limit

            robot.rearShifter.setPosition(rearInitial); //shift middle servo to initial (sweep) position
            robot.middleShifter.setPosition(middleInitial); //shift rear servo to initial (drive) position


        }

        int initializedLoop = 0; //used at start of teleop to only initialize once



        waitForStart(); //Wait for the game to start (driver presses PLAY)

        while (opModeIsActive()) {
            //run until the end of the match (driver presses STOP)

            //initialize once, at start of first teleop loop
            if (opModeIsActive()&&initializedLoop==0) {
                runtime.reset();
                moveRack(1,0);
                moveRack(2,0);
                moveRack(3,0);
                moveRack(4,0);

                robot.ballLift.setPosition(0); //stop ball feed
                //robot.chute.setPosition(.84); //set chute to down position

                robot.pixyServo.setPosition(pixyDown); //rotate pixy servo down

                robot.rearShifter.setPosition(rearInitial); //shift middle servo to initial (sweep) position
                robot.middleShifter.setPosition(middleInitial); //shift rear servo to initial (drive) position


                robot.pixyServo.setPosition(pixyUp);


                initializedLoop = 1; //set loop to 1 after completeling initialization process once, so it doesn't repeat
            }

        //set variables from joystick values
            left = (ScaleInputDrive(gamepad1.left_stick_y)); //scale the left joystick value to the left variable for drive power
            right = (ScaleInputDrive(gamepad1.right_stick_y)); //scale the right joystick value to the right variable for drive power
            turn = (gamepad1.left_stick_x+gamepad1.right_stick_x)/2; //(experimental) turn robot if press joysticks sideways (average)

            //if gamepad 2 hold a button combination for 10 loops (down later), then gamepad 2 controls the shifted motors for the lift
            if (loopCount >= 2 ) {
                lift = gamepad2.right_stick_y;
                lift = Range.clip(lift, -1, 1);   //clip right driver power input to maximum of 1 and minimum of -1

                robot.sweepMotor1.setPower(-lift);
                robot.driveMotor2.setPower(-lift);
                robot.passMotor2.setPower(-lift);
                robot.passMotor1.setPower(left);
                robot.driveMotor1.setPower(right);

                if (gamepad2.dpad_up) {
                    robot.capBall.setPosition(0); //pull cap ball in
                } else if (gamepad2.dpad_down) {
                    robot.capBall.setPosition(1); //push cap ball out
                } else {
                    robot.capBall.setPosition(.5); //push cap ball out
                }

            } else {
                left = Range.clip(left, -1, 1); //clip left driver power input to maximum of 1 and minimum of -1
                right = Range.clip(right, -1, 1); //clip right driver power input to maximum of 1 and minimum of -1
                turn = Range.clip(turn, -1, 1);

                if (gamepad1.right_stick_button) {
                    left = .3;
                    right = .4;
                    moveRack(2,3.5);
                }
                if (gamepad1.left_stick_button) {
                    left = -.3;
                    right = -.4;
                    moveRack(1,4.4);
                }
                currentTime = runtime.seconds();

                robot.dSweep.setPosition(1);
                robot.pSweep.setPosition(0);

                if (allianceColor==1) { //red alliance
                    if (robot.lineColor.blue()>blueBallValue) {
                        detectTime = currentTime;
                    }
                } else if (allianceColor==2){
                    if (robot.lineColor.red() > redBallValue) {
                        detectTime = currentTime;
                    }
                }

                if (gamepad2.left_trigger > .25) { //run this if the trigger is pulled
                    if (currentTime<detectTime+ballRejectTime&&currentTime>2) {
                        robot.sweepMotor1.setPower(-1);
                    } else {
                        robot.sweepMotor1.setPower(gamepad2.left_trigger);
                    }
                } else if (gamepad2.dpad_left && !gamepad2.left_stick_button && !gamepad2.right_stick_button) {
                    robot.sweepMotor1.setPower(-1);
                } else {
                    robot.sweepMotor1.setPower(0);
                }

                if ((left>.85 || left<-.85) && (right>.85 || right <-.85)) {
                    robot.driveMotor1.setPower(-left);
                    robot.driveMotor2.setPower(-left);
                    robot.passMotor1.setPower(-right);
                    robot.passMotor2.setPower(-right);
                } else if (gamepad1.left_trigger>.25) {
                    robot.driveMotor1.setPower(-left);
                    robot.driveMotor2.setPower(-left);
                    robot.passMotor1.setPower(-right);
                    robot.passMotor2.setPower(-right);
                } else {
                    robot.driveMotor1.setPower(-left*3/4);
                    robot.driveMotor2.setPower(-left*3/4);
                    robot.passMotor1.setPower(-right*3/4);
                    robot.passMotor2.setPower(-right*3/4);
                }
            }

            if (gamepad1.dpad_left) {

            }
            if (gamepad1.dpad_right) {

            }

            if (gamepad1.right_trigger>.25) {
                moveRack(1, 4.4);
                moveRack(2, 3.5);
            } else {
                if (gamepad1.left_bumper) {
                    moveRack(1, 17);
                    moveRack(3, 17);
                } else {
                    moveRack(1, 0);
                    moveRack(3, 0);
                }
                if (gamepad1.right_bumper) {
                    moveRack(2, 17);
                    moveRack(4, 17);
                } else {
                    moveRack(2, 0);
                    moveRack(4, 0);
                }
            }

            if (gamepad2.dpad_right && !gamepad2.left_stick_button && !gamepad2.right_stick_button) {
                robot.ballLift.setPosition(.5);
            } else if (gamepad2.left_bumper) {
                robot.ballLift.setPosition(1);
                shooterBoost = .01;
            } else {
                robot.ballLift.setPosition(0);
                shooterBoost = -.01;
            }

            //shooting trajectory code
            if (gamepad2.right_trigger > .25) {
                goalHypDistance = goalRange.getVoltage() / 0.009766; //distance of hypotenuse to goal in inches
                distanceToGoal = goalHypDistance * (Math.cos(sensorAngle/57.298)); //find the distance to the goal using cosine of hypotenuse and sensor angle
                totalDistance = distanceToGoal * .0254 / 0.71 + .5; //convert distance to goal to meters, then convert to distance THROUGH goal, then add offset of .5meters
                angle = (Math.atan(((4 * height) / totalDistance)) * 57.298); //calculate angle of ball to go through goal
                velocity = ((Math.sqrt(2 * 9.80665 * height)) / (Math.sin(angle / 57.298))); //calculate velocity of ball for trajectory
                chutePosition = position90 - (servoCounts * (62.5 - angle)); //convert angle of the chute to get correct trajectory

                double shooterSpeed = shooterConstant / 4.6 * velocity; //convert velocity to shooter speed level for correct trajectory
                shooterSpeed = Range.clip(shooterSpeed, 0, 1);

                if (shooterSpeed <= 1 && shooterSpeed >= -1) {
                    robot.shootMotor1.setPower(shooterSpeed-.01+shooterBoost);
                    robot.shootMotor2.setPower(shooterSpeed-.01+shooterBoost);
                }

            } else if (gamepad2.right_bumper) {
                robot.shootMotor1.setPower(.58); //here in case our sensor fails, we could manually override
                robot.shootMotor1.setPower(.58);
            } else if (loopCount<2) {
                robot.shootMotor1.setPower(.45); //set shooters to 45% if bumper or trigger isn't pressed, so spinup takes less time
                robot.shootMotor2.setPower(.45);
                robot.turretMotor1.setPower(1);
            } else {
                robot.shootMotor1.setPower(0);
                robot.shootMotor2.setPower(0);
                robot.turretMotor1.setPower(0);
            }
            robot.chute.setPosition(chutePosition);

            //pixy tracking code
            double voltreading = (float) pixySensor.getVoltage();
            if (voltreading > pixyMax || voltreading < pixyMin) {
                telemetry.addData("Out of Range", "");
                telemetry.update();
            } else if (voltreading > pixyCenter && xPosition > turretLow && xPosition < turretHigh) {
                if (voltreading > (pixyCenter + 1.7)) {
                    xPosition = xPosition + 200;
                } else if (voltreading > (pixyCenter + 1.28)) {
                    xPosition = xPosition + 75;
                } else if (voltreading > (pixyCenter + .885)) {
                    xPosition = xPosition + 40;
                } else if (voltreading > (pixyCenter + .49)) {
                    xPosition = xPosition + 15;
                } else if (voltreading > (pixyCenter + .1)) {
                    xPosition = xPosition + 7;
                } else if (voltreading > (pixyCenter + .05)) {
                    xPosition = xPosition + 1;
                } else if (voltreading > (pixyCenter + deadband)) {
                    telemetry.addData("Pixy Centered",":)");
                }
            } else if (voltreading < pixyCenter) {
                if (voltreading < (pixyCenter - 1.7)) {
                    xPosition = xPosition - 200;
                } else if (voltreading < (pixyCenter - 1.28)) {
                    xPosition = xPosition - 75;
                } else if (voltreading < (pixyCenter - .885)) {
                    xPosition = xPosition - 40;
                } else if (voltreading < (pixyCenter - .49)) {
                    xPosition = xPosition - 15;
                } else if (voltreading < (pixyCenter - .1)) {
                    xPosition = xPosition - 7;
                } else if (voltreading < (pixyCenter - .05)) {
                    xPosition = xPosition - 1;
                } else if (voltreading < (pixyCenter - deadband)) {
                    telemetry.addData(":) ","Pixy Centered");
                }
            }

            //prevent turret from going past minimum and maximum


            /*if (gamepad2.x) {
                turretPosition = -1100; //set turret to left position while x is pressed
            } else if (gamepad2.y) {
                turretPosition = 0; //set turret to front position while y is pressed
            } else if (gamepad2.b) {
                turretPosition = 1100; //set turret to right position while b is pressed
            } else if (gamepad2.a) {
                turretPosition = 2200; //set turret to rear position while a is pressed
            } else {
                turretPosition = xPosition; //use pixy camera tracking while no buttons are pressed
            }*/

            if (gamepad2.x||gamepad1.x) {
                xPosition = -1000-turretOffset; //set turret to left position while x is pressed
            } else if (gamepad2.y||gamepad1.y) {
                xPosition = 0-turretOffset; //set turret to front position while y is pressed
            } else if (gamepad2.b||gamepad1.b) {
                xPosition = 1000-turretOffset; //set turret to right position while b is pressed
            } else if (gamepad2.a||gamepad1.a) {
                xPosition = 2000-turretOffset; //set turret to rear position while a is pressed
            }
            if (xPosition<turretLow) {
                xPosition = turretLow+25;
            }
            if (xPosition>turretHigh) {
                xPosition = turretHigh-25;
            }

            turretPosition = xPosition;

            robot.turretMotor1.setTargetPosition(turretPosition); //set turret to position of variable
            robot.turretMotor1.setPower(1); //set turret to full speed to get to targeted position



            if (gamepad2.right_stick_button&&gamepad2.dpad_left) {
                pixyCenter=pixyCenter+.01;
            }
            if (gamepad2.right_stick_button&&gamepad2.dpad_right) {
                pixyCenter=pixyCenter-.01;
            }

            if (gamepad2.right_stick_button&&gamepad2.dpad_up) {
                position90=position90+.01;
            }
            if (gamepad2.right_stick_button&&gamepad2.dpad_down) {
                position90=position90-.01;
            }

            if (gamepad2.left_stick_button&&gamepad2.dpad_up) {
                shooterConstant=shooterConstant+.001;
            }
            if (gamepad2.left_stick_button&&gamepad2.dpad_down) {
                shooterConstant=shooterConstant-.001;
            }

            if ((gamepad2.dpad_left && gamepad2.left_stick_button && loopCount<=4)) {
                loopCount = loopCount +1;
            }

            if ((gamepad2.dpad_right && gamepad2.left_stick_button && loopCount>=-4)) {
                loopCount = loopCount -1;
            }

            if (loopCount>=2) {
                robot.rearShifter.setPosition(rearShifted);
                robot.middleShifter.setPosition(middleShifted);
            }else if (loopCount<=-2) {
                robot.rearShifter.setPosition(rearInitial);
                robot.middleShifter.setPosition(middleInitial);
            } else {
                robot.rearShifter.setPosition(rearInitial);
                robot.middleShifter.setPosition(middleInitial);
            }

            if (gamepad1.dpad_up&&gamepad1.right_stick_button) {
                sensorAngle = sensorAngle+.05;
            } else if (gamepad1.dpad_down&&gamepad1.right_stick_button) {
                sensorAngle = sensorAngle-.05;
            }
            //robot.capBall.setPosition(capBallPosition);

            // Send telemetry message to signify robot running;
            telemetry.addData("PixyCenter", pixyCenter);
            telemetry.addData("Servo 90 Position", position90);
            telemetry.addData("Shooter Constant", shooterConstant);
            telemetry.addData("Reject Blue Value",robot.lineColor.blue());
            telemetry.addData("Reject Red Value",robot.lineColor.red());
            telemetry.addData("Current Time",currentTime);
            telemetry.addData("Detect Time",detectTime);
            telemetry.addData("Alliance Color", allianceColor);
            telemetry.addData("Sensor Angle",sensorAngle);
            telemetry.addData("Blue",robot.lineColor.blue());
            //telemetry.addData("Range", goalRange.getVoltage());
            //telemetry.addData("DistanceToGoal", distanceToGoal);
            //telemetry.addData("Total Distance", totalDistance);
            //telemetry.addData("Goal Hyp Distance", goalHypDistance);
            telemetry.update();

            robot.waitForTick(40); //Pause for metronome tick.  40 mS each cycle = update 25 times a second.
        }
    }

    public void moveRack(double rack, double positionCM) {
        double position = positionCM * 9.65/2; //9.65 counts per 1 cm movement of rack / 2 for ratio
        if (rack == 1) {
            double positionFinal = dsRack1_Min - position;
            if (positionFinal > dsRack1_Max) {
                robot.rackDS1.setPosition(1 - (positionFinal / 250));
            }
        } else if (rack == 2) {
            double positionFinal = dsRack2_Min - position;
            if (positionFinal > dsRack2_Max) {
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
            } //contains 4 cases, 1 for each servo; the same program can be used for any of the 4 servo racks
        }
    }

    //This sub program "scales" the input coming in.
    //Takes joystick input, and sets power to closest sored value of the array.
    //This results in precise driving when moving in the lower half of the 0-1 power scale.
    // While in the higher half, this allows for straighter driving.
    float ScaleInputDrive(float ScaleInputDrive) {
        double[] DriveArray = {0,.1,.15,.2,.25,.3,.4,.45,.5,.6,.7,.75,.8, .85,.9,1};
        //double[] DriveArray = {0,.1,.15,.2,.25,.3,.4,.45,.5,.6,.7,.75,.8, .85,.9,1};
        //ScaleInputDrive is multiplied by (DriveArray total variables) 15 because arrays start at 0, so its numbers 0-15, instead of 1-16
        int DriveIndex = (int) (ScaleInputDrive * 15);
        // This allows for "negative" numbers in the array, without having to directly enter them
        if (DriveIndex < 0) {
            DriveIndex = -DriveIndex;
        }
        double DriveScale = 0;
        if (ScaleInputDrive < 0) {
            DriveScale = -DriveArray[DriveIndex];
        }else {
            DriveScale = DriveArray[DriveIndex];
        }
        // Returns the value DriveScale, which is used with the Joysticks when they are set to
        // the variables DriveLeft and DriveRight
        return (float)DriveScale;
    }



}
