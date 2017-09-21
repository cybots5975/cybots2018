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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="PID Test 2", group="Drive")
//@Disabled
public class TestServoPID extends LinearOpMode {
    //DcMotor leftSide;
    //DcMotor rightSide;
    Servo    DServo1 = null; //Driver ServoFront (1)
    AnalogInput DSensor1 = null; //Driver Sensor Front (1)
    DcMotor   DMotor1;

    public static final double Kp = .02;

    public static final double Ki = 0;

    public static final double Kd = .008;

    public int integral;

    public int dt = 20;

    public double u;

    public int error;

    public int previousError;

    public int setPoint;

    public double rightPower;

    public int targetValue = 0; //180 was int

    public double maxVolt = 5;

    public double angle;

    public int opTarget;

    public boolean turnEfficiency = true;

    public int driveDirection;

    public double zeroPosVolt = 0;

    public int angleError;

    public int angleErrorOp;

    public int measuredAngle;

    boolean closeTo5;
    boolean closeTo0;

    @Override
    public void runOpMode() throws InterruptedException {
        //leftSide = hardwareMap.dcMotor.get("lw");
        //rightSide = hardwareMap.dcMotor.get("rw");
        //rightSide.setDirection(DcMotor.Direction.REVERSE);

        //leftSide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightSide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DSensor1 = hardwareMap.analogInput.get("PSe2");
        DServo1 = hardwareMap.servo.get("PS2");
        DMotor1 = hardwareMap.dcMotor.get("PM2");

        DServo1.setPosition(.5);

        waitForStart();

        //rightSide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(opModeIsActive()) {

            gamepad1.setJoystickDeadzone(.1F);

            double leftY = -gamepad1.left_stick_y;
            double leftX = gamepad1.left_stick_x;

            double speed = Math.sqrt((leftX*leftX)+(leftY*leftY));
            Range.clip(speed,0,1);

            //targetValue = (int) (Math.toDegrees(Math.atan2(leftX,leftY)));



           if (leftX>=0) {
                //no change
            } else if (leftX<0) {
                targetValue = 360+targetValue;
            }

            telemetry.addData("Target Angle",targetValue);


            targetValue = 0; //setting the angle of the module here


//START PID MODULE CODE



            angle = ((DSensor1.getVoltage()-zeroPosVolt)/maxVolt)*360; //getting the angle from the encoder by dividing voltage by 360
            //calculate the angle with 180 degrees difference to see if it's faster than turning all the way around
            measuredAngle = (int)angle;

            if ((targetValue-180)<0) {
                opTarget = 360+(targetValue-180);
            } else {
                opTarget = targetValue-180;
            }
            //angle_error = target_angle - measured_angle

            /*if (turnEfficiency) {
                if ((Math.abs(targetValue-angle))<=(Math.abs(opTarget-angle))) {
                    driveDirection = 1;
                } else {
                    targetValue = opTarget;
                    driveDirection = -1;
                }
            }*/

            angleError = (targetValue - measuredAngle);
            angleError -= (360*Math.floor(0.5+((angleError+0d)/360.0)));

            angleErrorOp = (opTarget - measuredAngle);
            angleErrorOp -= (360*Math.floor(0.5+((angleErrorOp+0d)/360.0)));

            if (turnEfficiency) {
                if (Math.abs(angleError)>Math.abs(angleErrorOp)) {
                    targetValue=opTarget;
                    angleError = (targetValue - measuredAngle);
                    angleError -= (360*Math.floor(0.5+((angleError+0d)/360.0)));
                } else {
                }
            }

            /*if (turnEfficiency==true) {
                if ((Math.abs(angleError))<=(Math.abs(angleErrorOp))) {
                    error = angleError;
                    driveDirection = 1;
                } else {
                    error = angleErrorOp;
                    driveDirection = -1;
                }
            } else {
                error = angleError;
            }*/

            //setPoint = angleError;

            //error = angleError;//targetValue - setPoint;

            error = angleError;

            integral += Ki * error * dt;

/*
            if(integral > targetValue * 0.25) {
                integral = (int) (targetValue * 0.25);
            }*/

            u = (Kp * error + integral + Kd * (error - previousError) / dt);

            previousError = error;

            rightPower = -1*u;

            if (rightPower>0) {
                DServo1.setPosition(.5+(rightPower/2));
            } else if (rightPower<0) {
                DServo1.setPosition(.5+(rightPower/2));
            } else {
                DServo1.setPosition(rightPower);
            }

    DMotor1.setPower(speed);

            telemetry.addData("Gamepad X", leftX);
            telemetry.addData("Gamepad Y", leftY);
            telemetry.addData("Speed",speed);
            telemetry.addData("Output", u);
            telemetry.addData("Volt",DSensor1.getVoltage());
            telemetry.addData("Power", DServo1.getPosition());
            telemetry.addData("DriveDirection", driveDirection);
            telemetry.addData("previousError", previousError);
            telemetry.addData("Error", error);
            telemetry.addData("targetValue",targetValue);
            telemetry.addData("measured angle", measuredAngle);
            telemetry.update();
        }
    }

    public double floor(double n) {
        return (n - (n % 1));
    }
}

/*
angle = (DSensor1.getVoltage()/maxVolt)*360;

        if ((angle-180)<0) {
        opAngle = 360+(angle-180);
        } else {
        opAngle = angle-180;
        }

        if (turnEfficiency=true) {
        if ((Math.abs(targetValue-angle))<(Math.abs(targetValue-opAngle))) {
        setPoint = (int) angle;
        driveDirection = 1;
        } else {
        setPoint = (int) opAngle;
        driveDirection = -1;
        }
        } else {
        setPoint = (int) angle;
        }*/
