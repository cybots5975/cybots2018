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
package org.firstinspires.ftc.teamcode.matchCode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TEST SPMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM", group="TESSTTTTTTSTTSTSTSTTSTSTSTTSTTTTTTSTSTTTTSTTSTTTSTTTTTTTTTTTTT")
//@Disabled
public class TestSPM extends LinearOpMode {
    CRServo test;
    AnalogInput encoder;
    //PID loop Variables
    private int integral = 0;
    public int error;
    private int previousError = 0;

    private double zeroPosition;
    public int reverse;
    private boolean zeroReset;
    private boolean efficiency = true;

    private int holdPosition;

    @Override
    public void runOpMode() {
        test = hardwareMap.crservo.get("test");
        encoder = hardwareMap.analogInput.get("enc");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            test.setPower(swivelPID(angle(),0));

            telemetry.addData("Power",test.getPower());
            telemetry.update();
        }
    }
    double angleError;
    double angleErrorOp;
    int targetOp;

    //reverse180 calculates the error (difference) from the current angle to the the target angle...
    //...it also finds the opposite angle (180° offset) to see if it is colser for the module to rotate to
    private int reverse180(int targetAngle, int position) {
        targetOp = (targetAngle +180)%360;

        angleError = (targetAngle - position);
        angleError -= (360*Math.floor(0.5+((angleError)/360.0)));

        angleErrorOp = (targetOp - position);
        angleErrorOp -= (360*Math.floor(0.5+((angleErrorOp)/360.0)));

        int targetValue;
        if ((Math.abs(angleError)>Math.abs(angleErrorOp))) {
            targetValue =targetOp;
            reverse=-1;
        } else {
            targetValue = targetAngle;
            reverse=1;
        }

        if (zeroReset) {
            targetValue = 0;
        }

        angleError = (targetValue - position);
        angleError -= (360*Math.floor(0.5+((angleError+0d)/360.0)));

        error = (int)angleError;

        return error;
    }

    //PID (Proportional Integral Derivative) loop is used to take the error from target and...
    //...proportionally calculate what speed it needs `to rotate to reach the target value
    private double swivelPID (int angle, int targetAngle) {
        final double Kp = .015; //.03
        final double Ki = 0;
        final double Kd = .01;
        int dt = 20;

        error = reverse180(targetAngle,angle);

        integral += Ki * error * dt;

        double u = (Kp * error + integral + Kd * (error - previousError) / dt);

        previousError = error;

        double PIDpower = -1 * u;

        //convert to servo power range from 0-1
        double powerOut = PIDpower/2;
        telemetry.addData("PID Power",powerOut);

        powerOut = Range.clip(powerOut,-.88,.88);

        /*if (powerOut<.2&&powerOut>.05) {
            powerOut *= 2;
        } else if (powerOut>-.2&&powerOut<-.05) {
            powerOut *= 2;
        }*/

        return powerOut;
    }

    public int angle() {
        double maxVolt = 2.06;
        double angle = ((encoder.getVoltage()-zeroPosition)/ maxVolt)*360;
        if (angle<0) {
            angle = 360+angle;
        }

        return (int)angle;
    }
    
}
