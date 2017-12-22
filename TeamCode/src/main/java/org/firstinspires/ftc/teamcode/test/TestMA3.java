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
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.sensors.MA3Encoder;

@TeleOp(name="Test MA3 Encoder", group="Testing")
public class TestMA3 extends LinearOpMode {
    private MA3Encoder enc;
    private DcMotorImplEx motor;
    double deltaTime, lastTime, pos1, pos2, deltaPos, velocity; //variables used for getVelocity
    double incremental;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        motor = (DcMotorImplEx) hardwareMap.dcMotor.get("DS1");

        enc = new MA3Encoder(hardwareMap,"DSe1");
        enc.setMaxVoltage(5.12);
        enc.setZeroVoltage(0);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("Absolute Position",enc.getAbsolute());
            telemetry.addData("Incremental Position",enc.getIncremental());
            telemetry.addData("Velocity",enc.getVelocity());
            telemetry.addData("Voltage",enc.getVoltage());
            telemetry.addLine();
            telemetry.addData("DeltaTime",enc.deltaTime);
            telemetry.addData("Last Time",enc.lastTime);
            telemetry.addData("Pos1",enc.pos1);
            telemetry.addData("Pos2",enc.pos2);
            telemetry.addData("PosDelta",enc.deltaPos);
            telemetry.update();

            velocity = gamepad1.left_stick_y*1440;
            motor.setVelocity(velocity+enc.getVelocity(), AngleUnit.DEGREES);

            sleep(20);
        }
    }
}
