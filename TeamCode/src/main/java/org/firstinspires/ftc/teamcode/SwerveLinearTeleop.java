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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.HardwareSwerveV1;

@TeleOp(name="SwerveLinearTeleop Template", group="Swerve")
@Disabled
public class SwerveLinearTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareSwerveV1 robot           = new HardwareSwerveV1();   // Use the SwerveV1 hardware file

    //Module Zero Positions
    public final static double D1Zero = 0.0; //Sensor Voltage for Driver Front(1) Zero
    public final static double D2Zero = 0.0; //Sensor Voltage for Driver Back(2) Zero
    public final static double P1Zero = 0.0; //Sensor Voltage for Passenger Front(1) Zero
    public final static double P2Zero = 0.0; //Sensor Voltage for Passenger Back(2) Zero

    public DcMotor  DMotor1 = null; //Driver Motor Front (1)
    public DcMotor  DMotor2 = null; //Driver Motor Back (2)
    public DcMotor  PMotor1 = null; //Passenger Motor Front (1)
    public DcMotor  PMotor2 = null; //Passenger Motor Back (2)
    //Swerve Drivebase Servos
    public Servo    DServo1 = null; //Driver ServoFront (1)
    public Servo    DServo2 = null; //Driver ServoFront (2)
    public Servo    PServo1 = null; //Passenger ServoFront (1)
    public Servo    PServo2 = null; //Passenger ServoFront (2)

    //Swerve Drivebase Encoders
    public AnalogInput DSensor1 = null; //Driver Sensor Front (1)
    public AnalogInput  DSensor2 = null; //Driver Sensor Back (2)
    public AnalogInput  PSensor1 = null; //Passenger Sensor Front (1)
    public AnalogInput  PSensor2 = null; //Passenger Sensor Back (2)

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }

    public void SwerveDrive (double x1, double y1, double x2) {
        final double L = 14; //length between axles
        final double W = 16; //width between axles

        double r = Math.sqrt ((L * L) + (W * W));
        y1 *= -1;

        double a = x1 - x2 * (L / r);
        double b = x1 + x2 * (L / r);
        double c = y1 - x2 * (W / r);
        double d = y1 + x2 * (W / r);

        double backRightSpeed = Math.sqrt ((a * a) + (d * d));
        double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
        double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
        double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));

        double backRightAngle = Math.atan2 (a, d) / 3.1415926536 *180 +180;
        double backLeftAngle = Math.atan2 (a, c) / 3.1415926536 *180 +180;
        double frontRightAngle = Math.atan2 (b, d) / 3.1415926536 *180 +180;
        double frontLeftAngle = Math.atan2 (b, c) / 3.1415926536 *180 +180;

        DMotor1.setPower(frontLeftSpeed);   //Set speed of Driver Motor Front(1) to front left
        DMotor2.setPower(backLeftSpeed);    //Set speed of Driver Motor Back(2) to back left
        PMotor1.setPower(frontLeftSpeed);   //Set speed of Pass Motor Front(1) to front right
        PMotor2.setPower(backLeftSpeed);    //Set speed of Pass Motor Back(2) to back right

        Double DS1 = DSensor1.getVoltage(); //Get voltage of Driver Front(1) encoder
        Double DS2 = DSensor2.getVoltage(); //Get voltage of Driver Front(1) encoder
        Double PS1 = PSensor1.getVoltage(); //Get voltage of Driver Front(1) encoder
        Double PS2 = PSensor1.getVoltage(); //Get voltage of Driver Front(1) encoder

        DServo1.setPosition(SwivelMath(DS1,frontLeftAngle,0,5)); //Rotate the module to position
        DServo2.setPosition(SwivelMath(DS2,backLeftAngle,0,5)); //Rotate the module to position
        PServo1.setPosition(SwivelMath(PS1,frontRightAngle,0,5)); //Rotate the module to position
        PServo2.setPosition(SwivelMath(PS2,backRightAngle,0,5)); //Rotate the module to position

    }

    public double SwivelMath (double voltage, double targetAngle, double startVolt, double maxVolt) {
        double voltDegree = maxVolt/360;
        double targetVolt = startVolt + (targetAngle * voltDegree);
        double servoPower = .5;

        if (voltage>targetVolt+.5) {
            servoPower = 1;
        }else if (voltage<targetVolt+.25) {
            servoPower = .8;
        }else if (voltage<targetVolt+.15) {
            servoPower = .7;
        }else if (voltage<targetVolt+.1) {
            servoPower = .6;
        }else if (voltage<targetVolt+.02) {
            servoPower = .5;
        }else if (voltage>targetVolt-.02) {
            servoPower = .5;
        }else if (voltage>targetVolt-.1) {
            servoPower = .4;
        }else if (voltage>targetVolt-.15) {
            servoPower = .3;
        }else if (voltage>targetVolt-.25) {
            servoPower = .2;
        }else if (voltage>targetVolt-.5) {
            servoPower = 0;
        }
        return servoPower;
    }


}
