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
