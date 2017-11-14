package org.firstinspires.ftc.teamcode.othersCode;

/**
 * Created by kskrueger on 11/13/17.
 */

public class JackPIDTurn {
    //gyroTurn uses a PID loop to rotate the robot to the target angle using IMU
    public void gyroTurn (double turnSpeed, int targetAngle, int error) {

        while (Math.abs(getHeading()-targetAngle)<error) {
            double pidOffset = PID(.0007,0,.12,20,targetAngle,(int)getHeading());

            double power = pidOffset*turnSpeed;
            //set power to motors
            /*robot.rightBackWheel.setPower(power);
            robot.rightFrontWheel.setPower(power);
            robot.leftBackWheel.setPower(-power);
            robot.leftFrontWheel.setPower(-power);*/
        }
        //stop motors
        /*robot.rightBackWheel.setPower(0);
        robot.rightFrontWheel.setPower(0);
        robot.leftBackWheel.setPower(0);
        robot.leftFrontWheel.setPower(0);*/
    }

    //PID loop Variables
    private int integral = 0;
    private int previousError = 0;

    double PID (double kP, double kI, double kD, int dt, int targetValue, int position) {
        int angleError = (targetValue - position);
        angleError -= (360*Math.floor(0.5+(((double)angleError)/360.0)));

        int error = angleError;

        integral += kI * error * dt;

        double u = (kP * error + integral + kD * (error - previousError) / dt);

        previousError = error;

        return u;
    }

    //Calculate the heading of the absolute orientation sensor (IMU) on the robot
    private double getHeading() {
        double angle = 0;//imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angle;
    }
}
