package org.firstinspires.ftc.teamcode.util.multiGlyph.odemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.sensors.IMU;
import org.firstinspires.ftc.teamcode.util.PID;

import java.util.Arrays;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class PositionTracking {

    final private int xOffset = 1000;
    final private int yOffset = 1000;

    final private int xTolerance = 25;
    final private int yTolerance = 25;
    final private int turnTolerance = 3;

    private int prevX = 0;
    private int prevY = 0;
    private double prevHeading = 0;

    private int xPositionAbs = 0;
    private int yPositionAbs = 0;

    private boolean positionThread = false;
    private boolean driveThread = false;

    private PID xPID = new PID();
    private PID yPID = new PID();
    private PID turnPID = new PID();

    private LinearOpMode opMode;
    private IMU imu;
    private DcMotor xWheel, yWheel, frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor;

    public PositionTracking (LinearOpMode opMode, IMU imu,
                             DcMotor rightIntake, DcMotor leftIntake,
                             DcMotor frontLeftMotor, DcMotor rearLeftMotor,
                             DcMotor frontRightMotor, DcMotor rearRightMotor) {
        this.opMode = opMode;
        this.imu = imu;
        this.xWheel = rightIntake;
        this.yWheel = leftIntake;
        this.frontLeftMotor = frontLeftMotor;
        this.rearLeftMotor = rearLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.rearRightMotor = rearRightMotor;
    }

    public int xPosition() {
        return xPositionAbs;
    }

    public int yPosition() {
        return yPositionAbs;
    }

    private void fieldCentric(double forwards, double horizontal, double turning, int heading){
        //uses the orientation of the robot to offset the input powers.
        double forwrd = forwards * -1;
        double strafe = horizontal;

        double gyro_radians = heading * Math.PI/180;
        double temp = forwrd * cos(gyro_radians) + strafe * sin(gyro_radians);
        strafe = -forwrd * sin(gyro_radians) + strafe * cos(gyro_radians);
        forwrd = temp;

        robotCentric(-forwrd,strafe,turning);
    }

    private void robotCentric(double forwards, double horizontal, double turning) {
        double leftFront = forwards + horizontal + turning;
        double leftBack = forwards - horizontal + turning;
        double rightFront = forwards - horizontal - turning;
        double rightBack = forwards + horizontal - turning;

        double[] wheelPowers = {Math.abs(rightFront), Math.abs(leftFront), Math.abs(leftBack), Math.abs(rightBack)};
        Arrays.sort(wheelPowers);
        double biggestInput = wheelPowers[3];
        if (biggestInput > 1) {
            leftFront /= biggestInput;
            leftBack /= biggestInput;
            rightFront /= biggestInput;
            rightBack /= biggestInput;
        }

        frontLeftMotor.setPower(leftFront);
        frontRightMotor.setPower(rightFront);
        rearLeftMotor.setPower(leftBack);
        rearRightMotor.setPower(rightBack);
    }

    public void position (int xInput, int yInput, double heading) {
        double angleDelta = heading - prevHeading;
        prevHeading = heading;

        int xDelta = xInput - prevX;
        prevX = xInput;

        int yDelta = yInput - prevY;
        prevY = yInput;

        double strafe = xDelta - (xOffset*angleDelta/360);
        double forwrd = yDelta - (yOffset*angleDelta/360);

        double gyro_radians = heading * Math.PI/180;
        double temp = forwrd * cos(gyro_radians) + strafe * sin(gyro_radians);
        strafe = -forwrd * sin(gyro_radians) + strafe * cos(gyro_radians);
        forwrd = temp;

        yPositionAbs = (int)forwrd;
        xPositionAbs = (int)strafe;
    }

    public void stopTracking () {
        positionThread = false;
    }

    public void stopDrive () {
        driveThread = false;
    }

    public void startEncoderTracking() {
        positionThread = true;
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                while(!opMode.isStopRequested()&&opMode.opModeIsActive()&&positionThread) {
                    int xPosition = xWheel.getCurrentPosition();
                    int yPosition = yWheel.getCurrentPosition();
                    double heading = imu.getHeading();

                    position(xPosition,yPosition,heading);
                }
            }
        }).start();
    }

    public void driveToCoordinates(int xTarget, int yTarget, int headingTarget, int heading) {
        driveThread = true;
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                while(!opMode.isStopRequested()&&opMode.opModeIsActive()&&driveThread) {
                    double xPower = xPID.runDistance(xTarget,xPositionAbs);
                    double yPower = yPID.runDistance(yTarget,yPositionAbs);
                    double turnPower = turnPID.runDistance(headingTarget,heading);

                    fieldCentric(xPower,yPower,turnPower,heading);

                    //check if in tolerance and end thread if so
                    if (Math.abs(xTarget-xPositionAbs) < xTolerance
                        &&Math.abs(yTarget-yPositionAbs) < yTolerance
                        &&Math.abs(headingTarget-heading) < turnTolerance) {

                        driveThread = false;

                    }
                }
                robotCentric(0,0,0);
            }
        }).start();
    }

}
