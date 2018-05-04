package org.firstinspires.ftc.teamcode.util.multiGlyph.odemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.drivebase.VectorDrive;
import org.firstinspires.ftc.teamcode.subsystems.sensors.IMU;
import org.firstinspires.ftc.teamcode.util.PID;
import org.firstinspires.ftc.teamcode.util.logging.ArrayLogging;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class PositionTracking {

    final private int xOffset = 0;
    final private int yOffset = 0;

    public final int xPerInch = -233;
    public final int yPerInch = -233;

    final private int xTolerance = 100;
    final private int yTolerance = 100;
    final private int turnTolerance = 3;

    private int prevX = 0;
    private int prevY = 0;
    private double prevHeading = 0;

    public int xPositionAbs = 0;
    public int yPositionAbs = 0;

    private boolean positionThread = false;
    public boolean driveThread = false;

    public PID xPID = new PID();
    private PID yPID = new PID();
    public PID turnPID = new PID();

    private LinearOpMode opMode;
    private IMU imu;
    private DcMotor xWheel, yWheel, frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor;
    private VectorDrive drive;

    int xTarget;
    int yTarget;
    int headingTarget;
    int heading;

    private double P_C_AD = .009;
    private double I_C_AD = 0;
    private double D_C_AD = .009;

    private static final double P_C_TURN = 0.011;
    private static final double I_C_TURN = 0.0000125;
    private static final double D_C_TURN = 0;

    public double yForwrd;
    public double xStrafe;

    public double angleError;
    public double angleDelta;

    public boolean stop = false;

    private ArrayLogging logging;
    private ElapsedTime loggingTime = new ElapsedTime();
    private int loggingCount = 0;
    public boolean loggingOn = false;

    public PositionTracking (LinearOpMode opMode, IMU imu,
                             DcMotor xWheel, DcMotor yWheel,
                             VectorDrive drive) {
        this.opMode = opMode;
        this.imu = imu;
        this.xWheel = xWheel;
        this.yWheel = yWheel;
        this.drive = drive;

        turnPID.setVariables(.08,0,1);

        //used for logging debug
        this.logging = new ArrayLogging(13,10000);

        loggingTime.reset();

        //log initial headers
        logging.storeValue(0,0,"Count #");
        logging.storeValue(1,0,"Time");
        logging.storeValue(2,0,"Heading Target");
        logging.storeValue(3,0,"Heading");
        logging.storeValue(4,0,"X Target");
        logging.storeValue(5,0,"xPositionAbs");
        logging.storeValue(6,0,"Y Target");
        logging.storeValue(7,0,"yPositionAbs");
        logging.storeValue(8,0,"xPower");
        logging.storeValue(9,0,"yPower");
        logging.storeValue(10,0,"pidOffset Power");
        logging.storeValue(11,0,"turnPower");
    }

    public int xPosition() {
        return xPositionAbs;
    }

    public int yPosition() {
        return yPositionAbs;
    }

    public void zeroEncoders () {
        xPositionAbs = 0;
        yPositionAbs = 0;
        xWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int xInchCounts (double inches) {
        return (int)(inches * xPerInch);
    }

    public void absolutePositioning (int xInput, int yInput, double heading) {
        angleDelta = (heading - prevHeading);
        angleDelta -= (360*Math.floor(0.5+((angleDelta)/360.0)));

        prevHeading = heading;

        int xDelta = xInput - prevX;
        prevX = xInput;

        int yDelta = yInput - prevY;
        prevY = yInput;

        double strafe = xDelta - (xOffset*angleDelta/360);
        double forwrd = yDelta - (yOffset*angleDelta/360);

        xStrafe += strafe;
        yForwrd += forwrd;

        double gyro_radians = Math.toRadians(heading);
        double temp = forwrd * cos(gyro_radians) + strafe * sin(gyro_radians);
        strafe = -forwrd * sin(gyro_radians) + strafe * cos(gyro_radians);
        forwrd = temp;

        yPositionAbs += (int)forwrd;
        xPositionAbs += (int)strafe;
    }

    public void stopTracking () {
        positionThread = false;
    }

    public void stopDrive () {
        driveThread = false;
        stop = true;

        logging.save("Logging");
    }

    public void drive (int xTarget, int yTarget, int headingTarget, int heading, double xCoefficient, double yCoefficient) {
        double xPower = -xCoefficient * xPid(xTarget,xPositionAbs);
        double yPower = yCoefficient * yPid(yTarget,yPositionAbs);
        double pidOffset = turnPID.run(headingTarget,heading);
        double turnPower = -pidOffset * .2;
        drive.fieldCentric(yPower,xPower,turnPower);
    }

    /*        if (true) {
            loggingCount += 1;
            logging.storeValueInt(0,loggingCount,loggingCount);
            logging.storeValueInt(1,loggingCount,loggingTime.milliseconds());
            logging.storeValueInt(2,loggingCount,headingTarget);
            logging.storeValueInt(3,loggingCount,heading);
            logging.storeValueInt(4,loggingCount,xTarget);
            logging.storeValueInt(5,loggingCount,xPositionAbs);
            logging.storeValueInt(6,loggingCount,yTarget);
            logging.storeValueInt(7,loggingCount,yPositionAbs);
            logging.storeValueInt(8,loggingCount,xPower);
            logging.storeValueInt(9,loggingCount,yPower);
            logging.storeValueInt(10,loggingCount,pidOffset);
            logging.storeValueInt(11,loggingCount,turnPower);
        }
        */

    public void autonomousDrive (int xTarget, int yTarget, int headingTarget, int heading, double xCoefficient, double yCoefficient) {
        stop = false;
        while((  Math.abs(xTarget-xPositionAbs) > xTolerance ||
                Math.abs(yTarget-yPositionAbs) > yTolerance)&&
                !stop
                && opMode.opModeIsActive()
                && !opMode.isStopRequested())
        {
            drive(xTarget,yTarget,headingTarget,heading,xCoefficient,yCoefficient);

        }
        drive.robotCentric(0,0,0);

    }



    public void startAbsoluteEncoderThread() {
        positionThread = true;
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                Thread.currentThread().setPriority(9);
                while(positionThread) {
                    int xPosition = xWheel.getCurrentPosition();
                    int yPosition = yWheel.getCurrentPosition();
                    double heading = imu.getHeading();

                    absolutePositioning(xPosition,yPosition,(360-heading));
                }
            }
        }).start();
    }
    //
    //final int xTarget, final int yTarget, final int headingTarget, final int heading
    public void driveThreadLoop() {
        driveThread = true;
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                /*!opMode.isStopRequested()&&opMode.opModeIsActive()&&*/
                while(driveThread) {
                    double xPower = -0.05 * xPid(xTarget,xPositionAbs);
                    double yPower = 0.03 * yPid(yTarget,yPositionAbs);
                    //0;//yPID.runDistance(yTarget,yPositionAbs);
                    double turnPower = turnPid(headingTarget,heading);

                    drive.robotCentric(yPower,xPower,turnPower);
                }
                drive.robotCentric(0,0,0);

            }
        }).start();
    }

    public void driveToCordinates(final int xTarget, final int yTarget, final int headingTarget, final int heading) {
        while((xTarget-xPositionAbs) > xTolerance
                &&Math.abs(yTarget-yPositionAbs) > yTolerance
                &&Math.abs(headingTarget-heading) > turnTolerance) {

            double xPower = -0.05 * xPid(xTarget,xPositionAbs);
            double yPower = 0.03 * yPid(yTarget,yPositionAbs);
            double turnPower = turnPid(headingTarget,heading);

            drive.fieldCentric(yPower,xPower,turnPower);
        }
        drive.robotCentric(0,0,0);
    }

    //final int xTarget, final int yTarget, final int headingTarget, final int heading
    public void driveToCoordinatesThread(final int xTarget, final int yTarget, final int headingTarget, final int heading) {
        driveThread = true;
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                while(driveThread) {
                    double xPower = -0.05 * xPid(xTarget,xPositionAbs);
                    double yPower = 0.04 * yPid(yTarget,yPositionAbs);
                    double pidOffset = turnPID.run(headingTarget,heading);
                    double turnPower = -pidOffset * .2;

                    drive.robotCentric(0,0,turnPower);

                    //check if in tolerance and end thread if so end
                    if (Math.abs(xTarget-xPositionAbs) < xTolerance
                            &&Math.abs(yTarget-yPositionAbs) < yTolerance
                            &&Math.abs(headingTarget-heading) < turnTolerance) {

                        driveThread = false;
                    }
                }
                drive.robotCentric(0,0,0);
            }
        }).start();
    }

    private double previousErrorY, iErrorY;
    private double yPid (double fixedHeading, double angleCurrent) {
        double pError = fixedHeading - angleCurrent;        //Finds the current difference between the target and current
        iErrorY += pError;
        double dError = pError - previousErrorY;

        previousErrorY = pError;

        return (pError * P_C_AD) + (iErrorY *I_C_AD) + (dError* D_C_AD);
    }

    private double previousErrorX, iErrorX;
    private double xPid (double fixedHeading, double angleCurrent) {
        double pError = fixedHeading - angleCurrent;        //Finds the current difference between the target and current
        iErrorX += pError;
        double dError = pError - previousErrorX;

        previousErrorX = pError;

        return (pError * P_C_AD) + (iErrorX *I_C_AD) + (dError* D_C_AD);
    }

    private double previousErrorTurn, iErrorTurn;
    private double turnPid (double fixedHeading, double angleCurrent) {
        double pError = fixedHeading - angleCurrent;        //Finds the current difference between the target and current
        pError-= (360*Math.floor(0.5+((pError)/360.0)));

        iErrorTurn += pError;
        double dError = pError - previousErrorTurn;

        previousErrorTurn = pError;

        return (pError * P_C_TURN) + (iErrorTurn *I_C_TURN) + (dError* D_C_TURN);
    }

}
