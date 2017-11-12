package org.firstinspires.ftc.teamcode.oldRobots;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Servo channel:  Servo to raise/lower arm: "arm"
 * Servo channel:  Servo to open/close claw: "claw"
 *
 * Note: the configuration of the servos is such that:
 *   As the arm servo approaches 0, the arm position moves up (away from the floor).
 *   As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class HardwareTOM
{
    /* Public OpMode members. */
    public DcMotor driveMotor1     =null;
    public DcMotor passMotor1      =null;
    public DcMotor driveMotor2     =null;
    public DcMotor passMotor2      =null;
    public DcMotor sweepMotor1     =null;
    public DcMotor turretMotor1    =null;
    public DcMotor shootMotor1     =null;
    public DcMotor shootMotor2     =null;
    public Servo rackDS1;
    public Servo rackDS2;
    public Servo rackPS1;
    public Servo rackPS2;
    public Servo ballLift;
    public Servo chute;
    public Servo pixyServo;
    public Servo rearShifter;
    public Servo middleShifter;
    public Servo capBall;
    public Servo dSweep;
    public Servo pSweep;


    public ModernRoboticsI2cGyro gyro;
    public ModernRoboticsI2cColorSensor lineColor;
    public ModernRoboticsI2cColorSensor lineColor2;
    public ModernRoboticsAnalogOpticalDistanceSensor dsODS1;
    public ModernRoboticsI2cRangeSensor dsRange1;
    public ModernRoboticsI2cRangeSensor psRange1;
    public ModernRoboticsI2cColorSensor dsColor1;
    public ModernRoboticsI2cColorSensor psColor1;

    public double xTrack;
    public double yTrack;


    public final static double ARM_HOME = 0.2;
    public final static double CLAW_HOME = 0.2;
    public final static double ARM_MIN_RANGE  = 0.20;
    public final static double ARM_MAX_RANGE  = 0.90;
    public final static double CLAW_MIN_RANGE  = 0.20;
    public final static double CLAW_MAX_RANGE  = 0.7;

    public double dsRack1_Min = 129;
    public double dsRack1_Max = 22; //y goes out
    public double dsRack2_Min = 99;
    public double dsRack2_Max = 207; //a goes out

    public double psRack1_Min = 109;
    public double psRack1_Max = 222; //a goes out
    public double psRack2_Min = 157;
    public double psRack2_Max = 233; //a goes out


    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareTOM() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        driveMotor1     = hwMap.dcMotor.get("ds1");
        passMotor1      = hwMap.dcMotor.get("ps1");
        driveMotor2     = hwMap.dcMotor.get("ds2");
        passMotor2      = hwMap.dcMotor.get("ps2");
        driveMotor1.setDirection(DcMotor.Direction.REVERSE);
        driveMotor2.setDirection(DcMotor.Direction.REVERSE);
        sweepMotor1     = hwMap.dcMotor.get("sw1");
        turretMotor1    = hwMap.dcMotor.get("tm1");
        shootMotor1     = hwMap.dcMotor.get("sm1");
        shootMotor2     = hwMap.dcMotor.get("sm2");
        shootMotor2.setDirection(DcMotor.Direction.REVERSE);
        rackDS1         = hwMap.servo.get("dsRack1");
        rackDS2         = hwMap.servo.get("dsRack2");
        rackPS1         = hwMap.servo.get("psRack1");
        rackPS2         = hwMap.servo.get("psRack2");
        ballLift        = hwMap.servo.get("ballLift");
        chute           = hwMap.servo.get("chute");
        pixyServo       = hwMap.servo.get("pixyServo");
        rearShifter     = hwMap.servo.get("rShift");
        middleShifter   = hwMap.servo.get("mShift");
        capBall         = hwMap.servo.get("capBall");
        dSweep          = hwMap.servo.get("dSweep");
        pSweep          = hwMap.servo.get("pSweep");

        //lineColor.setI2cAddress(I2cAddr.create8bit(0x3C));
        gyro = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");
        lineColor = (ModernRoboticsI2cColorSensor)hwMap.colorSensor.get("lineColor");
        lineColor.setI2cAddress(I2cAddr.create8bit(0x3c));
        lineColor.enableLed(true); // Turn the LED on

        lineColor2 = (ModernRoboticsI2cColorSensor)hwMap.colorSensor.get("lineColor2");
        lineColor2.setI2cAddress(I2cAddr.create8bit(0x22));
        lineColor.enableLed(true);

        psColor1 = (ModernRoboticsI2cColorSensor)hwMap.colorSensor.get("psColor1");
        psColor1.setI2cAddress(I2cAddr.create8bit(0x10));
        psColor1.enableLed(false);

        dsColor1 = (ModernRoboticsI2cColorSensor)hwMap.colorSensor.get("dsColor1");
        dsColor1.setI2cAddress(I2cAddr.create8bit(0x20));
        dsColor1.enableLed(false);

        dsRange1 = hwMap.get(ModernRoboticsI2cRangeSensor.class, "dsRange1");
        //dsRange1.setI2cAddress(I2cAddr.create8bit(0x28));

       /*psRange1 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "psRange1");
       psRange1.setI2cAddress(I2cAddr.create8bit(0x28));*/

        dsODS1 = (ModernRoboticsAnalogOpticalDistanceSensor)hwMap.opticalDistanceSensor.get("dsODS1");


        // Set all motors to zero power
        driveMotor1.setPower(0);
        passMotor1.setPower(0);
        driveMotor2.setPower(0);
        passMotor2.setPower(0);
        sweepMotor1.setPower(0);
        turretMotor1.setPower(0);
        shootMotor1.setPower(0);
        shootMotor2.setPower(0);

        ballLift.setPosition(.5);

        chute.setPosition(.84);

        rackDS1.setPosition(dsRack1_Min);
        rackDS2.setPosition(dsRack2_Min);
        rackPS1.setPosition(psRack1_Min);
        rackPS2.setPosition(psRack2_Min);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        driveMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        passMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        passMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sweepMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shootMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //driveMotor1.setMaxSpeed(1250); //sets top speed to 3.5 ft/sec based on 4in wheel and 3:1 ratio
        //passMotor1.setMaxSpeed(1250);
        //driveMotor2.setMaxSpeed(1250);
        //passMotor2.setMaxSpeed(1250);


        //pixy2.sendWriteCommand(0x3D, 1, new byte[] {0x0C});

        // Define and initialize ALL installed servos.
        //arm = hwMap.servo.get("arm");
        //claw = hwMap.servo.get("claw");
        //arm.setPosition(ARM_HOME);
        //claw.setPosition(CLAW_HOME);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

