package org.firstinspires.ftc.teamcode.Old_Swerve;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
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
public class HardwareSwerveV1
{
    /* Public OpMode members. */
//Swerve Drivebase Motors
    public DcMotor  DMotor1; //Driver Motor Front (1)
    public DcMotor  DMotor2; //Driver Motor Back (2)
    public DcMotor  PMotor1; //Passenger Motor Front (1)
    public DcMotor  PMotor2; //Passenger Motor Back (2)

//Swerve Drivebase Servos
    public Servo    DServo1; //Driver ServoFront (1)
    public Servo    DServo2; //Driver ServoFront (2)
    public Servo    PServo1; //Passenger ServoFront (1)
    public Servo    PServo2; //Passenger ServoFront (2)

//Swerve Drivebase Encoders
    public AnalogInput  DSensor1; //Driver Sensor Front (1)
    public AnalogInput  DSensor2; //Driver Sensor Back (2)
    public AnalogInput  PSensor1; //Passenger Sensor Front (1)
    public AnalogInput  PSensor2; //Passenger Sensor Back (2)

    public BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareSwerveV1() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        //Define and initialize ALL installed servos.
        DServo1 = hwMap.servo.get("DS1"); //Driver Servo Front(1)
        DServo2 = hwMap.servo.get("DS2"); //Driver Servo Back(2)
        PServo1 = hwMap.servo.get("PS1"); //Pass Servo Front(1)
        PServo2 = hwMap.servo.get("PS2"); //Pass Servo Back(2)*/

        DServo1.setPosition(.5); //Set Driver Servo Front(1) to 0 power
        DServo2.setPosition(.5); //Set Driver Servo Back(2) to 0 power
        PServo1.setPosition(.5); //Set Pass Servo Front(1) to 0 power
        PServo2.setPosition(.5); //Set Pass Servo Back(2) to 0 power*/

        //Define and Initialize Motors
        DMotor1 = hwMap.dcMotor.get("DM1"); //Driver Motor Front(1)
        DMotor2 = hwMap.dcMotor.get("DM2"); //Driver Motor Back(2)
        PMotor1 = hwMap.dcMotor.get("PM1"); //Passenger Motor Front(1)
        PMotor2 = hwMap.dcMotor.get("PM2"); //Passenger Motor Back(2)
        //PMotor1.setDirection(DcMotor.Direction.REVERSE);
        //PMotor2.setDirection(DcMotor.Direction.REVERSE);

        //Set all motors to zero power
        DMotor1.setPower(0); //Set Drive Motor 1 to 0% power
        DMotor2.setPower(0); //Set Drive Motor 2 to 0% power
        PMotor1.setPower(0); //Set Pass Motor 1 to 0% power
        PMotor2.setPower(0); //Set Pass Motor 2 to 0% power

        /*
        //Set all motors to run with encoders.
        DMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Set Drive Motor 1 to use encoder
        DMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Set Drive Motor 2 to use encoder
        PMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Set Pass Motor 1 to use encoder
        PMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Set Pass Motor 2 to use encoder*/

        DMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        DSensor1 = hwMap.analogInput.get("DSe1");
        DSensor2 = hwMap.analogInput.get("DSe2");
        PSensor1 = hwMap.analogInput.get("PSe1");
        PSensor2 = hwMap.analogInput.get("PSe2");


//IMU initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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
