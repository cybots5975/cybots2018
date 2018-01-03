/*
@author Karter
*/
package org.firstinspires.ftc.teamcode.subsystems;

import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.drivebase.VectorDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivebase.VectorDrive.driveType;
import org.firstinspires.ftc.teamcode.subsystems.drivebase.mecanum.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivebase.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.sensors.IMU;
import org.firstinspires.ftc.teamcode.util.CybotsVisionConfig;
import org.firstinspires.ftc.teamcode.util.ReadPrefs;
import org.firstinspires.ftc.teamcode.util.vuforia.CybotVuMark;

import static org.firstinspires.ftc.teamcode.subsystems.drivebase.VectorDrive.driveType.MECANUM;

public class Robot{
    //define hardware devices
    public driveType driveType = MECANUM;
    public VectorDrive drive;
    public DcMotor DMotor1, DMotor2, PMotor1, PMotor2, IntakeMotor;
    public CRServo DServo1, DServo2, PServo1, PServo2;
    public Servo DSIntakeServo, PSIntakeServo, JewelArm, JewelKick;
    //public VexMotor DSwervo1, DSwervo2, PSwervo1, PSwervo2;
    public AnalogInput DSensor1, DSensor2, PSensor1, PSensor2;
    public Intake intake;
    public IMU imu, imu2;
    public ReadPrefs prefs;

    public final double kickLeft   = 0,   kickCenter = .45, kickRight = 1;
    public final double raisedArm  = .02, middleArm = .5,   loweredArm = 1;

    public LinearOpMode opMode;
    public HardwareMap hwMap;
    public CybotVuMark VuMark1;
    public CybotsVisionConfig jewelVision;
    public boolean Vuforia = false;
    public boolean JewelVision = false;
    public JewelDetector.JewelOrder jewelOrder;

    public ElapsedTime runtime  = new ElapsedTime();

    /* Constructor */
    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    //this is here for old opModes still
    public void init() {
        hwMap = opMode.hardwareMap;
        //Define and initialize ALL installed servos.
        DServo1 = hwMap.crservo.get("DS1"); //Driver Servo Front(1)
        DServo2 = hwMap.crservo.get("DS2"); //Driver Servo Back(2)
        PServo1 = hwMap.crservo.get("PS1"); //Pass Servo Front(1)
        PServo2 = hwMap.crservo.get("PS2"); //Pass Servo Back(2)*/

        DSIntakeServo = hwMap.servo.get("DIn");
        PSIntakeServo = hwMap.servo.get("PIn");

        JewelArm = hwMap.servo.get("Arm");
        //.02 is back
        //1 is down

        JewelKick = hwMap.servo.get("Kick");
        //.45 is centered
        //0 kicks left, 1 kicks right

        //Define and Initialize Motors
        DMotor1 = hwMap.dcMotor.get("DM1"); //Driver Motor Front(1)
        DMotor2 = hwMap.dcMotor.get("DM2"); //Driver Motor Back(2)
        PMotor1 = hwMap.dcMotor.get("PM1"); //Passenger Motor Front(1)
        PMotor2 = hwMap.dcMotor.get("PM2"); //Passenger Motor Back(2)

        IntakeMotor = hwMap.dcMotor.get("InM");

        DSensor1 = hwMap.analogInput.get("DSe1");
        DSensor2 = hwMap.analogInput.get("DSe2");
        PSensor1 = hwMap.analogInput.get("PSe1");
        PSensor2 = hwMap.analogInput.get("PSe2");

        //ServoImplEx myServo = (ServoImplEx)hwMap.servo.get("myservo");

        imu = new IMU();
        imu2 = new IMU();
        imu.initIMU(hwMap,"imu");
        imu2.initIMU(hwMap,"imu2");
        imu.setHeadingOffset(0);
        imu2.setHeadingOffset(0);

        //MA3Encoder test = new MA3Encoder(hwMap,"DSe");

        DServo1.setPower(0); //Set Driver Servo Front(1) to 0 power
        DServo2.setPower(0); //Set Driver Servo Back(2) to 0 power
        PServo1.setPower(0); //Set Pass Servo Front(1) to 0 power
        PServo2.setPower(0); //Set Pass Servo Back(2) to 0 power

        JewelArm.setPosition(raisedArm);
        JewelKick.setPosition(kickCenter);

        DMotor1.setDirection(DcMotor.Direction.FORWARD);
        DMotor2.setDirection(DcMotor.Direction.FORWARD);
        PMotor1.setDirection(DcMotor.Direction.REVERSE);
        PMotor2.setDirection(DcMotor.Direction.REVERSE);

        //Set all motors to zero power
        DMotor1.setPower(0); //Set Drive Motor 1 to 0% power
        DMotor2.setPower(0); //Set Drive Motor 2 to 0% power
        PMotor1.setPower(0); //Set Pass Motor 1 to 0% power
        PMotor2.setPower(0); //Set Pass Motor 2 to 0% power

        //Set all motors to run with encoders.
        DMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Set Drive Motor 1 to use encoder
        DMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Set Drive Motor 2 to use encoder
        PMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Set Pass Motor 1 to use encoder
        PMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Set Pass Motor 2 to use encoder

        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake intake = new Intake(IntakeMotor,DSIntakeServo,PSIntakeServo);
        this.intake = intake;

        intake.store();

        if (Vuforia) {
            this.VuMark1 = new CybotVuMark(hwMap,VuforiaLocalizer.CameraDirection.BACK,true);
        }

        if (JewelVision) {
            this.jewelVision = new CybotsVisionConfig(hwMap);
        }

        if (driveType.equals(MECANUM)) {
            drive = new MecanumDrive(
                    opMode, imu, imu2,
                    DMotor1, DServo1, DSensor1,
                    DMotor2, DServo2, DSensor2,
                    PMotor1, PServo1, PSensor1,
                    PMotor2, PServo2, PSensor2);
        }else {
            drive = new SwerveDrive(
                    opMode, imu, imu2,
                    DMotor1, DServo1, DSensor1,
                    DMotor2, DServo2, DSensor2,
                    PMotor1, PServo1, PSensor1,
                    PMotor2, PServo2, PSensor2);
        }

        prefs = new ReadPrefs(hwMap);
    }

    //stop all robot movements
    public void stop(){
        //set all motors to zero power
        DMotor1.setPower(0);
        DMotor2.setPower(0);
        PMotor1.setPower(0);
        PMotor2.setPower(0);
        //set servos to stop
        DServo1.setPower(0);
        DServo2.setPower(0);
        PServo1.setPower(0);
        PServo2.setPower(0);
    }

    public void pause(double seconds){
        runtime.reset();
        while (runtime.seconds()<seconds&&!opMode.isStopRequested()) {
            opMode.telemetry.addData("Waiting",seconds-runtime.seconds());
            opMode.telemetry.update();
            //waiting
        }
    }
}