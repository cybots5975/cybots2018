/*@author Karter*/package org.firstinspires.ftc.teamcode.General;import com.qualcomm.robotcore.hardware.AnalogInput;import com.qualcomm.robotcore.hardware.DcMotor;import com.qualcomm.robotcore.hardware.HardwareMap;import com.qualcomm.robotcore.hardware.Servo;import com.qualcomm.robotcore.util.ElapsedTime;import org.firstinspires.ftc.teamcode.Sensors.IMU;import org.firstinspires.ftc.teamcode.Subsystems.Intake;import org.firstinspires.ftc.teamcode.drivebase.swerve.core.SwerveDrive;public class Robot {    //define hardware devices    public DcMotor DMotor1, DMotor2, PMotor1, PMotor2, DSIntakeMotor, PSIntakeMotor;    public Servo DServo1, DServo2, PServo1, PServo2, DSIntakeServo, PSIntakeServo;    public AnalogInput DSensor1, DSensor2, PSensor1, PSensor2;    public SwerveDrive drive;    public Intake intake;    private IMU imu, imu2;    HardwareMap hwMap  = null;    private ElapsedTime period  = new ElapsedTime();    /* Constructor */    public Robot() {    }    /* Initialize standard Hardware interfaces */    public void init(HardwareMap ahwMap) {        // save reference to HW Map        hwMap = ahwMap;        //Define and initialize ALL installed servos.        DServo1 = hwMap.servo.get("DS1"); //Driver Servo Front(1)        DServo2 = hwMap.servo.get("DS2"); //Driver Servo Back(2)        PServo1 = hwMap.servo.get("PS1"); //Pass Servo Front(1)        PServo2 = hwMap.servo.get("PS2"); //Pass Servo Back(2)*/        //Define and Initialize Motors        DMotor1 = hwMap.dcMotor.get("DM1"); //Driver Motor Front(1)        DMotor2 = hwMap.dcMotor.get("DM2"); //Driver Motor Back(2)        PMotor1 = hwMap.dcMotor.get("PM1"); //Passenger Motor Front(1)        PMotor2 = hwMap.dcMotor.get("PM2"); //Passenger Motor Back(2)        DSensor1 = hwMap.analogInput.get("DSe1");        DSensor2 = hwMap.analogInput.get("DSe2");        PSensor1 = hwMap.analogInput.get("PSe1");        PSensor2 = hwMap.analogInput.get("PSe2");        imu = new IMU();        imu2 = new IMU();        imu.initIMU(hwMap,"imu");        imu2.initIMU(hwMap,"imu2");        //MA3Encoder test = new MA3Encoder(hwMap,"DSe");        DServo1.setPosition(.5); //Set Driver Servo Front(1) to 0 power        DServo2.setPosition(.5); //Set Driver Servo Back(2) to 0 power        PServo1.setPosition(.5); //Set Pass Servo Front(1) to 0 power        PServo2.setPosition(.5); //Set Pass Servo Back(2) to 0 power        DMotor1.setDirection(DcMotor.Direction.FORWARD);        DMotor2.setDirection(DcMotor.Direction.FORWARD);        PMotor1.setDirection(DcMotor.Direction.REVERSE);        PMotor2.setDirection(DcMotor.Direction.REVERSE);        //Set all motors to zero power        DMotor1.setPower(0); //Set Drive Motor 1 to 0% power        DMotor2.setPower(0); //Set Drive Motor 2 to 0% power        PMotor1.setPower(0); //Set Pass Motor 1 to 0% power        PMotor2.setPower(0); //Set Pass Motor 2 to 0% power        //Set all motors to run with encoders.        DMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Set Drive Motor 1 to use encoder        DMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Set Drive Motor 2 to use encoder        PMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Set Pass Motor 1 to use encoder        PMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Set Pass Motor 2 to use encoder        DMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);        DMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);        PMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);        PMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);        SwerveDrive drive = new SwerveDrive(imu, imu2,                DMotor1,DServo1,DSensor1,                DMotor2,DServo2,DSensor2,                PMotor1,PServo1,PSensor1,                PMotor2,PServo2,PSensor2);        this.drive = drive;        Intake intake = new Intake(DSIntakeMotor,PSIntakeMotor,DSIntakeServo,PSIntakeServo);        this.intake = intake;        intake.store();    }    public void stop(){        //Set all motors to zero power        DMotor1.setPower(0); //Set Drive Motor 1 to 0% power        DMotor2.setPower(0); //Set Drive Motor 2 to 0% power        PMotor1.setPower(0); //Set Pass Motor 1 to 0% power        PMotor2.setPower(0); //Set Pass Motor 2 to 0% power        DServo1.setPosition(.5); //Set Driver Servo Front(1) to 0 power        DServo2.setPosition(.5); //Set Driver Servo Back(2) to 0 power        PServo1.setPosition(.5); //Set Pass Servo Front(1) to 0 power        PServo2.setPosition(.5); //Set Pass Servo Back(2) to 0 power*/    }}