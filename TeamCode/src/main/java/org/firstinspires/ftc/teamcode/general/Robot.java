/*@author Karter*/package org.firstinspires.ftc.teamcode.general;import com.qualcomm.hardware.lynx.LynxServoController;import com.qualcomm.robotcore.hardware.AnalogInput;import com.qualcomm.robotcore.hardware.CRServo;import com.qualcomm.robotcore.hardware.DcMotor;import com.qualcomm.robotcore.hardware.HardwareMap;import com.qualcomm.robotcore.hardware.Servo;import com.qualcomm.robotcore.util.ElapsedTime;import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;import org.firstinspires.ftc.teamcode.drivebase.swerve.core.SwerveDrive;import org.firstinspires.ftc.teamcode.general.vuforia.KarterVuMark1;import org.firstinspires.ftc.teamcode.sensors.IMU;import org.firstinspires.ftc.teamcode.subsystems.Intake;public class Robot {    //define hardware devices    public DcMotor DMotor1, DMotor2, PMotor1, PMotor2, IntakeMotor;    public CRServo DServo1, DServo2, PServo1, PServo2;    public Servo DSIntakeServo, PSIntakeServo, JewelArm;    public AnalogInput DSensor1, DSensor2, PSensor1, PSensor2;    public SwerveDrive drive;    public Intake intake;    public KarterVuMark1 VuMark1;    private IMU imu, imu2;    public boolean Vuforia = false;    HardwareMap hwMap  = null;    private ElapsedTime period  = new ElapsedTime();    /* Constructor */    public Robot() {    }    /* Initialize standard Hardware interfaces */    public void init(HardwareMap ahwMap) {        // save reference to HW Map        hwMap = ahwMap;        //Define and initialize ALL installed servos.        DServo1 = hwMap.crservo.get("DS1"); //Driver Servo Front(1)        DServo2 = hwMap.crservo.get("DS2"); //Driver Servo Back(2)        PServo1 = hwMap.crservo.get("PS1"); //Pass Servo Front(1)        PServo2 = hwMap.crservo.get("PS2"); //Pass Servo Back(2)*/        DSIntakeServo = hwMap.servo.get("DIn");        PSIntakeServo = hwMap.servo.get("PIn");        JewelArm = hwMap.servo.get("Arm");        //Define and Initialize Motors        DMotor1 = hwMap.dcMotor.get("DM1"); //Driver Motor Front(1)        DMotor2 = hwMap.dcMotor.get("DM2"); //Driver Motor Back(2)        PMotor1 = hwMap.dcMotor.get("PM1"); //Passenger Motor Front(1)        PMotor2 = hwMap.dcMotor.get("PM2"); //Passenger Motor Back(2)        IntakeMotor = hwMap.dcMotor.get("InM");        DSensor1 = hwMap.analogInput.get("DSe1");        DSensor2 = hwMap.analogInput.get("DSe2");        PSensor1 = hwMap.analogInput.get("PSe1");        PSensor2 = hwMap.analogInput.get("PSe2");//        ServoImplEx myServo = (ServoImplEx)hwMap.servo.get("myservo");        imu = new IMU();        imu2 = new IMU();        imu.initIMU(hwMap,"imu");        imu2.initIMU(hwMap,"imu2");        //MA3Encoder test = new MA3Encoder(hwMap,"DSe");        LynxServoController servoController = (LynxServoController)DSIntakeServo.getController();        DServo1.setPower(0); //Set Driver Servo Front(1) to 0 power        DServo2.setPower(0); //Set Driver Servo Back(2) to 0 power        PServo1.setPower(0); //Set Pass Servo Front(1) to 0 power        PServo2.setPower(0); //Set Pass Servo Back(2) to 0 power        JewelArm.setPosition(0);        /*servoController.setServoPwmDisable(4);        servoController.setServoPwmDisable(5);*/        DMotor1.setDirection(DcMotor.Direction.FORWARD);        DMotor2.setDirection(DcMotor.Direction.FORWARD);        PMotor1.setDirection(DcMotor.Direction.REVERSE);        PMotor2.setDirection(DcMotor.Direction.REVERSE);        //Set all motors to zero power        DMotor1.setPower(0); //Set Drive Motor 1 to 0% power        DMotor2.setPower(0); //Set Drive Motor 2 to 0% power        PMotor1.setPower(0); //Set Pass Motor 1 to 0% power        PMotor2.setPower(0); //Set Pass Motor 2 to 0% power        //Set all motors to run with encoders.        DMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Set Drive Motor 1 to use encoder        DMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Set Drive Motor 2 to use encoder        PMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Set Pass Motor 1 to use encoder        PMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Set Pass Motor 2 to use encoder        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);        DMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);        DMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);        PMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);        PMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);        SwerveDrive drive = new SwerveDrive(imu, imu2,                DMotor1,DServo1,DSensor1,                DMotor2,DServo2,DSensor2,                PMotor1,PServo1,PSensor1,                PMotor2,PServo2,PSensor2);        this.drive = drive;        Intake intake = new Intake(IntakeMotor,DSIntakeServo,PSIntakeServo);        this.intake = intake;        intake.store();        if (Vuforia) {            KarterVuMark1 VuMark1 = new KarterVuMark1(hwMap,VuforiaLocalizer.CameraDirection.BACK,true);            this.VuMark1 = VuMark1;        }    }    public void stop(){        //Set all motors to zero power        DMotor1.setPower(0); //Set Drive Motor 1 to 0% power        DMotor2.setPower(0); //Set Drive Motor 2 to 0% power        PMotor1.setPower(0); //Set Pass Motor 1 to 0% power        PMotor2.setPower(0); //Set Pass Motor 2 to 0% power        DServo1.setPower(0); //Set Driver Servo Front(1) to 0 power        DServo2.setPower(0); //Set Driver Servo Back(2) to 0 power        PServo1.setPower(0); //Set Pass Servo Front(1) to 0 power        PServo2.setPower(0); //Set Pass Servo Back(2) to 0 power*/    }}