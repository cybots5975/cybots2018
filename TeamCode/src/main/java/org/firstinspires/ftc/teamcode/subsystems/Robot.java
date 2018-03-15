/*
@author Karter
*/
package org.firstinspires.ftc.teamcode.subsystems;

import android.speech.tts.TextToSpeech;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;
import com.disnodeteam.dogecv.detectors.GenericDetector;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.drivebase.VectorDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivebase.VectorDrive.driveType;
import org.firstinspires.ftc.teamcode.subsystems.drivebase.mecanum.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivebase.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.sensors.IMU;
import org.firstinspires.ftc.teamcode.subsystems.sensors.MA3Encoder;
import org.firstinspires.ftc.teamcode.subsystems.sensors.PositionTracking;
import org.firstinspires.ftc.teamcode.util.CybotsVisionConfig;
import org.firstinspires.ftc.teamcode.util.DotStar;
import org.firstinspires.ftc.teamcode.util.ReadPrefs;
import org.firstinspires.ftc.teamcode.util.open.OpenRevDcMotorImplEx;
import org.firstinspires.ftc.teamcode.util.open.OpenRevHub;
import org.firstinspires.ftc.teamcode.util.vuforia.CybotVuMark;

import java.util.Locale;
import java.util.Objects;

import static org.firstinspires.ftc.teamcode.subsystems.drivebase.VectorDrive.driveType.MECANUM;

public class Robot{
    //define hardware devices
    public driveType driveType = MECANUM;

    public OpenRevHub revHubDS;

    public VectorDrive drive;
    public DcMotorEx DMotor1, DMotor2, PMotor1, PMotor2;
    public OpenRevDcMotorImplEx IntakeMotor, RelicMotor;
    public DcMotorEx ArmMotor;
    public CRServo DServo1, DServo2, PServo1, PServo2;
    public Servo PPinch, DPinch, DSIntakeServo, PSIntakeServo;
    public ServoImplEx JewelKick, JewelArm, RelicGrab, RelicPivot;
    //public VexMotor DSwervo1, DSwervo2, PSwervo1, PSwervo2;
    public AnalogInput DSensor1, DSensor2, PSensor1, PSensor2;
    public AnalogInput boxLimit;
    public ColorSensor glyphColor1, glyphColor2, glyphColor3, glyphColor4;
    public DistanceSensor glyphDistance1, glyphDistance2, glyphDistance3, glyphDistance4;
    public Intake intake;
    public PositionTracking positionTracking;

    public MA3Encoder xWheel;
    public MA3Encoder yWheel;

    public double xPosition, yPosition;

    public GlyphMech glyphMech;

    public RelicArm relicArm;

    public IMU imu, imu2;
    public ReadPrefs prefs;

    DotStar dotStar;

    //old
    public final double kickCenter = .375;
    public final double raisedArm  = .02, middleArm = .9,   loweredArm = 1;


    public final double armInit = .830,kickInit = .005;
    public final double armLow = .137, kickLow = .5;
    public final double kickLeft = .05, kickRight = .85;

    public LinearOpMode opMode;
    public HardwareMap hwMap;
    public CybotVuMark VuMark1;
    public CybotsVisionConfig jewelVision;
    public boolean Vuforia = false;
    public boolean JewelVision = false;
    public boolean JewelHybridFrames = true;
    public boolean BoxVision = false;
    public boolean isTeleop = false;
    public JewelDetector.JewelOrder jewelOrder;

    public TextToSpeech tts;

    public CryptoboxDetector.CryptoboxDetectionMode color;

    public GenericDetector relicDetector = null;

    //experimental
    public CryptoboxDetector cryptoboxDetector = null;

    public ElapsedTime runtime  = new ElapsedTime();

    /* Constructor */
    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    //this is here for old opModes still
    public void init() {
        hwMap = opMode.hardwareMap;
        prefs = new ReadPrefs(hwMap);
        initLEDs();
        //Define and initialize ALL installed servos.
        DServo1 = hwMap.crservo.get("DS1"); //Driver Servo Front(1)
        DServo2 = hwMap.crservo.get("DS2"); //Driver Servo Back(2)
        PServo1 = hwMap.crservo.get("PS1"); //Pass Servo Front(1)
        PServo2 = hwMap.crservo.get("PS2"); //Pass Servo Back(2)*/

        DSIntakeServo = (ServoImplEx) hwMap.servo.get("DIn");
        PSIntakeServo = (ServoImplEx) hwMap.servo.get("PIn");

        PPinch = hwMap.servo.get("PPinch");
        DPinch = hwMap.servo.get("DPinch");

        JewelArm = (ServoImplEx) hwMap.servo.get("Arm");
        //.02 is back
        //1 is down

        JewelKick = (ServoImplEx) hwMap.servo.get("Kick");
        //.45 is centered
        //0 kicks left, 1 kicks right

        RelicGrab = (ServoImplEx) hwMap.servo.get("RelicGrab");
        RelicPivot = (ServoImplEx) hwMap.servo.get("RelicPivot");

        //Define and Initialize Motors
        DMotor1 = (DcMotorEx) hwMap.dcMotor.get("DM1"); //Driver Motor Front(1)
        DMotor2 = (DcMotorEx) hwMap.dcMotor.get("DM2"); //Driver Motor Back(2)
        PMotor1 = (DcMotorEx) hwMap.dcMotor.get("PM1"); //Passenger Motor Front(1)
        PMotor2 = (DcMotorEx) hwMap.dcMotor.get("PM2"); //Passenger Motor Back(2)

        IntakeMotor = new OpenRevDcMotorImplEx((DcMotorImplEx) hwMap.dcMotor.get("InM"));

        ArmMotor = (DcMotorEx) hwMap.dcMotor.get("Glyph");

        RelicMotor = new OpenRevDcMotorImplEx((DcMotorImplEx) hwMap.dcMotor.get("RM"));

        revHubDS = new OpenRevHub(hwMap.get(LynxModule.class, "DS"));

        DSensor1 = hwMap.analogInput.get("DSe1");
        DSensor2 = hwMap.analogInput.get("DSe2");
        PSensor1 = hwMap.analogInput.get("PSe1");
        PSensor2 = hwMap.analogInput.get("PSe2");

        this.xWheel = new MA3Encoder(hwMap,"xWheel");
        this.yWheel = new MA3Encoder(hwMap,"yWheel");

        boxLimit = hwMap.analogInput.get("box");

        glyphColor1 = hwMap.get(ColorSensor.class, "glyph1");
        glyphDistance1 = hwMap.get(DistanceSensor.class, "glyph1");
        glyphColor2 = hwMap.get(ColorSensor.class, "glyph2");
        glyphDistance2 = hwMap.get(DistanceSensor.class, "glyph2");
        glyphColor3 = hwMap.get(ColorSensor.class, "glyph3");
        glyphDistance3 = hwMap.get(DistanceSensor.class, "glyph3");
        glyphColor4 = hwMap.get(ColorSensor.class, "glyph4");
        glyphDistance4 = hwMap.get(DistanceSensor.class, "glyph4");

        opMode.telemetry.addData("Motors","Init Done");
        opMode.telemetry.update();

        //ServoImplEx myServo = (ServoImplEx)hwMap.servo.get("myservo");

        imu = new IMU();
        imu2 = new IMU();
        imu.initIMU(hwMap,"imu");
        imu2.initIMU(hwMap,"imu2");
        imu.setHeadingOffset(0);
        imu2.setHeadingOffset(0);

        opMode.telemetry.addData("IMU","Init Done");
        opMode.telemetry.update();

        //MA3Encoder test = new MA3Encoder(hwMap,"DSe");

        DServo1.setPower(0); //Set Driver Servo Front(1) to 0 power
        DServo2.setPower(0); //Set Driver Servo Back(2) to 0 power
        PServo1.setPower(0); //Set Pass Servo Front(1) to 0 power
        PServo2.setPower(0); //Set Pass Servo Back(2) to 0 power

        PPinch.setDirection(Servo.Direction.REVERSE);

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

        RelicMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake intake = new Intake(IntakeMotor);
        this.intake = intake;

        this.positionTracking = new PositionTracking(xWheel,yWheel,imu,PSIntakeServo,DSIntakeServo);

        GlyphMech glyphMech = new GlyphMech(ArmMotor,PPinch,DPinch,boxLimit,opMode.isStopRequested());
        this.glyphMech = glyphMech;

        glyphMech.init();

        RelicArm relicArm = new RelicArm(RelicMotor,RelicGrab,RelicPivot);
        this.relicArm = relicArm;
        relicArm.init();

        if (isTeleop) {
            JewelArm.setPwmDisable();
            JewelKick.setPwmDisable();

        } else {
            JewelArm.setPwmEnable();
            JewelKick.setPwmEnable();
            JewelArm.setPosition(armInit);
            JewelKick.setPosition(kickInit);
            intake.store();
        }

        if (Vuforia) {
            this.VuMark1 = new CybotVuMark(hwMap,VuforiaLocalizer.CameraDirection.BACK,true);
        }

        if (JewelVision) {
            this.jewelVision = new CybotsVisionConfig(hwMap,JewelHybridFrames);
        }

        if (BoxVision) {
            cryptoboxDetector = new CryptoboxDetector();
            cryptoboxDetector.init(opMode.hardwareMap.appContext, CameraViewDisplay.getInstance());
            cryptoboxDetector.rotateMat = false;
            cryptoboxDetector.detectionMode = color;
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

        initTTS();
        opMode.telemetry.addData("TTS:","Init Done");

        opMode.telemetry.addData("INIT WHOLE:","Init Done");
        opMode.telemetry.update();

        opMode.telemetry.clearAll();
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
        while (runtime.seconds()<seconds&&!opMode.isStopRequested()&&opMode.opModeIsActive()) {
            //opMode.telemetry.addData("Waiting",seconds-runtime.seconds());
            //opMode.telemetry.update();
            //waiting
        }
    }

    public void pause(double seconds, boolean log){
        runtime.reset();
        while (runtime.seconds()<seconds&&!opMode.isStopRequested()&&opMode.opModeIsActive()) {
            opMode.telemetry.addData("Waiting",seconds-runtime.seconds());
            opMode.telemetry.update();
            //waiting
        }
    }

    private void initTTS()
    {
        tts = new TextToSpeech(opMode.hardwareMap.appContext, null);
        tts.setLanguage(Locale.UK);
        tts.setPitch(.5f);
        tts.setSpeechRate(.8f);
    }

    public void speak(final String text) {
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                tts.speak(text, TextToSpeech.QUEUE_FLUSH, null);
            }
        }).start();
    }

    public enum ledColor {red, blue, green}

    public void setLedColor(ledColor color) {
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                dotStar = new DotStar(opMode.hardwareMap,"ci","di");
                boolean loop = true;
                while(loop&&!opMode.isStopRequested()) {
                    switch (color) {
                        case red:
                            dotStar.setEntireStrip((byte)255, (byte)0, (byte)0);
                            break;
                        case blue:
                            dotStar.setEntireStrip((byte)0, (byte)0, (byte)200);
                            break;
                        case green:
                            dotStar.setEntireStrip((byte)0, (byte)255, (byte)0);
                            break;
                    }

                    loop = false;
                }
            }
        }).start();
    }

    public void initRelicVision() {
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                boolean loop = true;
                while(loop&&!opMode.isStopRequested()&&opMode.opModeIsActive()) {
                    relicDetector = new GenericDetector();
                    relicDetector.init(opMode.hardwareMap.appContext, CameraViewDisplay.getInstance());
                    relicDetector.enable();

                    loop = false;
                }
            }
        }).start();
    }

    public void disableRelicVision() {
        relicDetector.disable();
    }

    public void initLEDs() {
        String sideColor;
        sideColor = prefs.read("color");
        if (Objects.equals(sideColor,"red")) {
            setLedColor(Robot.ledColor.red);
        } else if (Objects.equals(sideColor,"blue")) {
            setLedColor(Robot.ledColor.blue);
        } else {
            setLedColor(Robot.ledColor.green);
        }
    }

    public void startEncoderTracking() {
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                while(!opMode.isStopRequested()&&opMode.opModeIsActive()) {
                    xPosition = xWheel.getIncremental();
                    yPosition = yWheel.getIncremental();

                }
            }
        }).start();
    }

/*    public void logging() {
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {

            }
        }).start();
    }*/
}