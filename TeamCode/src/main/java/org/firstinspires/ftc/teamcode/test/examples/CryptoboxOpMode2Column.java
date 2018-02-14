package org.firstinspires.ftc.teamcode.test.examples;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.box2column;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="DogeCV Red Cryptobox Detector 2 Col", group="DogeCV")
//@Disabled
public class CryptoboxOpMode2Column extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private box2column cryptoboxDetector = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        cryptoboxDetector = new box2column();
        cryptoboxDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        cryptoboxDetector.rotateMat = false;

        //Optional Test Code to load images via Drawables
        //cryptoboxDetector.useImportedImage = true;
        //cryptoboxDetector.SetTestMat(com.qualcomm.ftcrobotcontroller.R.drawable.test_cv4);

        cryptoboxDetector.enable();
        //
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("isCryptoBoxDetected", cryptoboxDetector.isCryptoBoxDetected());
        telemetry.addData("isColumnDetected ",  cryptoboxDetector.isColumnDetected());

        telemetry.addData("Column Left ",  cryptoboxDetector.getCryptoBoxLeftPosition());
        telemetry.addData("Column Center ",  cryptoboxDetector.getCryptoBoxCenterPosition());
        telemetry.addData("Column Right ",  cryptoboxDetector.getCryptoBoxRightPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        cryptoboxDetector.disable();
    }
}
