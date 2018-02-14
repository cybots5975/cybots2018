package org.firstinspires.ftc.teamcode.test.monkeyCopy;

import android.os.Environment;

import com.opencsv.CSVReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.logging.ArrayLogging;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.List;

@TeleOp(name="Monkey Do Testing", group="Monkey Copy")
//@Disabled
public class MonkeyDoTesting extends LinearOpMode {
    //private Robot robot = new Robot(this);
    private ArrayLogging log = new ArrayLogging(16,10000);
    private int count = 0;
    public ElapsedTime runtime = new ElapsedTime();
    private double[] position = new double[4];
    private double[] velocity = new double[4];

    double previousMilli = 0;

    private DcMotorEx Motor;

    File read = new File(Environment.getExternalStorageDirectory() + "/monkeySee.csv");

    @Override
    public void runOpMode() {
        //robot.init();
        Motor = (DcMotorEx) hardwareMap.dcMotor.get("Motor");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        try {
            String[][] readArray = readCSV(read);
        } catch (IOException e) {
            e.printStackTrace();
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //position[0] = Motor.getCurrentPosition();

            //velocity[0] = Motor.getVelocity(AngleUnit.DEGREES);




            while(runtime.milliseconds()-previousMilli < 20) {
                //wait
            }
            previousMilli = runtime.milliseconds();

            telemetry.update();
        }
    }

    public void initializeLogging() {
        log.storeValue(0, 0, "Count #");
        log.storeValue(1, 0, "Time");
        log.storeValue(2, 0, "LeftY Joystick");
        log.storeValue(3, 0, "LeftX Joystick");
        log.storeValue(4, 0, "RightX Joystick");

        log.storeValue(5, 0, "Gyro Heading");
        log.storeValue(6, 0, "Forward Encoder Counts");
        log.storeValue(7, 0, "Strafe Encoder Counts");

        log.storeValue(8, 0, "D1 Position");
        log.storeValue(9, 0, "D1 Velocity");

        log.storeValue(10, 0, "D2 Position");
        log.storeValue(11, 0, "D2 Velocity");

        log.storeValue(12, 0, "P1 Position");
        log.storeValue(13, 0, "P1 Velocity");

        log.storeValue(14, 0, "P1 Position");
        log.storeValue(15, 0, "P2 Velocity");
    }

    private static String[][] readCSV (File file) throws IOException
    {
        CSVReader csvReader = new CSVReader(new FileReader(file));
        List<String[]> list = csvReader.readAll();
        String[][] dataArr = new String[list.size()][];
        dataArr = list.toArray(dataArr);
        return dataArr;
    }

    private static void arrayCopy(String[][] aSource, String[][] aDestination)
    {
        for (int i = 0; i < aSource.length; i++) {
            System.arraycopy(aSource[i], 0, aDestination[i], 0, aSource[i].length);
        }
    }

}