package org.firstinspires.ftc.teamcode.util;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;

/**
 * Created by kskrueger for Cybots Robotics on 12/29/17.
 */

@TeleOp(name = "Sample Import", group = "testing")
@Disabled
public class ReadMatchConfigSettings extends LinearOpMode {

    private String filename = Environment.getExternalStorageDirectory() + "/FIRST/menuConfig.txt";

    private ReadPrefs prefs;

    public static final int MatchDescription = 0;
    public String MyMatchDescription;
    public static final int TeamColor = 1;
    public String MyTeamColor;
    private String textOut;

    @Override
    public void runOpMode() {
         prefs = new ReadPrefs(hardwareMap);

         textOut = prefs.read("position");

        getMatchConfig();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Saved:",textOut);
            telemetry.addData("Mode:",prefs.read("testMode"));
            telemetry.update();
        }
    }

    public void getSettings() {
        //Find the directory for the SD Card using the API
//*Don't* hardcode "/sdcard"
        File sdcard = Environment.getExternalStorageDirectory();

//Get the text file
        File file = new File(sdcard+"/FIRST/","menuConfig.txt");

//Read text from file
        StringBuilder text = new StringBuilder();

        try {
            BufferedReader br = new BufferedReader(new FileReader(file));
            String line;

            while ((line = br.readLine()) != null) {
                text.append(line);
                text.append('\n');
            }
            br.close();
        }
        catch (IOException e) {
            //You'll need to add proper error handling here
        }

        textOut = text.toString();
    }

    public void getMatchConfig() {
        try {
            FileInputStream fis = new FileInputStream(filename);
            InputStreamReader isr = new InputStreamReader(fis);
            BufferedReader bufferedReader = new BufferedReader(isr);
            StringBuilder sb = new StringBuilder();
            String line;
            int index = -1;
            while ((line = bufferedReader.readLine()) != null)
            {
                line = line.trim(); switch(++index)
                {
                    case MatchDescription:
                        MyMatchDescription = line;
                        break;
                    case TeamColor:
                        MyTeamColor = line;
                        break;
                }
            }
        } catch (IOException ignored) {
        }
    }
}
