package org.firstinspires.ftc.teamcode.matchCode.Teleop;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Objects;

@TeleOp(name="Change Autonomous Mode", group="General")
//@Disabled
public class AutonChanger extends LinearOpMode {

    String position = "";
    String color;

    @Override
    public void runOpMode() {

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Select a position with a,b,x,y to set the mode","");
            telemetry.addData("Selected Mode", readPrefs("position"));

            if (gamepad1.a) {
                position = "RED CLOSE";
                color = "red";
            } else if (gamepad1.b) {
                position = "RED FAR";
                color = "red";
            } else if (gamepad1.x) {
                position = "BLUE CLOSE";
                color = "blue";
            } else if (gamepad1.y) {
                position = "BLUE FAR";
                color = "blue";
            }

            if (!Objects.equals(position,"")) {
                savePrefs("position",position);
                savePrefs("color",color);
            }

            telemetry.update();
        }
    }

    private void savePrefs(String title, String data) {
        SharedPreferences preferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        SharedPreferences.Editor editor = preferences.edit();
        editor.putString(title,data);
        editor.apply();
    }

    private String readPrefs(String title) {
        SharedPreferences preferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        String name = preferences.getString(title, "");
        return name;
    }
}
