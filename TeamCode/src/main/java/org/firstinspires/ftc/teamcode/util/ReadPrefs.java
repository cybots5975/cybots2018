package org.firstinspires.ftc.teamcode.util;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by kskrueger for Cybots Robotics on 12/29/17.
 */

public class ReadPrefs {
    HardwareMap hardwareMap;

    public ReadPrefs(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public String read(String key) {
        SharedPreferences preferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        return preferences.getString(key, "");
    }
}
