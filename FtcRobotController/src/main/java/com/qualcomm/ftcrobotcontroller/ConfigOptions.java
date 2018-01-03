package com.qualcomm.ftcrobotcontroller;

import android.app.Activity;
import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.view.View;
import android.widget.Button;
import android.widget.Switch;
import android.widget.TextView;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

public class ConfigOptions extends Activity implements View.OnClickListener{
    private String position = "NOT SELECTED";
    private String color = "blank";
    private Boolean testMode;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_config_options);

        TextView selection = (TextView) findViewById(R.id.selectionView);
        selection.setText(position);

        Switch testModeSwitch = (Switch) findViewById(R.id.testModeSwitch);
        testModeSwitch.setChecked(readPrefs("testMode").equals("true"));

        Button save = (Button) findViewById(R.id.save_position);
        save.setOnClickListener(this);
        Button one = (Button) findViewById(R.id.red_close);
        one.setOnClickListener(this);
        Button two = (Button) findViewById(R.id.red_far);
        two.setOnClickListener(this);
        Button three = (Button) findViewById(R.id.blue_close);
        three.setOnClickListener(this);
        Button four = (Button) findViewById(R.id.blue_far);
        four.setOnClickListener(this);
    }

    @Override
    public void onClick(View v) {
        int i = v.getId();
        TextView selection = (TextView) findViewById(R.id.selectionView);

        if (i == R.id.save_position) {
            Switch testModeSwitch = (Switch) findViewById(R.id.testModeSwitch);
            testMode = testModeSwitch.isChecked();
            savePrefs("testMode",testMode.toString());
            savePrefs("position",position);
            savePrefs("color",color);
            selection.setText("Saved: "+readPrefs("position"));

            if (position!="NOT SELECTED") {
                Intent inspectionModeIntent = new Intent(AppUtil.getDefContext(), FtcRobotControllerActivity.class);
                startActivity(inspectionModeIntent);
            }

        } else if (i == R.id.red_close) {// do your code
            position = "RED CLOSE";
            color = "red";
            setColor(color);
            selection.setText(position);
        } else if (i == R.id.red_far) {// do your code
            position = "RED FAR";
            color = "red";
            setColor(color);
            selection.setText(position);
        } else if (i == R.id.blue_close) {// do your code
            position = "BLUE CLOSE";
            color = "blue";
            setColor(color);
            selection.setText(position);
        } else if (i == R.id.blue_far) {// do your code
            position = "BLUE FAR";
            color = "blue";
            setColor(color);
            selection.setText(position);
        } else {
            position = "NOT SELECTED";
            setColor(color);
            selection.setText("Select position below and save");
        }

    }

    public void savePrefs(String title, String data) {
        SharedPreferences preferences = PreferenceManager.getDefaultSharedPreferences(this);
        SharedPreferences.Editor editor = preferences.edit();
        editor.putString(title,data);
        editor.apply();
    }

    public String readPrefs(String title) {
        SharedPreferences preferences = PreferenceManager.getDefaultSharedPreferences(this);
        String name = preferences.getString(title, "");
        return name;
    }

    public void setColor(String color) {
        TextView selection = (TextView) findViewById(R.id.selectionView);
        if (color=="blue") {
            selection.setTextColor(getResources().getColor(R.color.bright_blue));
        } else if (color=="red") {
            selection.setTextColor(getResources().getColor(R.color.bright_red));
        } else {
            selection.setTextColor(getResources().getColor(R.color.bright_green));
        }
    }
}
