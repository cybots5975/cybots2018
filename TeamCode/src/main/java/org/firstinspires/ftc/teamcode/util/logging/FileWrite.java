package org.firstinspires.ftc.teamcode.util.logging;

import android.os.Environment;

import com.opencsv.CSVWriter;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;

/**
 * Created by kskrueger on 11/12/17.
 */

public class FileWrite {

    String[][] test = new String[1][2];

    public void logArray () throws IOException {
        save(test,"swerveLog");
    }

    public void save(String[][] array, String fileName) throws IOException {
        String csv = Environment.getExternalStorageDirectory().getPath()+"/"+fileName+".csv";
        CSVWriter writer = new CSVWriter(new FileWriter(csv));
        writer.writeAll(Arrays.asList(array));
        writer.close();
    }
}