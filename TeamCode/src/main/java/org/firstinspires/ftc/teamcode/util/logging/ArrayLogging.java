package org.firstinspires.ftc.teamcode.logging;

import android.os.Environment;

import com.opencsv.CSVWriter;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;

/**
 * Created by kskrueger on 11/12/17.
 */

public class ArrayLogging {
    public String array[][];
    int xSize, ySize;

    public ArrayLogging(int xSize, int ySize) {
        this.array = new String[ySize][xSize];
        this.xSize = xSize;
        this.ySize = ySize;
    }

    public void storeValue (int xPos, int yPos, String value) {
        if (xPos<xSize&&yPos<ySize) {
            array[yPos][xPos] = value;
        }
    }

    public void storeValueInt (int xPos, int yPos, double value) {
        if (xPos<xSize&&yPos<ySize) {
            array[yPos][xPos] = String.valueOf(value);
        }
    }

    public String getValue (int xPos, int yPos) {
        return array[yPos][xPos];
    }

    public void log () throws IOException {
        save("swervelog2");
    }

    public void save(String fileName) throws IOException {
        String csv = Environment.getExternalStorageDirectory().getPath()+"/"+fileName+".csv";
        CSVWriter writer = new CSVWriter(new FileWriter(csv));
        writer.writeAll(Arrays.asList(array));
        writer.close();
    }
}
