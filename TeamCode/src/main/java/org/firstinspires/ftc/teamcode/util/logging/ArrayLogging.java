package org.firstinspires.ftc.teamcode.util.logging;

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

    //initialize the array with your x and y size
    public ArrayLogging(int xSize, int ySize) {
        this.array = new String[ySize][xSize];
        this.xSize = xSize;
        this.ySize = ySize;
    }

    //use this to store a String value/variable to the array
    public void storeValue (int xPos, int yPos, String value) {
        if (xPos<xSize&&yPos<ySize) {
            array[yPos][xPos] = value;
        }
    }

    //use this to store an int to the array
    public void storeValueInt (int xPos, int yPos, double value) {
        if (xPos<xSize&&yPos<ySize) {
            array[yPos][xPos] = String.valueOf(value);
        }
    }

    //use this to retrieve the value from a position in the array
    public String getValue (int xPos, int yPos) {
        return array[yPos][xPos];
    }

    //this will save the array to a CSV file with the entered file name
    //the saved CSV will be located in the /sdCard folder on the phone
    public void log (String name) {
        try {
            saveFile(name);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void save (String name) {
        try {
            saveFile(name);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void saveFile(String fileName) throws IOException {
        String csv = Environment.getExternalStorageDirectory().getPath()+"/"+fileName+".csv";
        CSVWriter writer = new CSVWriter(new FileWriter(csv));
        writer.writeAll(Arrays.asList(array));
        writer.close();
    }
}
