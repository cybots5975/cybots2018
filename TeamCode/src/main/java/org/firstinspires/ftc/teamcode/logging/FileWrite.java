package org.firstinspires.ftc.teamcode.logging;

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
        save("swerveLog");
    }

    public void save(String fileName) throws IOException {
        String csv = android.os.Environment.getExternalStorageDirectory().getAbsolutePath();
        CSVWriter writer = new CSVWriter(new FileWriter(csv));

        /*List<String[]> data = new ArrayList<String[]>();
        data.add(new String[] {"India", "New Delhi"});
        data.add(new String[] {"United States", "Washington D.C"});
        data.add(new String[] {"Germany", "Berlin"});
        writer.writeAll(data);
        */


        writer.writeAll(Arrays.asList(test));

        writer.close();
    }

}