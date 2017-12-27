package org.firstinspires.ftc.teamcode.util.vuforia;

import android.graphics.Bitmap;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

/**
 * Created by kskrueger on 12/1/17.
 */

public class VuforiaToCVtest {
    Image rgb;

    public Image convert (VuforiaLocalizer vuforia) {
        VuforiaLocalizer.CloseableFrame frame = null; //takes the frame at the head of the queue
        try {
            frame = vuforia.getFrameQueue().take();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        long numImages = frame.getNumImages();

        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;
            }
        }

    /*rgb is now the Image object that we've used in the video*/
        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());

        //put the image into a MAT for OpenCV
        Mat tmp = new Mat(rgb.getWidth(), rgb.getHeight(), CvType.CV_8UC4);
        Utils.bitmapToMat(bm, tmp);

//close the frame, prevents memory leaks and crashing
        frame.close();

        return rgb;
    }
}