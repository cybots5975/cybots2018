package org.firstinspires.ftc.teamcode;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * Created by guinea on 10/5/17.
 * A nice demo class for using OpenCVPipeline. This one also demonstrates how to use OpenCV to threshold
 * for a certain color (blue), which is very common in robotics OpenCV applications.
 */

public class ExampleBlueVision extends OpenCVPipeline {
    private boolean showBlue = true;
    // To keep it such that we don't have to instantiate a new Mat every call to processFrame,
    // we declare the Mats up here and reuse them. This is easier on the garbage collector.
    private Mat hsv = new Mat();
    private Mat thresholded = new Mat();
    private Mat thresholded_rgba = new Mat();


    public void setShowBlue(boolean enabled) {
        showBlue = enabled;
    }

    // This is called every camera frame.
    @Override
    public Mat processFrame(Mat rgba, Mat gray) {
        // First, we change the colorspace from RGBA to HSV, which is usually better for color
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV, 3);
        // Then, we threshold our hsv image so that we get a black/white binary image where white
        // is the blues listed in the specified range
        Core.inRange(hsv, new Scalar(90, 128, 30), new Scalar(170, 255, 255), thresholded);
        // Then we display our nice little binary threshold on screen
        if (showBlue) {
            // since the thresholded image data is a black and white image, we have to convert it back to rgba
            Imgproc.cvtColor(thresholded, thresholded_rgba, Imgproc.COLOR_GRAY2RGBA);
            return thresholded_rgba;
        } else {
            // if we aren't displaying the binary image, just show the original frame onscreen.
            return rgba;
        }
    }
}
