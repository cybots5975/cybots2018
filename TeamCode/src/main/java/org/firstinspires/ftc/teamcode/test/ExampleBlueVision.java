package org.firstinspires.ftc.teamcode.test;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.firstinspires.ftc.teamcode.general.vuforia.HSVfilters;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

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

    public enum order{
        blueFirst,
        redFirst;
    }

    public static order jewelsOrder;

    public void setShowBlue(boolean enabled) {
        showBlue = enabled;
    }

    // This is called every camera frame.
    @Override
    public Mat processFrame(Mat frame) {
        // First, we change the colorspace from RGBA to HSV, which is usually better for color
        //Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV, 3);
        // Then, we threshold our hsv image so that we get a black/white binary image where white
        // is the blues listed in the specified range
        //Core.inRange(hsv, new Scalar(90, 128, 30), new Scalar(170, 255, 255), thresholded);
        // Then we display our nice little binary threshold on screen

        /*if (showBlue) {


            // since the thresholded image data is a black and white image, we have to convert it back to rgba
            Imgproc.cvtColor(thresholded, thresholded_rgba, Imgproc.COLOR_GRAY2RGBA);
            return thresholded_rgba;
        } else {
            // if we aren't displaying the binary image, just show the original frame onscreen.
            return rgba;
        }*/

        //Mat frameb = new Mat();

        //Core.rotate(frame,frame,Core.ROTATE_90_COUNTERCLOCKWISE);

        //Core.transpose(frame,frame);

        Mat output = new Mat();

        //process(frame);

        output = processFrame2(frame);

        return output;
    }

    /*private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
                              Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
        Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                new Scalar(hue[1], sat[1], val[1]), out);
    }*/

    //Outputs
//Outputs
    private Mat blurOutput = new Mat();
    private Mat hsvThresholdOutput = new Mat();
    private Mat cvDilateOutput = new Mat();
    private Mat cvErodeOutput = new Mat();

    /**
     * This is the primary method that runs the entire pipeline and updates the outputs.
     */
    public void process(Mat source0) {
        // Step Blur0:
        Mat blurInput = source0;
        BlurType blurType = BlurType.get("Box Blur");
        double blurRadius = 0.0;
        blur(blurInput, blurType, blurRadius, blurOutput);

        // Step HSV_Threshold0:
        Mat hsvThresholdInput = blurOutput;
        double[] hsvThresholdHue = {89.02877697841726, 131.10356536502547};
        double[] hsvThresholdSaturation = {167.4010791366906, 255.0};
        double[] hsvThresholdValue = {11.465827338129495, 239.84719864176571};
        hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

        // Step CV_dilate0:
        Mat cvDilateSrc = hsvThresholdOutput;
        Mat cvDilateKernel = new Mat();
        Point cvDilateAnchor = new Point(-1, -1);
        double cvDilateIterations = 8.0;
        int cvDilateBordertype = Core.BORDER_CONSTANT;
        Scalar cvDilateBordervalue = new Scalar(-1);
        cvDilate(cvDilateSrc, cvDilateKernel, cvDilateAnchor, cvDilateIterations, cvDilateBordertype, cvDilateBordervalue, cvDilateOutput);

        // Step CV_erode0:
        Mat cvErodeSrc = cvDilateOutput;
        Mat cvErodeKernel = new Mat();
        Point cvErodeAnchor = new Point(-1, -1);
        double cvErodeIterations = 2.0;
        int cvErodeBordertype = Core.BORDER_CONSTANT;
        Scalar cvErodeBordervalue = new Scalar(-1);
        cvErode(cvErodeSrc, cvErodeKernel, cvErodeAnchor, cvErodeIterations, cvErodeBordertype, cvErodeBordervalue, cvErodeOutput);

    }

    /**
     * This method is a generated getter for the output of a Blur.
     * @return Mat output from Blur.
     */
    public Mat blurOutput() {
        return blurOutput;
    }

    /**
     * This method is a generated getter for the output of a HSV_Threshold.
     * @return Mat output from HSV_Threshold.
     */
    public Mat hsvThresholdOutput() {
        return hsvThresholdOutput;
    }

    /**
     * This method is a generated getter for the output of a CV_dilate.
     * @return Mat output from CV_dilate.
     */
    public Mat cvDilateOutput() {
        return cvDilateOutput;
    }

    /**
     * This method is a generated getter for the output of a CV_erode.
     * @return Mat output from CV_erode.
     */
    public Mat cvErodeOutput() {
        return cvErodeOutput;
    }


    /**
     * An indication of which type of filter to use for a blur.
     * Choices are BOX, GAUSSIAN, MEDIAN, and BILATERAL
     */
    enum BlurType{
        BOX("Box Blur"), GAUSSIAN("Gaussian Blur"), MEDIAN("Median Filter"),
        BILATERAL("Bilateral Filter");

        private final String label;

        BlurType(String label) {
            this.label = label;
        }

        public static BlurType get(String type) {
            if (BILATERAL.label.equals(type)) {
                return BILATERAL;
            }
            else if (GAUSSIAN.label.equals(type)) {
                return GAUSSIAN;
            }
            else if (MEDIAN.label.equals(type)) {
                return MEDIAN;
            }
            else {
                return BOX;
            }
        }

        @Override
        public String toString() {
            return this.label;
        }
    }

    /**
     * Softens an image using one of several filters.
     * @param input The image on which to perform the blur.
     * @param type The blurType to perform.
     * @param doubleRadius The radius for the blur.
     * @param output The image in which to store the output.
     */
    private void blur(Mat input, BlurType type, double doubleRadius,
                      Mat output) {
        int radius = (int)(doubleRadius + 0.5);
        int kernelSize;
        switch(type){
            case BOX:
                kernelSize = 2 * radius + 1;
                Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
                break;
            case GAUSSIAN:
                kernelSize = 6 * radius + 1;
                Imgproc.GaussianBlur(input,output, new Size(kernelSize, kernelSize), radius);
                break;
            case MEDIAN:
                kernelSize = 2 * radius + 1;
                Imgproc.medianBlur(input, output, kernelSize);
                break;
            case BILATERAL:
                Imgproc.bilateralFilter(input, output, -1, radius, radius);
                break;
        }
    }

    /**
     * Segment an image based on hue, saturation, and value ranges.
     *
     * @param input The image on which to perform the HSL threshold.
     * @param hue The min and max hue
     * @param sat The min and max saturation
     * @param val The min and max value
    //* @param output The image in which to store the output.
     */
    private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
                              Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
        Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                new Scalar(hue[1], sat[1], val[1]), out);
    }

    /**
     * Expands area of higher value in an image.
     * @param src the Image to dilate.
     * @param kernel the kernel for dilation.
     * @param anchor the center of the kernel.
     * @param iterations the number of times to perform the dilation.
     * @param borderType pixel extrapolation method.
     * @param borderValue value to be used for a constant border.
     * @param dst Output Image.
     */
    private void cvDilate(Mat src, Mat kernel, Point anchor, double iterations,
                          int borderType, Scalar borderValue, Mat dst) {
        if (kernel == null) {
            kernel = new Mat();
        }
        if (anchor == null) {
            anchor = new Point(-1,-1);
        }
        if (borderValue == null){
            borderValue = new Scalar(-1);
        }
        Imgproc.dilate(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
    }

    /**
     * Expands area of lower value in an image.
     * @param src the Image to erode.
     * @param kernel the kernel for erosion.
     * @param anchor the center of the kernel.
     * @param iterations the number of times to perform the erosion.
     * @param borderType pixel extrapolation method.
     * @param borderValue value to be used for a constant border.
     * @param dst Output Image.
     */
    private void cvErode(Mat src, Mat kernel, Point anchor, double iterations,
                         int borderType, Scalar borderValue, Mat dst) {
        if (kernel == null) {
            kernel = new Mat();
        }
        if (anchor == null) {
            anchor = new Point(-1,-1);
        }
        if (borderValue == null) {
            borderValue = new Scalar(-1);
        }
        Imgproc.erode(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
    }

    public String order = "";

    public Mat processFrame2(Mat frame)
    {
        // if the frame is not empty, process it
        if (!frame.empty())
        {
            // init
            //Mat flippedImage = new Mat();
            Mat resizedImage = new Mat();
            Mat blurredImage = new Mat();
            Mat hsvImage = new Mat();
            Mat maskRedLower = new Mat();
            Mat maskRedUpper = new Mat();
            Mat maskRedCombined = new Mat();
            Mat morphOutputRed = new Mat();
            Mat maskBlue = new Mat();
            Mat morphOutputBlue = new Mat();

            //Core.flip(frame, flippedImage, 1);

            //resize to lower resolution
            //Imgproc.resize(frame,resizedImage,new Size(640,480),0,0,Imgproc.INTER_CUBIC);

            // remove some noise
            Imgproc.blur(frame, blurredImage, new Size(7, 7));

            // convert the frame to HSV
            Imgproc.cvtColor(blurredImage, hsvImage, Imgproc.COLOR_RGB2HSV);

            // get thresholding values from the UI
            // remember: H ranges 0-180, S and V range 0-255
            Scalar minValuesRedUpper = new Scalar(HSVfilters.HueStartUpper_red, HSVfilters.SaturationStart_red, HSVfilters.ValueStart_red);
            Scalar maxValuesRedUpper = new Scalar(HSVfilters.HueStopUpper_red, HSVfilters.SaturationStop_red, HSVfilters.ValueStop_red);
            Scalar minValuesRedLower = new Scalar(HSVfilters.HueStartLower_red, HSVfilters.SaturationStart_red, HSVfilters.ValueStart_red);
            Scalar maxValuesRedLower = new Scalar(HSVfilters.HueStopLower_red, HSVfilters.SaturationStop_red, HSVfilters.ValueStop_red);
            Scalar minValuesBlue = new Scalar(HSVfilters.HueStart_blue, HSVfilters.SaturationStart_blue, HSVfilters.ValueStart_blue);
            Scalar maxValuesBlue = new Scalar(HSVfilters.HueStop_blue, HSVfilters.SaturationStop_blue, HSVfilters.ValueStop_blue);

            //threshold HSV image to select red ball
            Core.inRange(hsvImage, minValuesRedLower, maxValuesRedLower, maskRedLower);
            Core.inRange(hsvImage, minValuesRedUpper, maxValuesRedUpper, maskRedUpper);

            //Combine the two red masks
            Core.addWeighted(maskRedUpper, 1.0, maskRedLower, 1.0, 0.0, maskRedCombined);

            //threshold HSV image to select red ball
            Core.inRange(hsvImage, minValuesBlue, maxValuesBlue, maskBlue);

            // morphological operators
            // dilate with large element, erode with small ones
            Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(24, 24));
            Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(12, 12));

            Imgproc.erode(maskRedCombined, morphOutputRed, erodeElement);
            Imgproc.erode(morphOutputRed, morphOutputRed, erodeElement);

            Imgproc.dilate(morphOutputRed, morphOutputRed, dilateElement);
            Imgproc.dilate(morphOutputRed, morphOutputRed, dilateElement);

            Imgproc.erode(maskBlue, morphOutputBlue, erodeElement);
            Imgproc.erode(morphOutputBlue, morphOutputBlue, erodeElement);

            Imgproc.dilate(morphOutputBlue, morphOutputBlue, dilateElement);
            Imgproc.dilate(morphOutputBlue, morphOutputBlue, dilateElement);

            // find the jewels contours and show them
            frame = this.findAndDrawBalls(morphOutputRed, morphOutputBlue, frame);

        }


        return frame;
    }

    public Mat findAndDrawBalls(Mat maskedImageRed, Mat maskedImageBlue, Mat frame)
    {
        //init
        List<MatOfPoint> contoursRed = new ArrayList<>();
        List<MatOfPoint> contoursBlue = new ArrayList<>();
        Mat hierarchyRed = new Mat();
        Mat hierarchyBlue = new Mat();
        Rect blueRect = null;
        Rect redRect = null;
        boolean MTOROSC = false;

        // find contours
        Imgproc.findContours(maskedImageRed, contoursRed, hierarchyRed, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(maskedImageBlue, contoursBlue, hierarchyBlue, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

        //filter(contoursRed,contoursRed);

        //if any contours exist...
        if (hierarchyRed.size().height > 0 && hierarchyRed.size().width > 0)
        {
            // for each contour, display it in blue
            for (int idx = 0; idx >= 0; idx = (int) hierarchyRed.get(0, idx)[0])
            {
                Imgproc.drawContours(frame, contoursRed, idx, new Scalar(255, 0, 0));
            }

            MatOfPoint2f approxCurve = new MatOfPoint2f();

            //For each contour found
            for (int i = 0; i < contoursRed.size(); i++)
            {
                if (i >= 1)
                {
                    MTOROSC = true;
                }

                //Convert contoursRed(i) from MatOfPoint to MatOfPoint2f
                MatOfPoint2f contour2f = new MatOfPoint2f(contoursRed.get(i).toArray());

                //Processing on mMOP2f1 which is in type MatOfPoint2f
                double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;
                Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

                //Convert back to MatOfPoint
                MatOfPoint points = new MatOfPoint(approxCurve.toArray());

                // Get bounding rect of contour
                Rect rect = Imgproc.boundingRect(points);
                redRect = rect;

                //System.out.println("X: " + rect.x);
                //System.out.println("Y: " + rect.y);

                // draw enclosing rectangle (all same color, but you could use variable i to make them unique)
                Imgproc.rectangle(frame, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new Scalar(0, 255, 0, 255), 2);
                Imgproc.putText(frame, "R", new Point(rect.x - 5, rect.y - 10), Core.FONT_HERSHEY_PLAIN, 2, new Scalar(0, 255, 0), 2);
            }
        } else {
            redRect = null;
        }

        //if any contours exist...
        if (hierarchyBlue.size().height > 0 && hierarchyBlue.size().width > 0)
        {
            // for each contour, display it in blue
            for (int idx = 0; idx >= 0; idx = (int) hierarchyBlue.get(0, idx)[0])
            {
                Imgproc.drawContours(frame, contoursBlue, idx, new Scalar(0, 0, 255));
            }

            MatOfPoint2f approxCurve = new MatOfPoint2f();

            //For each contour found
            for (int i = 0; i < contoursBlue.size(); i++)
            {
                if (i >= 1)
                {
                    MTOROSC = true;
                }

                //Convert contoursRed(i) from MatOfPoint to MatOfPoint2f
                MatOfPoint2f contour2f = new MatOfPoint2f(contoursBlue.get(i).toArray());
                //Processing on mMOP2f1 which is in type MatOfPoint2f
                double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;
                Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

                //Convert back to MatOfPoint
                MatOfPoint points = new MatOfPoint(approxCurve.toArray());

                // Get bounding rect of contour
                Rect rect = Imgproc.boundingRect(points);
                blueRect = rect;

                //System.out.println("X: " + rect.x);
                //System.out.println("Y: " + rect.y);

                // draw enclosing rectangle (all same color, but you could use variable i to make them unique)
                Imgproc.rectangle(frame, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new Scalar(0, 255, 0, 255), 2);
                Imgproc.putText(frame, "B", new Point(rect.x - 5, rect.y - 10), Core.FONT_HERSHEY_PLAIN, 2, new Scalar(0, 255, 0), 2);
            }
        } else
        {
            blueRect = null;
        }

        if ((blueRect != null) && (redRect != null))
        {
            if (!MTOROSC)
            {
                long confidence;

                if (blueRect.y < redRect.y)
                {
                    confidence = Math.round(((blueRect.y - redRect.y) * .66) + 100);
                } else
                {
                    confidence = Math.round(((redRect.y - blueRect.y) * .66) + 100);
                }

                if (confidence > 100)
                {
                    confidence = 100;
                } else if (confidence < 0)
                {
                    confidence = 0;
                }

                if (blueRect.y > redRect.y)
                {
                    Imgproc.putText(frame, "Order: red,blue", new Point(5, 25), Core.FONT_HERSHEY_PLAIN, 2, new Scalar(0, 255, 0), 2);
                    order = "Red,Blue";
                    jewelsOrder = jewelsOrder.redFirst;
                } else
                {
                    Imgproc.putText(frame, "Order: blue,red", new Point(5, 25), Core.FONT_HERSHEY_PLAIN, 2, new Scalar(0, 255, 0), 2);
                    order = "Blue,Red";
                    jewelsOrder = jewelsOrder.blueFirst;

                }

                Imgproc.putText(frame, "Confidence: " + confidence + "%", new Point(5, 50), Core.FONT_HERSHEY_PLAIN, 2, new Scalar(0, 255, 0), 2);
            } else
            {
                Imgproc.putText(frame, "Err - MTOROSC", new Point(5, 25), Core.FONT_HERSHEY_PLAIN, 2, new Scalar(0, 255, 0), 2);
            }
        } else
        {
            if (!MTOROSC)
            {
                Imgproc.putText(frame, "Order: ???", new Point(5, 25), Core.FONT_HERSHEY_PLAIN, 2, new Scalar(0, 255, 0), 2);
            } else
            {
                Imgproc.putText(frame, "Err - MTOROSC", new Point(5, 25), Core.FONT_HERSHEY_PLAIN, 2, new Scalar(0, 255, 0), 2);
            }
        }

        MTOROSC = false;

        //Imgproc.putText(frame, "FPS: " + Math.round(1000 / lastTimeMills), new Point(5, 75), Core.FONT_HERSHEY_PLAIN, 2, new Scalar(0, 255, 0), 2);

        return frame;
    }
}
