/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.matchCode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.general.ClosableVuforiaLocalizer;
import org.firstinspires.ftc.teamcode.general.vuforia.HSVfilters;
import org.firstinspires.ftc.teamcode.test.ExampleBlueVision;
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

@TeleOp(name="Test CV 1", group="TESTING CV")
//@Disabled
public class TeleopTemplateCV1 extends LinearOpMode {
    private ClosableVuforiaLocalizer vuforia;
    ExampleBlueVision blueVision;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters;

        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AWXa2Uv/////AAAAGfurKeRqY0A1kSnac5nkp2JqA4O6hIqfI5aTjOVuQjPZo5eceByKm3Xz+vTurmsPQ7W9lS7qB1/CAPf04NarSGqMSvs+YE2Zf5xuqRcEvvTQe2RG8hk7J3jUnWs1ujcnTCezIboYM8OVMAb7rb7Xq1vid7DsKNlgX1ubpcE/DJ+0waUR3vTfU/tPuoeANaEld54egOAq8pLEpZ1MsYNWKhqiihzwqnbftT94r9dSMnoevFIzxtX7T02EQekUFO8Hu+RJgLkvpUE87tvygVUVcKH5Dtn1Or9lpYXtAbJ21oPoElpvWyabwOrUwjQXq9tJ+HII1uyBxJ8eefdXW8dgJ3efwG4Ydj2bXERZ4ri55nFe";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = new ClosableVuforiaLocalizer(parameters);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        vuforia.close();

        blueVision = new ExampleBlueVision();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        blueVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        blueVision.setShowBlue(false);
        // start the vision system
        blueVision.enable();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            blueVision.setShowBlue(true);

            telemetry.addData("Order",blueVision.order);
            telemetry.update();

            if (isStopRequested()) {
                blueVision.setShowBlue(false);
                blueVision.disable();
            }
        }

    }

    public String order = "";

    public Mat processFrame(Mat frame)
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

                if (blueRect.x > redRect.x)
                {
                    Imgproc.putText(frame, "Order: red,blue", new Point(5, 25), Core.FONT_HERSHEY_PLAIN, 2, new Scalar(0, 255, 0), 2);
                    order = "Red,Blue";
                } else
                {
                    Imgproc.putText(frame, "Order: blue,red", new Point(5, 25), Core.FONT_HERSHEY_PLAIN, 2, new Scalar(0, 255, 0), 2);
                    order = "Blue,Red";
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
