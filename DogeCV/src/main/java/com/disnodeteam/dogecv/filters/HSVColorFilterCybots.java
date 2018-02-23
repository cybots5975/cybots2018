package com.disnodeteam.dogecv.filters;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;


public class HSVColorFilterCybots extends DogeCVColorFilter {

    private Scalar high = new Scalar(255, 255, 255);
    private Scalar low = new Scalar(0, 0, 0);

    public HSVColorFilterCybots(Scalar low, Scalar high) {
        low = this.low;
        high = this.high;
    }

    @Override
    public void process(Mat input, Mat mask) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV_FULL);
        Imgproc.GaussianBlur(input, input, new Size(3, 3), 0);

        //Scalar lower = new Scalar(perfect.val[0] - range.val[0], perfect.val[1] - range.val[1],perfect.val[2] - range.val[2]);
        //Scalar upper = new Scalar(perfect.val[0] + range.val[0], perfect.val[1] + range.val[1],perfect.val[2] + range.val[2]);
        Scalar lower = new Scalar(low.val[0], low.val[1], low.val[2]);
        Scalar upper = new Scalar(high.val[0], high.val[1], high.val[2]);
        Core.inRange(input, lower, upper, mask);
        input.release();
    }
}