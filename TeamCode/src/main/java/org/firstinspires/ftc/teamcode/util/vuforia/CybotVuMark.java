package org.firstinspires.ftc.teamcode.util.vuforia;

import android.graphics.Bitmap;
import android.graphics.Matrix;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.util.ClosableVuforiaLocalizer;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

/**
 * Created by kskrueger on 11/11/17.
 */

public class CybotVuMark {
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;

    ClosableVuforiaLocalizer vuforia;
    ///////////////////////////THIS IS THE REGULAR VUFORIA METHOD///////////////////////////
    //VuforiaLocalizer vuforia;

    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    public double tX;
    public double tY;
    public double tZ;

    // Extract the rotational components of the target relative to the robot
    public double rX;
    public double rY;
    public double rZ;

    public CybotVuMark(HardwareMap hardwareMap, VuforiaLocalizer.CameraDirection direction, boolean showCamera) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters;

        if (showCamera) {
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        } else {
            parameters = new VuforiaLocalizer.Parameters();
        }

        parameters.vuforiaLicenseKey = "AWXa2Uv/////AAAAGfurKeRqY0A1kSnac5nkp2JqA4O6hIqfI5aTjOVuQjPZo5eceByKm3Xz+vTurmsPQ7W9lS7qB1/CAPf04NarSGqMSvs+YE2Zf5xuqRcEvvTQe2RG8hk7J3jUnWs1ujcnTCezIboYM8OVMAb7rb7Xq1vid7DsKNlgX1ubpcE/DJ+0waUR3vTfU/tPuoeANaEld54egOAq8pLEpZ1MsYNWKhqiihzwqnbftT94r9dSMnoevFIzxtX7T02EQekUFO8Hu+RJgLkvpUE87tvygVUVcKH5Dtn1Or9lpYXtAbJ21oPoElpvWyabwOrUwjQXq9tJ+HII1uyBxJ8eefdXW8dgJ3efwG4Ydj2bXERZ4ri55nFe";

        parameters.cameraDirection = direction;

        vuforia = new ClosableVuforiaLocalizer(parameters);
        ///////////////////////////THIS IS THE REGULAR VUFORIA METHOD///////////////////////////
        //this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);

        this.relicTrackables = relicTrackables;
        this.relicTemplate = relicTemplate;
    }

    public void activate() {
        relicTrackables.activate();
    }

    public RelicRecoveryVuMark scan() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        return vuMark;
    }

    public void calculatePosition () {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                tX = trans.get(0);
                tY = trans.get(1);
                tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                rX = rot.firstAngle;
                rY = rot.secondAngle;
                rZ = rot.thirdAngle;
            }
        }
    }

    public Mat capture() {
        Mat mat = new Mat();
        VuforiaLocalizer.CloseableFrame frame = null;
        try {
            frame = vuforia.getFrameQueue().take();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        Image vuforiaFrame;
        long numImages = frame.getNumImages();
        for (int i = 0; i < numImages; i++)
        {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565)
            {
                vuforiaFrame = frame.getImage(i);
                Bitmap bitmap = Bitmap.createBitmap(vuforiaFrame.getWidth(), vuforiaFrame.getHeight(), Bitmap.Config.RGB_565);
                bitmap.copyPixelsFromBuffer(vuforiaFrame.getPixels());
                Bitmap bmp32 = bitmap.copy(Bitmap.Config.ARGB_8888, true);
                Utils.bitmapToMat(bmp32, mat);
            }
        }

        return mat;
    }

    public static Bitmap rotateBitmap(Bitmap source, float angle)
    {
        Matrix matrix = new Matrix();
        matrix.postRotate(angle);
        return Bitmap.createBitmap(source, 0, 0, source.getWidth(), source.getHeight(), matrix, true);
    }

    public void close() {
        vuforia.close();
    }

}