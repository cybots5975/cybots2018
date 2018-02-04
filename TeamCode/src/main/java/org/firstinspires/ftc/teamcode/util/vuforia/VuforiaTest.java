package org.firstinspires.ftc.teamcode.util.vuforia;
import android.graphics.Bitmap;
import android.graphics.Matrix;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
@Autonomous(name = "Concept: VuMark Id", group = "Concept")
public class VuforiaTest extends LinearOpMode
{
    VuforiaLocalizer vuforia;
    int frameNum = 0;
    @Override
    public void runOpMode() throws InterruptedException
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AWXa2Uv/////AAAAGfurKeRqY0A1kSnac5nkp2JqA4O6hIqfI5aTjOVuQjPZo5eceByKm3Xz+vTurmsPQ7W9lS7qB1/CAPf04NarSGqMSvs+YE2Zf5xuqRcEvvTQe2RG8hk7J3jUnWs1ujcnTCezIboYM8OVMAb7rb7Xq1vid7DsKNlgX1ubpcE/DJ+0waUR3vTfU/tPuoeANaEld54egOAq8pLEpZ1MsYNWKhqiihzwqnbftT94r9dSMnoevFIzxtX7T02EQekUFO8Hu+RJgLkvpUE87tvygVUVcKH5Dtn1Or9lpYXtAbJ21oPoElpvWyabwOrUwjQXq9tJ+HII1uyBxJ8eefdXW8dgJ3efwG4Ydj2bXERZ4ri55nFe";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();
        relicTrackables.activate();
        while (opModeIsActive())
        {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN)
            {
                telemetry.addData("VuMark", "%s visible", vuMark);
            } else
            {
                telemetry.addData("VuMark", "not visible");
            }
            telemetry.update();
            VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
            Image vuforiaFrame;
            long numImages = frame.getNumImages();
            for (int i = 0; i < numImages; i++)
            {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565)
                {
                    frameNum++;
                    vuforiaFrame = frame.getImage(i);
                    Bitmap bitmap = Bitmap.createBitmap(vuforiaFrame.getWidth(), vuforiaFrame.getHeight(), Bitmap.Config.RGB_565);
                    bitmap.copyPixelsFromBuffer(vuforiaFrame.getPixels());
                }
            }
        }
    }
    public static Bitmap rotateBitmap(Bitmap source, float angle)
    {
        Matrix matrix = new Matrix();
        matrix.postRotate(angle);
        return Bitmap.createBitmap(source, 0, 0, source.getWidth(), source.getHeight(), matrix, true);
    }
}