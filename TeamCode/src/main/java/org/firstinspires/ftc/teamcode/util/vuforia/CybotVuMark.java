package org.firstinspires.ftc.teamcode.util.vuforia;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.util.ClosableVuforiaLocalizer;

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
    OpenGLMatrix pose;

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

    public void close() {
        vuforia.close();
    }

}