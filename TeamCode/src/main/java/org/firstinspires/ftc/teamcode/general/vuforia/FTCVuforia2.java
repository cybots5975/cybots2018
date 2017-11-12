package org.firstinspires.ftc.teamcode.general.vuforia;

import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

public class FTCVuforia2 {

    // Variables to be used for later
    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables visionTargets;
    private VuforiaTrackable target;
    private VuforiaTrackable target1;
    private VuforiaTrackable target2;
    private VuforiaTrackable target3;
    private VuforiaTrackableDefaultListener listener;
    private VuforiaTrackableDefaultListener listener1;
    private VuforiaTrackableDefaultListener listener2;
    private VuforiaTrackableDefaultListener listener3;

    private OpenGLMatrix lastKnownLocation;
    private OpenGLMatrix lastKnownLocation1;
    private OpenGLMatrix lastKnownLocation2;
    private OpenGLMatrix lastKnownLocation3;
    private OpenGLMatrix phoneLocation;

    private float robotX;
    private float robotY;
    private float robotAngle;

    public float xtest;

    private static final String VUFORIA_KEY = "AWXa2Uv/////AAAAGfurKeRqY0A1kSnac5nkp2JqA4O6hIqfI5aTjOVuQjPZo5eceByKm3Xz+vTurmsPQ7W9lS7qB1/CAPf04NarSGqMSvs+YE2Zf5xuqRcEvvTQe2RG8hk7J3jUnWs1ujcnTCezIboYM8OVMAb7rb7Xq1vid7DsKNlgX1ubpcE/DJ+0waUR3vTfU/tPuoeANaEld54egOAq8pLEpZ1MsYNWKhqiihzwqnbftT94r9dSMnoevFIzxtX7T02EQekUFO8Hu+RJgLkvpUE87tvygVUVcKH5Dtn1Or9lpYXtAbJ21oPoElpvWyabwOrUwjQXq9tJ+HII1uyBxJ8eefdXW8dgJ3efwG4Ydj2bXERZ4ri55nFe"; // Insert your own key here

    public void preOp() {
        setupVuforia();

        // We don't know where the robot is, so set it to the origin
        // If we don't include this, it would be null, which would cause errors later on
        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);
    }

    public void activate () {
        visionTargets.activate();
    }

    public void testThis() {
        int myVariable = 5;

        this.xtest = myVariable;
    }

    public double getX(double target) {
        if (target==0) {
            OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();
            if(latestLocation != null)
                lastKnownLocation = latestLocation;

            float[] coordinates = lastKnownLocation.getTranslation().getData();
            robotX = coordinates[0];
        } else if (target==1) {
            OpenGLMatrix latestLocation1 = listener1.getUpdatedRobotLocation();
            if(latestLocation1 != null)
                lastKnownLocation1 = latestLocation1;

            float[] coordinates1 = lastKnownLocation1.getTranslation().getData();
            robotX = coordinates1[0];
        } else if (target==2) {
            OpenGLMatrix latestLocation2 = listener2.getUpdatedRobotLocation();
            if(latestLocation2 != null)
                lastKnownLocation2 = latestLocation2;

            float[] coordinates2 = lastKnownLocation2.getTranslation().getData();
            robotX = coordinates2[0];
        } else {
            OpenGLMatrix latestLocation3 = listener3.getUpdatedRobotLocation();
            if(latestLocation3 != null)
                lastKnownLocation3 = latestLocation3;

            float[] coordinates3 = lastKnownLocation3.getTranslation().getData();
            robotX = coordinates3[0];
        }

        return robotX;
    }

    public double getY(double target) {
        if (target==0) {
            OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();
            if(latestLocation != null)
                lastKnownLocation = latestLocation;

            float[] coordinates = lastKnownLocation.getTranslation().getData();
            robotY = coordinates[1];
        } else if (target==1) {
            OpenGLMatrix latestLocation1 = listener1.getUpdatedRobotLocation();
            if(latestLocation1 != null)
                lastKnownLocation1 = latestLocation1;

            float[] coordinates1 = lastKnownLocation1.getTranslation().getData();
            robotY = coordinates1[1];
        } else if (target==2) {
            OpenGLMatrix latestLocation2 = listener2.getUpdatedRobotLocation();
            if(latestLocation2 != null)
                lastKnownLocation2 = latestLocation2;

            float[] coordinates2 = lastKnownLocation2.getTranslation().getData();
            robotY = coordinates2[1];
        } else {
            OpenGLMatrix latestLocation3 = listener3.getUpdatedRobotLocation();
            if(latestLocation3 != null)
                lastKnownLocation3 = latestLocation3;

            float[] coordinates3 = lastKnownLocation3.getTranslation().getData();
            robotY = coordinates3[1];
        }

        return robotY;
    }

    public double getAngle(double target) {
        if (target==0) {
            OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();
            if(latestLocation != null)
                lastKnownLocation = latestLocation;

            float[] coordinates = lastKnownLocation.getTranslation().getData();
            robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        } else if (target==1) {
            OpenGLMatrix latestLocation1 = listener1.getUpdatedRobotLocation();
            if(latestLocation1 != null)
                lastKnownLocation1 = latestLocation1;

            float[] coordinates1 = lastKnownLocation1.getTranslation().getData();
            robotAngle = Orientation.getOrientation(lastKnownLocation1, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        } else if (target==2) {
            OpenGLMatrix latestLocation2 = listener2.getUpdatedRobotLocation();
            if(latestLocation2 != null)
                lastKnownLocation2 = latestLocation2;

            float[] coordinates2 = lastKnownLocation2.getTranslation().getData();
            robotAngle = Orientation.getOrientation(lastKnownLocation2, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        } else {
            OpenGLMatrix latestLocation3 = listener3.getUpdatedRobotLocation();
            if(latestLocation3 != null)
                lastKnownLocation3 = latestLocation3;

            float[] coordinates3 = lastKnownLocation3.getTranslation().getData();
            robotAngle = Orientation.getOrientation(lastKnownLocation3, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        }

        return robotAngle;
    }

    // Creates a matrix for determining the locations and orientations of objects
    // Units are millimeters for x, y, and z, and degrees for u, v, and w
    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    // Formats a matrix into a readable string
    private String formatMatrix(OpenGLMatrix matrix)
    {
        return matrix.formatAsTransform();
    }


    private void setupVuforia() {
        // Setup parameters to create localizer
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId); // To remove the camera view from the screen, remove the R.id.cameraMonitorViewId
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("FTC_2016-17");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        // Setup the target to be tracked
        target = visionTargets.get(0); // 0 corresponds to the wheels target
        target.setName("Wheels Target");
        target.setLocation(createMatrix(0, 0, 0, 90, 0, 90));

        target = visionTargets.get(1); // 1 corresponds to the tools target
        target.setName("Tools Target");
        target.setLocation(createMatrix(0, 100, 0, 90, 0, 90));

        target = visionTargets.get(2); // 2 corresponds to the legos target
        target.setName("Legos Target");
        target.setLocation(createMatrix(0, 200, 0, 90, 0, 90));

        target = visionTargets.get(3); // 3 corresponds to the gears target
        target.setName("Gears Target");
        target.setLocation(createMatrix(0, 300, 0, 90, 0, 90));

        // Set phone location on robot
        phoneLocation = createMatrix(0, 0, 0, 90, 0, 0);

        // Setup listener and inform it of phone information

        listener = (VuforiaTrackableDefaultListener) target.getListener();
        listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
        listener1 = (VuforiaTrackableDefaultListener) target1.getListener();
        listener1.setPhoneInformation(phoneLocation, parameters.cameraDirection);
        listener2 = (VuforiaTrackableDefaultListener) target2.getListener();
        listener2.setPhoneInformation(phoneLocation, parameters.cameraDirection);
        listener3 = (VuforiaTrackableDefaultListener) target3.getListener();
        listener3.setPhoneInformation(phoneLocation, parameters.cameraDirection);

    }
}

/*public void getCoordinates (double target) {
        // Ask the listener for the latest information on where the robot is
        OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();
        OpenGLMatrix latestLocation1 = listener1.getUpdatedRobotLocation();
        OpenGLMatrix latestLocation2 = listener2.getUpdatedRobotLocation();
        OpenGLMatrix latestLocation3 = listener3.getUpdatedRobotLocation();

        // The listener will sometimes return null, so we check for that to prevent errors
        if(latestLocation != null)
            lastKnownLocation = latestLocation;

        if(latestLocation1 != null)
            lastKnownLocation1 = latestLocation1;

        if(latestLocation2 != null)
            lastKnownLocation2 = latestLocation2;

        if(latestLocation3 != null)
            lastKnownLocation3 = latestLocation3;

        float[] coordinates = lastKnownLocation.getTranslation().getData();
        float[] coordinates1 = lastKnownLocation1.getTranslation().getData();
        float[] coordinates2 = lastKnownLocation2.getTranslation().getData();
        float[] coordinates3 = lastKnownLocation3.getTranslation().getData();

        float robotX;
        float robotY;
        float robotAngle;

        if (target==0) {
            robotX = coordinates[0];
            robotY = coordinates[1];
            robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        } else if (target==1) {
            robotX = coordinates1[0];
            robotY = coordinates1[1];
            robotAngle = Orientation.getOrientation(lastKnownLocation1, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        } else if (target==2) {
            robotX = coordinates2[0];
            robotY = coordinates2[1];
            robotAngle = Orientation.getOrientation(lastKnownLocation2, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        } else if (target==3) {
            robotX = coordinates3[0];
            robotY = coordinates3[1];
            robotAngle = Orientation.getOrientation(lastKnownLocation3, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        } else {
            robotX = 0;
            robotY = 0;
            robotAngle = 0;
        }

        this.robotX = robotX;
        this.robotY = robotY;
        this.robotAngle = robotAngle;

        //return formatMatrix(lastKnownLocation);

        // Send information about whether the target is visible, and where the robot is
        //telemetry.addData("Tracking " + target.getName(), listener.isVisible());
        //telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));
    }

    public void target0() {
        OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();
        if(latestLocation != null)
            lastKnownLocation = latestLocation;

        float[] coordinates = lastKnownLocation.getTranslation().getData();
        robotX = coordinates[0];
        robotY = coordinates[1];
        robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
    }
    public void target1() {
        OpenGLMatrix latestLocation1 = listener1.getUpdatedRobotLocation();
        if(latestLocation1 != null)
            lastKnownLocation1 = latestLocation1;

        float[] coordinates1 = lastKnownLocation1.getTranslation().getData();
        robotX = coordinates1[0];
        robotY = coordinates1[1];
        robotAngle = Orientation.getOrientation(lastKnownLocation1, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
    }
    public void target2() {
        OpenGLMatrix latestLocation2 = listener2.getUpdatedRobotLocation();
        if(latestLocation2 != null)
            lastKnownLocation2 = latestLocation2;

        float[] coordinates2 = lastKnownLocation2.getTranslation().getData();
        robotX = coordinates2[0];
        robotY = coordinates2[1];
        robotAngle = Orientation.getOrientation(lastKnownLocation2, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
    }
    public void target3() {
        OpenGLMatrix latestLocation3 = listener3.getUpdatedRobotLocation();
        if(latestLocation3 != null)
            lastKnownLocation3 = latestLocation3;

        float[] coordinates3 = lastKnownLocation3.getTranslation().getData();
        robotX = coordinates3[0];
        robotY = coordinates3[1];
        robotAngle = Orientation.getOrientation(lastKnownLocation3, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
    }



    */