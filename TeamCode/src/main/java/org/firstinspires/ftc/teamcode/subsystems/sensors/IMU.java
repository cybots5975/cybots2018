package org.firstinspires.ftc.teamcode.subsystems.sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by kskrueger on 10/22/17.
 */

public class IMU {
    private BNO055IMU imu; //gets BNO055IMU class from the SDK for the Rev aka Adafruit aka Bosch IMU

    private double offset = 0; //offset will default to 0, but can be changed using setHeadingOffset(offsetNumberHere)

    public void initIMU(HardwareMap hwMap, String name){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, name);
        imu.initialize(parameters);
    }

    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle+360+offset)%360;
    }

    public void setHeadingOffset(double offset) {
        this.offset = offset;
    }

    private double getError(double targetAngle) {
        double angleError;
        angleError = (targetAngle - getHeading());
        angleError -= (360*Math.floor(0.5+((angleError)/360.0)));

        return angleError;
    }
}