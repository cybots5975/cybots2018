package org.firstinspires.ftc.teamcode.drivebase.swerve.core;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Sensors.IMU;

/**
 * Created by jordanbrobots on 11/8/17.
 */

public class FakeIMU extends IMU {

    @Override
    public void initIMU(HardwareMap hwMap, String name) {

    }

    @Override
    public double getHeading(double offset) {
        //Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //return (angles.firstAngle+360+offset)%360;
        return 0;
    }
}
