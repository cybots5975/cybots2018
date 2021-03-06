package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Created by kskrueger on 10/27/17.
 */

public class Constants {
    public static final double      FL_OFFSET               = 1.638;
    public static final double      BL_OFFSET               = 1.046;
    public static final double      FR_OFFSET               = 1.326;
    public static final double      BR_OFFSET               = 1.426;

    private static final double     COUNTS_PER_MOTOR_REV    = 1440;    //Neverest Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 2/3;     //This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0;     //For figuring circumference
    public static final double      COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV *
                                                               DRIVE_GEAR_REDUCTION) /
                                                               (WHEEL_DIAMETER_INCHES * Math.PI);

    //private static final String     FLMotor                 = "DM1";
}