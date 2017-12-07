package org.firstinspires.ftc.teamcode.general;

/**
 * Created by kskrueger on 10/27/17.
 */

public class Constants {
    public static final double FL_OFFSET= 1.644;
    public static final double BL_OFFSET= 1.051;
    public static final double FR_OFFSET= 1.334;
    public static final double BR_OFFSET= 1.439; //was 1.181 on 11/18/17 before the replacement

    private static final double     COUNTS_PER_MOTOR_REV    = 1440;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 2/3;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0;     // For figuring circumference
    public static final double      COUNTS_PER_INCH          = (COUNTS_PER_MOTOR_REV *
                                                                DRIVE_GEAR_REDUCTION) /
                                                                (WHEEL_DIAMETER_INCHES * Math.PI);
}