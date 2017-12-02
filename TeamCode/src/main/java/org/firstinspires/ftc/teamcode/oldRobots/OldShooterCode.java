package org.firstinspires.ftc.teamcode.oldRobots;

import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Created by kskrueger on 11/27/17.
 */

public class OldShooterCode {

    //shooter distance variables
    double velocity; //velocity of the shooting wheels
    double goalHypDistance; //hypotenuse distance to center goal (used for calculating trajectory)
    double angle; //angle of chute
    double height = .85; //height of goal relative to the robot top in meters
    double distanceToGoal; //distance to goal (calculated using hypotenuse measured by sensor)
    double totalDistance; //calulates target position for ball to go through the goal (calculated from distanceToGoal)
    double servoCounts = .0066; //servo counts per 1 degree of chute angle movement
    double chutePosition = .299; //starting position of the chute
    double position90 = .310; //was previously .302 (servo value of chute base at 90 degrees to robot)
    double shooterConstant = .575;
    double sensorAngle = 34.2;
    AnalogInput goalRange;

    void code () {
        goalHypDistance = goalRange.getVoltage() / 0.009766; //distance of hypotenuse to goal in inches
        distanceToGoal = goalHypDistance * (Math.cos(sensorAngle/57.298)); //find the distance to the goal using cosine of hypotenuse and sensor angle
        totalDistance = distanceToGoal * .0254 / 0.71 + .5; //convert distance to goal to meters, then convert to distance THROUGH goal, then add offset of .5meters
        angle = (Math.atan(((4 * height) / totalDistance)) * 57.298); //calculate angle of ball to go through goal
        velocity = ((Math.sqrt(2 * 9.80665 * height)) / (Math.sin(angle / 57.298))); //calculate velocity of ball for trajectory
        chutePosition = position90 - (servoCounts * (62.5 - angle)); //convert angle of the chute to get correct trajectory

        double shooterSpeed = shooterConstant / 4.6 * velocity; //convert velocity to shooter speed level for correct trajectory
    }
}
