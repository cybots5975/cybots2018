package org.firstinspires.ftc.teamcode.util;

/**
 * Created by kskrueger on 12/25/17.
 */

public class PID {
    double kP;
    double kI;
    double kD;
    double lastTime;
    double integral = 0;
    int previousError = 0;
    double out;
    private int angleError;
    int error;
    private int tolerance = 0;
    public boolean withinTolerance = false;
    int targetValue;

    public PID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public PID(){
    }

    public void setVariables(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setTolerance (int tolerance) {
        this.tolerance = tolerance;
    }

    public double run(int targetValueInput, int position) {
        double dt = (System.currentTimeMillis() - lastTime);
        lastTime = System.currentTimeMillis();
        if (dt < 1) {
            return 0;
        }

        targetValue = targetValueInput;

        if (targetValueInput<0) {
            targetValue += 360;
        }

        angleError = (targetValue - position);
        angleError -= (360*Math.floor(0.5+((angleError)/360.0)));

        error = angleError;

        integral += kI * error * dt;

        if(integral > targetValue * 0.25) {
            integral = targetValue * 0.25;
        }

        double u = (kP * error + integral + kD * (error - previousError) / dt);

        previousError = error;

        withinTolerance = Math.abs(error) < tolerance;

        if (Math.abs(u)<.05) {
            u += Math.signum(u)*.05;
        }

        return u;
    }

    public double runDistance(int targetValue, int position) {
        double dt = (System.currentTimeMillis() - lastTime);
        lastTime = System.currentTimeMillis();

        int error = (targetValue - position);

        integral += kI * error * dt;

        if(integral > targetValue * 0.25) {
            integral = targetValue * 0.25;
        }

        double u = (kP * error + integral + kD * (error - previousError) / dt);

        previousError = error;

        return u;
    }

    /*public double update(double measured) {
        //grab the error for caching or use in other calculates
        double error = setPoint - measured;

        //if the PID has yet to execute more than once grab a timestamp to use in the future
        if (cycleTime == 0) {
            this.cycleTime = System.currentTimeMillis();
            previousError = error;
            return 0;
        }

        //calculate error and then find proprtional through adjusting
        proportional = kP * error;

        //check if integral is in range otherwise zero it out
        if ((integralRange == 0 || Math.abs(error) < integralRange)) integral += kI * error * dt;
        else integral = 0;

        double previousPosition = (previousError < 0 ? -1 : previousError > 0 ? 1 : 0), currentPosition = (error < 0 ? -1 : error > 0 ? 1 : 0);
        if (previousPosition != currentPosition) {
            integral = 0;
            RobotLog.ii("RESET", "Reset Integral");
        }
        //calculate derivative and then increase it by its kD
        derivative = kD * (error - previousError) / dt;
        //sanity check to prevent errors in the derivative
        derivative = (isNaN(derivative) || isInfinite(derivative) ? 0 : derivative);

        //save previous error for next integral
        previousError = error;

        //ensure that the PID is only calculating at certain intervals otherwise halt till next time
        if (this.delay > 0)
            pause((long) (this.delay - (System.currentTimeMillis() - this.cycleTime)));
        dt = System.currentTimeMillis() - this.cycleTime;
        cycleTime = System.currentTimeMillis();
        Logger.getInstance().queueData("P", proportional).queueData("I", integral).queueData("D", derivative).queueData("Reading", measured);
        //calculate the PID result
        double result = proportional + integral + derivative;
        //limit the PID result if range is present
        if (outputRange != 0) result = Math.max(-outputRange, Math.min(outputRange, result));
        return result;
    }*/
}
