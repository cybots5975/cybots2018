package org.firstinspires.ftc.teamcode.drivebase.swerve.core;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;

/**
 * Created by jordanbrobots on 11/8/17.
 */
public class SwerveDriveTest {
    SwerveDrive swerve;
    @Before
    public void setUp() throws Exception {
        swerve = new SwerveDrive(new FakeIMU(), new FakeIMU(),
                new FakeDcMotor(), new FakeServo(), new FakeAnalogInput(null, 0),
                new FakeDcMotor(), new FakeServo(), new FakeAnalogInput(null,0),
                new FakeDcMotor(), new FakeServo(), new FakeAnalogInput(null, 0),
                new FakeDcMotor(), new FakeServo(), new FakeAnalogInput(null,0));
    }

    @After
    public void tearDown() throws Exception {
    }

    @Test
    public void robotCentric() throws Exception {
        System.out.println("Drive Forward 1");
        swerve.RobotCentric(0,1,0,false);
        System.out.printf("D1 Angle: %d\n", swerve.D1.getTargeAngle());
        System.out.printf("D2 Angle: %d\n", swerve.D2.getTargeAngle());
        System.out.printf("P1 Angle: %d\n", swerve.P1.getTargeAngle());
        System.out.printf("P2 Angle: %d\n\n", swerve.P2.getTargeAngle());

        System.out.println("Drive Forward -1");
        swerve.RobotCentric(0,-1,0,false);
        System.out.printf("D1 Angle: %d\n", swerve.D1.getTargeAngle());
        System.out.printf("D2 Angle: %d\n", swerve.D2.getTargeAngle());
        System.out.printf("P1 Angle: %d\n", swerve.P1.getTargeAngle());
        System.out.printf("P2 Angle: %d\n\n", swerve.P2.getTargeAngle());

        System.out.println("Drive Strafe 1");
        swerve.RobotCentric(1,0,0,false);
        System.out.printf("D1 Angle: %d\n", swerve.D1.getTargeAngle());
        System.out.printf("D2 Angle: %d\n", swerve.D2.getTargeAngle());
        System.out.printf("P1 Angle: %d\n", swerve.P1.getTargeAngle());
        System.out.printf("P2 Angle: %d\n\n", swerve.P2.getTargeAngle());

        System.out.println("Drive Strafe -1");
        swerve.RobotCentric(-1,0,0,false);
        System.out.printf("D1 Angle: %d\n", swerve.D1.getTargeAngle());
        System.out.printf("D2 Angle: %d\n", swerve.D2.getTargeAngle());
        System.out.printf("P1 Angle: %d\n", swerve.P1.getTargeAngle());
        System.out.printf("P2 Angle: %d\n\n", swerve.P2.getTargeAngle());
    }

    @Test
    public void setEfficiency() throws Exception {
    }

    @Test
    public void fieldCentric() throws Exception {
    }

    @Test
    public void moveEncoder() throws Exception {
    }

    @Test
    public void gyroMove() throws Exception {
    }

    @Test
    public void PID() throws Exception {
    }

}