package org.firstinspires.ftc.teamcode.subsystems.sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "LED Strip Test", group = "TeleOp")
//@Disabled
public class LEDStripTest extends LinearOpMode
{
    DigitalChannel scl;
    DigitalChannel sda;

    @Override
    public void runOpMode ()
    {
        scl = hardwareMap.digitalChannel.get("ci");
        sda = hardwareMap.digitalChannel.get("di");

        scl.setMode(DigitalChannel.Mode.OUTPUT);
        sda.setMode(DigitalChannel.Mode.OUTPUT);


        setPinsLow();

        waitForStart();

        telemetry.addLine("Clocking...");
        telemetry.update();

        while (opModeIsActive())
        {
            startFrame();
            setLedsRed();
            endFrame();
            startFrame();
            setLedsBlue();
            endFrame();

        }

        setPinsLow();

        telemetry.addLine("Done!");
        telemetry.update();
    }

    private void setLedsBlue()
    {
        for (int i2 = 0; (i2 < 30) && opModeIsActive(); i2++)
        {
            sda.setState(true);
            for(int i = 0; (i < 8) && opModeIsActive(); i++)
            {
                clock();
            }

            sda.setState(true);
            for(int i = 0; (i < 8) && opModeIsActive(); i++)
            {
                clock();
            }

            sda.setState(false);
            for(int i = 0; (i < 16) && opModeIsActive(); i++)
            {
                clock();
            }
        }
    }

    private void setLedsRed()
    {
        for (int i2 = 0; (i2 < 30) && opModeIsActive(); i2++)
        {
            sda.setState(true);
            for(int i = 0; (i < 8) && opModeIsActive(); i++)
            {
                clock();
            }

            sda.setState(false);
            for(int i = 0; (i < 16) && opModeIsActive(); i++)
            {
                clock();
            }

            sda.setState(true);
            for(int i = 0; (i < 8) && opModeIsActive(); i++)
            {
                clock();
            }
        }
    }

    private void setPinsLow()
    {
        scl.setState(false);
        sda.setState(false);
    }

    private void setPinsHigh()
    {
        scl.setState(true);
        sda.setState(true);
    }

    private void startFrame()
    {
        sda.setState(false);
        for(int i = 0; (i < 32) && opModeIsActive(); i++)
        {
            clock();
        }
    }

    private void endFrame()
    {
        sda.setState(true);
        for(int i = 0; (i < 32) && opModeIsActive(); i++)
        {
            clock();
        }
    }

    private void clock()
    {
        try
        {
            //waitOneFullHardwareCycle();
            scl.setState(true);
            //waitOneFullHardwareCycle();
            scl.setState(false);
            //waitOneFullHardwareCycle();
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
    }
}