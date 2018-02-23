package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DotStar
{
    private DigitalChannel sda, scl;

    public DotStar(HardwareMap hardwareMap, String clockName, String dataName)
    {
        sda = hardwareMap.digitalChannel.get(dataName);
        scl = hardwareMap.digitalChannel.get(clockName);

        init();
    }

    private void init()
    {
        scl.setMode(DigitalChannel.Mode.OUTPUT);
        sda.setMode(DigitalChannel.Mode.OUTPUT);

        scl.setState(false);
        sda.setState(false);
    }

    private void clock()
    {
        scl.setState(true);
        scl.setState(false);
    }

    private void sendLedFrame(byte r, byte g, byte b)
    {
        sendLedStartFrame();
        writeByte(b);
        writeByte(g);
        writeByte(r);
    }

    public void setEntireStrip(byte r, byte g, byte b)
    {
        sendStripStartSequence();
        for(int i = 0; i < 30; i++)
        {
            sendLedFrame(r, g, b);
        }
        sendStripEndSequence();
    }

    public void setColorFirstLedOnly(byte r, byte g, byte b)
    {
        sendStripStartSequence();
        sendLedFrame(r, g, b);
        //sendStripEndSequence();
    }

    private void writeByte(byte b)
    {
        for (int i = 0; i < 8; i++)
        {
            sda.setState(((b >> i) & 1) == 1);
            clock();
        }
    }

    private void sendLedStartFrame()
    {
        sda.setState(true);

        /*
         * Send 3 ones for start, 5 ones
         * for brightness
         */
        for(int i = 0; i < 8; i++)
        {
            clock();
        }

        sda.setState(false);
    }

    private void sendStripStartSequence()
    {
        sda.setState(false);

        /*
         * Send 32 zeros
         */
        for(int i = 0; i < 32; i++)
        {
            clock();
        }
    }

    private void sendStripEndSequence()
    {
        sda.setState(true);

        /*
         * Send 32 ones
         */
        for(int i = 0; i < 32; i++)
        {
            clock();
        }

        sda.setState(false);
    }
}