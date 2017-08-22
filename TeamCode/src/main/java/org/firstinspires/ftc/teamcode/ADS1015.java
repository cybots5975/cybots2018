/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cWaitControl;

public class ADS1015 {

    I2cDeviceSynch device;
    I2cAddr i2cAddr;

    public ADS1015(I2cDeviceSynch device, int addr) {
        this.device = device;
        this.i2cAddr = I2cAddr.create8bit(addr);

        device.setI2cAddress(i2cAddr);
        device.setReadWindow(new I2cDeviceSynch.ReadWindow(0x00, 1, I2cDeviceSynch.ReadMode.REPEAT));
        device.engage();

        DisableArray();
    }

    public void ReadAnalog1()
    {
        device.write8(0x07, (byte)0x00, I2cWaitControl.ATOMIC);
        device.write8(0x03, (byte)0x00, I2cWaitControl.ATOMIC);
    }

    public void DisableArray()
    {
        device.write8(0x07, (byte)0xFF, I2cWaitControl.ATOMIC);
        device.write8(0x03, (byte)0xFF, I2cWaitControl.ATOMIC);
    }

    public int ReadArray()
    {
        int temp = device.read8(0x00);
        temp  = temp & 0xFF;

        int result = ((temp & 0x01) != 0 ? 0x20 : 0);
        result +=    ((temp & 0x02) != 0 ? 0x10 : 0);
        result +=    ((temp & 0x04) != 0 ? 0x80 : 0);
        result +=    ((temp & 0x08) != 0 ? 0x40 : 0);
        result +=    ((temp & 0x10) != 0 ? 0x04 : 0);
        result +=    ((temp & 0x20) != 0 ? 0x08 : 0);
        result +=    ((temp & 0x40) != 0 ? 0x02 : 0);
        result +=    ((temp & 0x80) != 0 ? 0x01 : 0);

        return result;
    }

    public double ReadPosition(int array)
    {
        double result = 0;
        int count = 0;
        if (array == 0) return -1;

        int fl = array;
        for(int i = 1; i <= 8; i++) {
            if((fl& 1) != 0) {
                result += i;
                count++;
            }

            fl = fl >> 1;
        }

        result = result / (8.0 * count);
        return result;
    }

    public double readI2Canalog (int channel) {

        if (channel > 3)
        {
            return 0;
        }

        // Start with default values
        byte config =   ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
                ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
                ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

        // Set PGA/voltage range
        config |= m_gain;

        // Set single-ended input channel
        switch (channel)
        {
            case (0):
                config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
                break;
            case (1):
                config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
                break;
            case (2):
                config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
                break;
            case (3):
                config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
                break;
        }

        // Set 'start single-conversion' bit
        config |= ADS1015_REG_CONFIG_OS_SINGLE;

        // Write config register to the ADC
        writeRegister(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);

        // Wait for the conversion to complete
        //sleep(m_conversionDelay);

        // Read the conversion results
        // Shift 12-bit results right 4 bits for the ADS1015
        return readRegister(m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift;

        return 0;
    }

    public void writeRegister(int i2cAddress, int reg, int value) {
        device.write8((byte)reg, (byte)((value>>8)), I2cWaitControl.ATOMIC);
        device.write8((byte)reg, (byte)((value & 0xFF)), I2cWaitControl.ATOMIC);
    }

    public void readRegister(int i2cAddress, int reg) {
        device.write8((byte)reg, (byte)(ADS1015_REG_POINTER_CONVERT), I2cWaitControl.ATOMIC);
        byte read = device.read8((byte)reg);
        return (read << 8) | read);
    }

    byte ADS1015_REG_CONFIG_MUX_SINGLE_0 = (byte)(0x4000);  // Single-ended AIN0
    byte ADS1015_REG_CONFIG_MUX_SINGLE_1 = (byte)(0x5000);  // Single-ended AIN1
    byte ADS1015_REG_CONFIG_MUX_SINGLE_2 = (byte)(0x6000);  // Single-ended AIN2
    byte ADS1015_REG_CONFIG_MUX_SINGLE_3 = (byte)(0x7000);  // Single-ended AIN3
    byte ADS1015_REG_CONFIG_CQUE_NONE = (byte)(0x0003);  // Disable the comparator and put ALERT/RDY in high state (default)
    byte ADS1015_REG_CONFIG_CLAT_NONLAT = (byte)(0x0000);  // Non-latching comparator (default)
    byte ADS1015_REG_CONFIG_CPOL_ACTVLOW = (byte)(0x0000);  // ALERT/RDY pin is low when active (default)
    byte ADS1015_REG_CONFIG_CMODE_TRAD = (byte)(0x0000);  // Traditional comparator with hysteresis (default)

    byte ADS1015_REG_CONFIG_DR_128SPS = (byte)(0x0000);  // 128 samples per second
    byte ADS1015_REG_CONFIG_DR_1600SPS = (byte)(0x0080);  // 1600 samples per second (default)

    byte ADS1015_REG_CONFIG_MODE_SINGLE = (byte)(0x0100);  // Power-down single-shot mode (default)
    byte ADS1015_REG_CONFIG_MODE_CONTIN = (byte)(0x0000);  // Continuous conversion mode

    byte ADS1015_REG_CONFIG_OS_SINGLE = (byte)(0x8000);  // Write: Set to start a single-conversion
    byte ADS1015_REG_POINTER_CONFIG = (byte)(0x01);

    byte m_i2cAddress;

    byte m_conversionDelay;

    byte ADS1015_REG_POINTER_CONVERT = (0x00);

    byte m_bitShift;

    byte m_gain;

    //public static final I2cAddr IR_SEEKER_V3_ORIGINAL_ADDRESS = I2cAddr.create8bit(0x38);
}

