package org.firstinspires.ftc.teamcode.subsystems.sensors;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

/**
 * Created by Data Force Team 6929 on 10/4/2017.
 */

public class VexEncoder {
    //ADDRESSES
    private final int ADDRESS_REGISTER = 0x4D;
    private final int POSITION_REGISTER = 0x40;
    private final int VELOCITY_REGISTER = 0x44;

    //Variables
    I2cDeviceSynch device;
    public int currentSetAddress;

    public VexEncoder(int address, I2cDeviceSynch device){
        super();
        this.device = device;
        this.currentSetAddress = 0;

        device.engage();
        device.write8(ADDRESS_REGISTER, address << 1);
        device.setI2cAddress(new I2cAddr(address));
        zero();
    }

    /**
     * Method used to change the software I2c address of the given I2cDeviceSynch
     * @param address The 7-bit address to change the software to.
     */
    public void setSoftwareAddress(int address){
        device.setI2cAddress(new I2cAddr(address));
    }

    /**
     * Gets the current position in ticks of the encoder.
     * @return An unsigned integer of the position of the encoder.
     */
    public int getPosition(){
        //Read position bytes
        int position;
        byte[] vals = device.read(POSITION_REGISTER, 4);

        //Update current address variable for debugging
        currentSetAddress = device.getI2cAddress().get8Bit();

        //Change bytes from signed to unsigned (bitwise AND) and OR them together into a single value
        position = ((vals[0] << 8) & 0x0000ff00) | (vals[1] & 0x000000ff) | ((vals[3] << 16) & 0x00ff0000) | ((vals[2] << 24) & 0xff000000);
        return position;
    }

    /**
     * Gets the unsigned velocity bytes from the velocity register.
     * @return An integer (unsigned) of the current velocity of the encoder.
     */
    public int getUnsignedVelocity(){
        int speed;

        //Read  2 byte values from velocity register
        byte[] vals = device.read(VELOCITY_REGISTER, 2);

        //Change the bytes from signed to unsigned and bitwise OR them together.
        speed = ((vals[0] << 8) & 0x0000ff00) | (vals[1] & 0x000000ff);
        return speed;
    }

    /**
     * Resets the device counter to 0 through the 0x4A register, sending a single bit
     */
    public void zero(){
        device.write8(0x4A, 1);
    }

    /**
     * Gets the raw value that is returned from the first byte of the position register
     * @return The raw position value from the first position register byte
     */
    public int getRawAddressValue(){
        return device.read8(POSITION_REGISTER);
    }

    /**
     * Gets the 7-bit address of the selected device.
     * @return The address of the selected device in 7-bit format
     */
    public int getAddress(){
        return device.getI2cAddress().get7Bit();
    }
}