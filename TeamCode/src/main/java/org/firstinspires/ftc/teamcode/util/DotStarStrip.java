package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Dot Star LED Strip
 * This library is capable of driving Dot Star LED Strips.
 * Strips should have a data and a clock pin. The Data and clock pins should be plugged into
 *      the digital output pins on either the REV hub or the MR CDIM.
 * If using the rev hub, make sure to use a level shifter between the digital pins and the pins
 *      on the LED strip.
 *
 * Note that the strip must be powered legally (ie, not with the REV hub or MR CDIM's digital power
 *      pins)
 *
 * @author Jaxon A Brown
 */

public class DotStarStrip {
    private final DigitalChannel dataPin, clockPin;
    private final DotStarPixel[] pixels;
    private double brightness = 1;

    /**
     * Sets up the Dot Star Strip. You should only use this once per strip per op mode
     * @param dataPin the DigitalChannel for the pin the Data line is connected to
     * @param clockPin the DigitalChannel for the pin the Clock line is connected to
     * @param pixelLength the number of pixels on the connected strip
     */
    public DotStarStrip(DigitalChannel dataPin, DigitalChannel clockPin, int pixelLength) {
        this.dataPin = dataPin;
        this.clockPin = clockPin;

        this.dataPin.setMode(DigitalChannel.Mode.OUTPUT);
        this.clockPin.setMode(DigitalChannel.Mode.OUTPUT);

        digitalWrite(dataPin, LOW);
        digitalWrite(clockPin, LOW);

        pixels = new DotStarPixel[pixelLength];
        for(int i = 0; i < pixelLength; i++) {
            pixels[i] = new DotStarPixel(i);
        }
    }

    /**
     * Sets up the Dot Star Strip. You should only use this once per strip per op mode
     * @param hardwareMap op mode's hardware map
     * @param dataPinName name of DigitalChannel in configuration connected to the Data line
     * @param clockPinName name of DigitalChannel in configuration connected to the Clock line
     * @param pixelLength the number of pixels on the connected strip
     */
    public DotStarStrip(HardwareMap hardwareMap, String dataPinName, String clockPinName, int pixelLength) {
        this(hardwareMap.digitalChannel.get(dataPinName), hardwareMap.digitalChannel.get(clockPinName), pixelLength);
    }

    /**
     * Get pixel at ID. The larger the ID, the further away from the connectors the pixel is
     * @param id id of the pixel id=[0,pixelLength)
     * @return the DotStarPixel at location id
     */
    public DotStarPixel getPixel(int id) {
        return pixels[id];
    }

    /**
     * Get the brightness modifier of the strip
     * @return brightness, [0,1]
     */
    public double getBrightness() {
        return brightness;
    }

    /**
     * Set the brightness modifier of the strip
     * @param brightness [0,1]
     */
    public void setBrightness(double brightness) {
        if(brightness < 0 || brightness > 1) {
            throw new IllegalArgumentException("Brightness MUST be between 0 and 1");
        }
        this.brightness = brightness;
    }

    /**
     * Updates the strip. This will perform an SPI bit-bang, and is synchronous.
     */
    public void updateStrip() {
        for(int i=0; i<4; i++) spiWrite(0x00); // Start Frame
        for(DotStarPixel pixel : pixels) { // For each pixel, in order
            // Write pixel value, multiply by brightness for dimming
            writePixel((byte) (pixel.getRed() * brightness), (byte) (pixel.getGreen() * brightness), (byte) (pixel.getBlue() * brightness));
        }
        for(int i=0; i<((pixels.length + 15) / 16); i++) spiWrite(0xFF); // End Frame
    }




    // --------------------------
    //       Internal calls
    // --------------------------

    private void writePixel(byte r, byte g, byte b) {
        spiWrite(0xFF); // Write Pixel Meta
        spiWrite(b); // Write blue first
        spiWrite(g); // Write green next
        spiWrite(r); // Write red last
    }




    // --------------------------
    //         SPI calls
    // --------------------------

    private static boolean HIGH = true, LOW = false;

    private void digitalWrite(DigitalChannel channel, boolean val) {
        channel.setState(val);
    }

    private void spiWrite(int out) {
        for(int i = 0; i < 8; i++) {
            digitalWrite(dataPin, ((out >> i) & 1) == 1);

            digitalWrite(clockPin, HIGH);
            digitalWrite(clockPin, LOW);
        }
    }


    // --------------------------
    //        API Classes
    // --------------------------

    public static class DotStarPixel {
        private final int pixelID;
        private byte red, green, blue;
        private double brightness = 1;

        private DotStarPixel(int pixelID) {
            this.pixelID = pixelID;
        }

        public int getPixelID() {
            return pixelID;
        }

        public byte getRed() {
            return (byte) (red * brightness);
        }

        public void setRed(byte red) {
            this.red = red;
        }

        public void setRed(int red) {
            this.red = (byte) red;
        }

        public byte getGreen() {
            return (byte) (green * brightness);
        }

        public void setGreen(byte green) {
            this.green = green;
        }

        public void setGreen(int green) {
            this.green = (byte) green;
        }

        public byte getBlue() {
            return (byte) (blue * brightness);
        }

        public void setBlue(byte blue) {
            this.blue = blue;
        }

        public void setBlue(int blue) {
            this.blue = (byte) blue;
        }

        public void setRGB(byte red, byte green, byte blue) {
            setRed(red);
            setGreen(green);
            setBlue(blue);
        }

        public void setRGB(int red, int green, int blue) {
            setRed(red);
            setGreen(green);
            setBlue(blue);
        }

        public double getBrightness() {
            return brightness;
        }

        public void setBrightness(double brightness) {
            if(brightness < 0 || brightness > 1) {
                throw new IllegalArgumentException("Brightness MUST be between 0 and 1");
            }
            this.brightness = brightness;
        }
    }
}