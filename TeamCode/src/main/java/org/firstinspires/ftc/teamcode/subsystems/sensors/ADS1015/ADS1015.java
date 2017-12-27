package org.firstinspires.ftc.teamcode.subsystems.sensors.ADS1015;

import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * ADS1015 Sensor constants and interface
 * @author Jaxon A Brown
 */
public interface ADS1015 {
    I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x48);

    void setGain(GainAmplifier gain);
    double getVoltage(int channel);

    enum Registers {
        CONVERSION(0x00),
        CONFIGURATION(0x01),
        LOW_THRESHOLD(0x02),
        HIGH_THRESHOLD(0x03);

        public final int bVal;

        Registers(int bVal) {
            this.bVal = bVal;
        }
    }

    enum ConfigurationMasks {
        OPERATIONAL_STATUS(0x8000),
        INPUT_MULTIPLEXER(0x7000),
        GAIN_AMPLIFIER(0x0E00),
        MODE(0x0100),
        DATA_RATE(0x00E0),
        COMPARATOR_MODE(0x0010),
        COMPARATOR_POLARITY(0x0008),
        COMPARATOR_LATCHING(0x0004),
        COMPARATOR_QUEUE(0x0003);

        public final short mask;

        ConfigurationMasks(int mask) {
            this.mask = (short) mask;
        }
    }

    enum ConfigurationFlags {
        OPERATIONAL_STATUS_READ_BUSY(0x0000),                               // Device is busy
        OPERATIONAL_STATUS_READ_NOT_BUSY(0x8000),                           // Device is free
        OPERATIONAL_STATUS_WRITE_NO_EFFECT(0x0000),                         // Has no effect
        OPERATIONAL_STATUS_WRITE_START_SINGLE_CONVERSION(0x8000),           // Start single conversion

        INPUT_MULTIPLEXER_DIFF_0_1(0x0000),                                 // Differential between A0 and A1
        INPUT_MULTIPLEXER_DIFF_0_3(0x1000),                                 // Differential between A0 and A3
        INPUT_MULTIPLEXER_DIFF_1_3(0x2000),                                 // Differential between A1 and A3
        INPUT_MULTIPLEXER_DIFF_2_3(0x3000),                                 // Differential between A2 and A3
        INPUT_MULTIPLEXER_SINGLE_0(0x4000),                                 // Single on A0
        INPUT_MULTIPLEXER_SINGLE_1(0x5000),                                 // Single on A1
        INPUT_MULTIPLEXER_SINGLE_2(0x6000),                                 // Single on A2
        INPUT_MULTIPLEXER_SINGLE_3(0x7000),                                 // Single on A3

        GAIN_AMPLIFIER_6_144V(0x0000),                                      // +/-6.144V range = Gain x2/3
        GAIN_AMPLIFIER_4_096V(0x0200),                                      // +/-4.096V range = Gain x1
        GAIN_AMPLIFIER_2_048V(0x0400),                                      // +/-2.048V range = Gain x2
        GAIN_AMPLIFIER_1_024V(0x0600),                                      // +/-1.024V range = Gain x4
        GAIN_AMPLIFIER_0_512V(0x0800),                                      // +/-0.512V range = Gain x8
        GAIN_AMPLIFIER_0_256V(0x0A00),                                      // +/-0.256V range = Gain x16

        MODE_CONTINUOUS(0x0000),                                            // Continuous conversion mode
        MODE_SINGLE(0x0100),                                                // Single Conversion

        DATA_RATE_128(0x0000),                                              // 128 samples per second
        DATA_RATE_250(0x0020),                                              // 250 samples per second
        DATA_RATE_490(0x0040),                                              // 490 samples per second
        DATA_RATE_920(0x0060),                                              // 920 samples per second
        DATA_RATE_1600(0x0080),                                             // 1600 samples per second
        DATA_RATE_2400(0x00A0),                                             // 2400 samples per second
        DATA_RATE_3300(0x00C0),                                             // 3300 samples per second

        COMPARATOR_MODE_TRADITIONAL(0x0000),                                // Use traditional comparator with hysteresis
        COMPARATOR_MODE_WINDOW(0x0010),                                     // Use window comparator

        COMPARATOR_POLARITY_LOW(0x0000),                                    // Comparator output is active low
        COMPARATOR_POLARITY_HIGH(0x0008),                                   // Comparator output is active high

        COMPARATOR_LATCHING_DISABLED(0x0000),                               // ALERT/RDY pin doesn't latch
        COMPARATOR_LATCHING_ENABLED(0x0004),                                // ALERT/RDY pin latches; eg, stays active until conversion data is read

        COMPARATOR_QUEUE_1CONV(0x0000),                                     // Assert the ALERT/RDY pin after 1 successive conversions exceeding the threshold
        COMPARATOR_QUEUE_2CONV(0x0001),                                     // Assert the ALERT/RDY pin after 2 successive conversions exceeding the threshold
        COMPARATOR_QUEUE_4CONV(0x0002),                                     // Assert the ALERT/RDY pin after 4 successive conversions exceeding the threshold
        COMPARATOR_QUEUE_DISABLED(0x0003);                                  // Disable comparator

        public final short flag;

        ConfigurationFlags(int flag) {
            this.flag = (short) flag;
        }
    }

    enum GainAmplifier {
        GAIN_6_144V(ConfigurationFlags.GAIN_AMPLIFIER_6_144V, 6.144),       // +/-6.144V range = Gain x2/3
        GAIN_4_096V(ConfigurationFlags.GAIN_AMPLIFIER_4_096V, 4.096),       // +/-4.096V range = Gain x1
        GAIN_2_048V(ConfigurationFlags.GAIN_AMPLIFIER_2_048V, 2.048),       // +/-2.048V range = Gain x2
        GAIN_1_024V(ConfigurationFlags.GAIN_AMPLIFIER_1_024V, 1.024),       // +/-1.024V range = Gain x4
        GAIN_0_512V(ConfigurationFlags.GAIN_AMPLIFIER_0_512V, 0.512),       // +/-0.512V range = Gain x8
        GAIN_0_256V(ConfigurationFlags.GAIN_AMPLIFIER_0_256V, 0.256);       // +/-0.256V range = Gain x16

        public final ConfigurationFlags flag;
        public final double voltage;

        GainAmplifier(ConfigurationFlags flag, double voltage) {
            this.flag = flag;
            this.voltage = voltage;
        }
    }
}