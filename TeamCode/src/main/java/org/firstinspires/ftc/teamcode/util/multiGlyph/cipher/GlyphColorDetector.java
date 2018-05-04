package org.firstinspires.ftc.teamcode.util.multiGlyph.cipher;
/*
Determine the color of an object using weights and biases calculated by a python
machine learning algorithm
*/

import com.qualcomm.robotcore.hardware.ColorSensor;

import static org.firstinspires.ftc.teamcode.util.multiGlyph.cipher.CipherMatch.GlyphColor.BROWN;
import static org.firstinspires.ftc.teamcode.util.multiGlyph.cipher.CipherMatch.GlyphColor.GREY;
import static org.firstinspires.ftc.teamcode.util.multiGlyph.cipher.CipherMatch.GlyphColor.UNKNOWN;

public class GlyphColorDetector {
    //Initialize weights for each color and biases
    private static final double ALPHA_COEF = -0.7219657;
    private static final double RED_COEF = 4.41198906;
    private static final double GREEN_COEF = -0.56744802;
    private static final double BLUE_COEF = -2.26867769;
    private static final double BIAS = 1.08390786;

    private int colorInt;
    private boolean glyphGrey;

    private ColorSensor ColorSensor;

    public GlyphColorDetector (ColorSensor ColorSensor) {
        this.ColorSensor = ColorSensor;
    }

    //Calculate sigmoid of inputted data
    private static double sigmoid(double x) {
        return 1 / (1 + Math.pow(Math.E, (-x)));
    }

    public CipherMatch.GlyphColor glyphColorML() {
        colorInt = (int) Math.round(sigmoid((
                        ColorSensor.alpha() * ALPHA_COEF +
                        ColorSensor.red() * RED_COEF +
                        ColorSensor.green() * GREEN_COEF +
                        ColorSensor.blue() * BLUE_COEF) + BIAS));

        return (colorInt == 0) ? GREY : BROWN;
    }

    public CipherMatch.GlyphColor glyphColor() {
        double alpha = ColorSensor.alpha();
        double red = ColorSensor.red();
        double blue = ColorSensor.blue();
        double green = ColorSensor.green();

        if((alpha > 73 && alpha < 130)
           && (red > 24 && red < 45)
           && (blue > 22 && blue < 45)
           && (green > 22 && green < 45))
            return GREY;

        if((alpha > 40 && alpha < 86)
           && (red > 12 && red < 30)
           && (blue > 10 && blue < 26)
           && (green > 10 && green < 26))
            return BROWN;

        return UNKNOWN;
    }
}


