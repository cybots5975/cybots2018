package org.firstinspires.ftc.teamcode.util.multiGlyph.cipher;
/*
Determine the color of an object using weights and biases calculated by a python
machine learning algorithm
*/

import com.qualcomm.robotcore.hardware.ColorSensor;

import static org.firstinspires.ftc.teamcode.util.multiGlyph.cipher.CipherMatch.GlyphColor.BROWN;
import static org.firstinspires.ftc.teamcode.util.multiGlyph.cipher.CipherMatch.GlyphColor.GREY;

public class GlyphColorDetection {
    //Initialize weights for each color and biases
    private static final double ALPHA_COEF = -0.7219657;
    private static final double RED_COEF = 4.41198906;
    private static final double GREEN_COEF = -0.56744802;
    private static final double BLUE_COEF = -2.26867769;
    private static final double BIAS = 1.08390786;

    int colorInt;

    ColorSensor ColorSensor;

    public GlyphColorDetection (ColorSensor ColorSensor) {
        this.ColorSensor = ColorSensor;
    }

    //Calculate sigmoid of inputted data
    public static double sigmoid(double x) {
        return 1 / (1 + Math.pow(Math.E, (-x)));
    }

    public CipherMatch.GlyphColor glyphColor() {
        colorInt = (int) Math.round(sigmoid((
                        ColorSensor.alpha() * ALPHA_COEF +
                        ColorSensor.red() * RED_COEF +
                        ColorSensor.green() * GREEN_COEF +
                        ColorSensor.blue() * BLUE_COEF) + BIAS));

        return (colorInt == 0) ? GREY : BROWN;
    }
}


