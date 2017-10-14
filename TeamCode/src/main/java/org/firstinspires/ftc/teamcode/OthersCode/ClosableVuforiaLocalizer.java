package org.firstinspires.ftc.teamcode.OthersCode;

import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

/**
 * <p>
 * This is a VuforiaLocalizer replacement, as it can do everything it does, as well as detach from the camera.
 * </p>
 * <p>
 *     To use, one can substitue "VuforiaLocalizer" with "ClosableVuforiaLocalizer", and replace the equivalent of
 *         this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
 *     with
 *         this.vuforia = new ClosableVuforiaLocalizer(parameters);
 *
 *     To close vuforia, simply call vuforia.close();
 * </p>
 */

public class ClosableVuforiaLocalizer extends VuforiaLocalizerImpl {
    public ClosableVuforiaLocalizer(Parameters parameters) {
        super(parameters);
    }
    @Override
    public void close() {
        super.close();
    }
}
