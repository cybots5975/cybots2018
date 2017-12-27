package org.firstinspires.ftc.teamcode.util.vuforia;

import android.app.Activity;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.nio.ByteBuffer;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.CountDownLatch;

public class VisionCamera implements OpModeManagerNotifier.Notifications {
    public static final String TAG = "VisionCamera";

    private VuforiaLocalizer vuforia;
    private VuforiaLocalizer.Parameters vuforiaParams;
    private FrameConsumer frameConsumer;
    private File imageDir;
    private int imageNum;

    private AppUtil appUtil = AppUtil.getInstance();
    private Activity activity;
    private OpModeManagerImpl opModeManager;

    private class FrameConsumer extends Thread {
        public static final String TAG = "FrameConsumer";

        private BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue;
        private Mat frame;
        private byte[] frameBuffer;
        private boolean running;

        public FrameConsumer(BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue) {
            this.frameQueue = frameQueue;
            this.running = true;
        }

        @Override
        public void run() {
            while (running) {
                // grab frames and process them
                if (!frameQueue.isEmpty()) {
                    VuforiaLocalizer.CloseableFrame vuforiaFrame = null;
                    try {
                        vuforiaFrame = frameQueue.take();
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }

                    if (vuforiaFrame == null) {
                        continue;
                    }

                    long startTime = System.currentTimeMillis();
                    for (int i = 0; i < vuforiaFrame.getNumImages(); i++) {
                        Image image = vuforiaFrame.getImage(i);
                        if (image.getFormat() == PIXEL_FORMAT.RGB888) {
                            int imageWidth = image.getWidth(), imageHeight = image.getHeight();
                            ByteBuffer byteBuffer = image.getPixels();
                            if (frameBuffer == null) {
                                frameBuffer = new byte[byteBuffer.capacity()];
                            }
                            byteBuffer.get(frameBuffer);
                            if (this.frame == null) {
                                this.frame = new Mat(imageHeight, imageWidth, CvType.CV_8UC3);
                            }
                            this.frame.put(0, 0, frameBuffer);

                            Imgproc.cvtColor(this.frame, this.frame, Imgproc.COLOR_RGB2BGR);

                            if (vuforiaParams.cameraDirection == VuforiaLocalizer.CameraDirection.FRONT) {
                                Core.flip(this.frame, this.frame, 1);
                            }

                            // TODO: process the frame

                            if (imageDir != null) {
                                String filename = imageNum + ".jpg";
                                Imgcodecs.imwrite(new File(imageDir, filename).getPath(), this.frame);
                                imageNum++;
                            }
                        }
                    }
                    vuforiaFrame.close();
                    Log.i(TAG, "Processed image in " + (System.currentTimeMillis() - startTime) + "ms");
                } else {
                    try {
                        Thread.sleep(5);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }
            }
        }

        public void terminate() {
            this.running = false;
        }
    }

    public VisionCamera() {
        this.activity = appUtil.getActivity();
        opModeManager = OpModeManagerImpl.getOpModeManagerOfActivity(activity);
        if (opModeManager != null) {
            opModeManager.registerListener(this);
        }
    }

    public void initialize(VuforiaLocalizer.Parameters vuforiaParams) {
        final CountDownLatch openCvInitialized = new CountDownLatch(1);

        final BaseLoaderCallback loaderCallback = new BaseLoaderCallback(activity) {
            @Override
            public void onManagerConnected(int status) {
                switch (status) {
                    case LoaderCallbackInterface.SUCCESS: {
                        Log.i(TAG, "OpenCV loaded successfully");
                        openCvInitialized.countDown();
                        break;
                    }
                    default: {
                        super.onManagerConnected(status);
                        break;
                    }
                }
            }
        };

        appUtil.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_2_0, activity, loaderCallback);
            }
        });

        vuforia = ClassFactory.createVuforiaLocalizer(vuforiaParams);
        this.vuforiaParams = vuforiaParams;

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        vuforia.setFrameQueueCapacity(1);

        try {
            openCvInitialized.await();
        } catch (InterruptedException e) {
            Log.w(TAG, e);
        }

        frameConsumer = new FrameConsumer(vuforia.getFrameQueue());
        frameConsumer.start();
    }

    public void setImageDir(File imageDir) {
        this.imageDir = imageDir;
    }

    public VuforiaLocalizer getVuforia() {
        return this.vuforia;
    }

    public void close() {
        if (frameConsumer != null) {
            frameConsumer.terminate();
            try {
                frameConsumer.join();
            } catch (InterruptedException e) {
                Log.w(TAG, e);
            }
            frameConsumer = null;
        }
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {

    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        close();
        if (opModeManager != null) {
            opModeManager.unregisterListener(this);
        }
    }
}