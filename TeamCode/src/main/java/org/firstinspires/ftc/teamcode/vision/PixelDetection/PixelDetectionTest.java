package org.firstinspires.ftc.teamcode.vision.PixelDetection;

import android.util.Size;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Disabled
//@Config
@TeleOp(name="Pixel Detection Test")
public class PixelDetectionTest extends OpMode {

    PixelDetectionProcessor pixelDetectionProcessor;
    VisionPortal visionPortal;

    public static double lowerThreshold = 0.15;
    public static double upperThreshold = 0.3;
    public static int topLeftX = 213;
    public static int topLeftY = 0;
    public static int botLeftX = 426;
    public static int botLeftY = 480;

    @Override
    public void init() {
        pixelDetectionProcessor = new PixelDetectionProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(pixelDetectionProcessor)
                .build();

    }

    @Override
    public void loop() {

        pixelDetectionProcessor.setMIDDLE_RECT(topLeftX, topLeftY, botLeftX, botLeftY);
        pixelDetectionProcessor.setThreshLower(lowerThreshold);
        pixelDetectionProcessor.setThreshUpper(upperThreshold);

        double perc = pixelDetectionProcessor.percUncovered;
        double num = pixelDetectionProcessor.numPixels;

        telemetry.addData("Percent", perc);
        telemetry.addData("Number", num);
    }

    @Override
    public void stop() {
        visionPortal.close();
    }
}
