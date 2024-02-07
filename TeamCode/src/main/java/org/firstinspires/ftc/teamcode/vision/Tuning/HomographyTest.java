package org.firstinspires.ftc.teamcode.vision.Tuning;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Mat;

@Disabled
public class HomographyTest extends LinearOpMode {

    VisionPortal visionPortal;
    HomographyProcessor homographyProcessor;
    Mat homography = null;

    @Override
    public void runOpMode() throws InterruptedException {
        homographyProcessor = new HomographyProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(homographyProcessor)
                .build();

        visionPortal.setProcessorEnabled(homographyProcessor, true);

        while (opModeInInit() && !isStopRequested()) {
            if (homography == null) {
                homography = homographyProcessor.getHomography();
            } else {
                telemetry.addData("", homography);
            }
        }
    }

}
