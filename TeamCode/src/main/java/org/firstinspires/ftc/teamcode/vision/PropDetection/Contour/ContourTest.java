package org.firstinspires.ftc.teamcode.vision.PropDetection.Contour;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Disabled
@TeleOp(name="Contour Test")
public class ContourTest extends LinearOpMode {

    VisionPortal visionPortal;
    PropContour propContour;

    @Override
    public void runOpMode() throws InterruptedException {
        propContour = new PropContour();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(propContour)
                .build();

        while (true) {
            if (!opModeInInit() || isStopRequested()) break;
        }
    }
}
