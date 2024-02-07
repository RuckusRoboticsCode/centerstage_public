package org.firstinspires.ftc.teamcode.vision.PropDetection.Contour;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Disabled
@TeleOp(name="Contour Test LAB")
public class ContourTestLAB extends LinearOpMode {

    VisionPortal visionPortal;
    PropContourLAB propContour;

    @Override
    public void runOpMode() throws InterruptedException {
        propContour = new PropContourLAB();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(propContour)
                .build();

        while (true) {
            if (!opModeInInit() || isStopRequested()) break;
//            telemetry.addData("Channels", propContour.channels);
            telemetry.update();
        }

        visionPortal.close();
    }
}
