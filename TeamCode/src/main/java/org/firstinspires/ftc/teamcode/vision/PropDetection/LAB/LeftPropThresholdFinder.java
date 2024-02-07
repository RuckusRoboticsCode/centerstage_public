package org.firstinspires.ftc.teamcode.vision.PropDetection.LAB;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Disabled
@Config
@TeleOp(name="Left Prop Thresholder", group="Vision")
public class LeftPropThresholdFinder extends OpMode {

    VisionPortal visionPortal;
    LeftPropProcessor leftPropProcessor;
    FtcDashboard dashboard;
    public static double leftThreshold = 0.15;
    public static double middleThreshold = 0.15;

    @Override
    public void init() {

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(20);

        leftPropProcessor = new LeftPropProcessor();
        leftPropProcessor.setLeftThreshold(leftThreshold);
        leftPropProcessor.setMiddleThreshold(middleThreshold);
        leftPropProcessor.setDetectingRed(true);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(leftPropProcessor)
                .build();
    }

    @Override
    public void init_loop() {
        leftPropProcessor.setLeftThreshold(leftThreshold);
        leftPropProcessor.setMiddleThreshold(middleThreshold);
        telemetry.addData("Left Percent", "%.4f", leftPropProcessor.leftPerc);
        telemetry.addData("Middle Percent", "%.4f", leftPropProcessor.middlePerc);
        telemetry.update();
    }

    @Override
    public void loop() {

    }
}
