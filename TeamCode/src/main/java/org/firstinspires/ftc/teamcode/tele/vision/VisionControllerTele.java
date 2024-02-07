package org.firstinspires.ftc.teamcode.tele.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class VisionControllerTele {

    AprilTagProcessor aprilTagProcessor;
    VisionPortal visionPortal;
    ArrayList<AprilTagDetection> detections;
    Telemetry telemetry;
    boolean atBackdrop = false;
    boolean previouslySeen = false;
    boolean currentlySeen = false;

    public VisionControllerTele(HardwareMap hardwareMap, Telemetry telemetry) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)
                .build();

        aprilTagProcessor.setDecimation(3);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTagProcessor)
                .build();

        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
    }

    public void updateDetections() {
        detections = aprilTagProcessor.getDetections();
//        currentlySeen = false;
        if (detections.size() == 0) {
            atBackdrop = false;
        } else {
            for (AprilTagDetection detection : detections) {
                if (detection != null) {
                    if (detection.id <= 6 && detection.id >= 0) {
//                        telemetry.addData("Range", detection.ftcPose.range);
                        if (detection.ftcPose.range < 20) {
//                    currentlySeen = true;
                            atBackdrop = true;
                        }
                        break;
                    } else {
                        atBackdrop = false;
                    }
                } else {
                    atBackdrop = false;
                }
            }
        }
//        atBackdrop = !currentlySeen && previouslySeen;
//        previouslySeen = currentlySeen;
    }

    public boolean isAtBackdrop() {
        return atBackdrop;
    }


    public void updateTelemetry() {
        telemetry.addData("Currently Seen", currentlySeen);
        telemetry.addData("Previously Seen", previouslySeen);
        telemetry.addData("At Backdrop", atBackdrop);
    }
}
