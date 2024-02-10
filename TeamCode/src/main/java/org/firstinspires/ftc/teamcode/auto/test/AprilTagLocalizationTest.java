package org.firstinspires.ftc.teamcode.auto.test;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helper.IMUControl;
import org.firstinspires.ftc.teamcode.auto.util.VisionController;

import java.util.ArrayList;

//@Disabled
@TeleOp(name="April Tag Localization Test", group="Vision")
public class AprilTagLocalizationTest extends OpMode {

    VisionController visionController;
    IMUControl imuControl;
    FtcDashboard dashboard;

    @Override
    public void init() {
        imuControl = new IMUControl(hardwareMap, telemetry, 0);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        visionController = new VisionController(hardwareMap, true, true, true, telemetry, dashboard);
        visionController.enableAprilTags();

    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        ArrayList<Pose2d> poses = visionController.getRaw();
        int i = 1;
        for (Pose2d pose : poses) {
            telemetry.addLine(String.format("Detected Pose #%d | x: %.2f, y: %.2f, heading: %.2f",
                    i, pose.getX(), pose.getY(),
                    pose.getHeading()));
            i++;
        }
    }
}
