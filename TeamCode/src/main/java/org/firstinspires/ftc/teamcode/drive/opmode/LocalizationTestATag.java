package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.util.VisionController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.helper.LinearOpModeEx;

import java.util.ArrayList;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
//@Disabled
@TeleOp(group = "drive")
public class LocalizationTestATag extends LinearOpModeEx {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        VisionController visionController = new VisionController(hardwareMap, true, true, true, telemetry, dashboard);
        visionController.enableAprilTags();
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            ArrayList<Pose2d> poses = visionController.getPositions(Math.toRadians(poseEstimate.getHeading()));
            int i = 1;
            for (Pose2d pose : poses) {
                telemetry.addLine(String.format("Detected Pose #%d | x: %.2f, y: %.2f, heading: %.2f",
                        i, pose.getX(), pose.getY(),
                        Math.toDegrees(pose.getHeading())));
                i++;
            }
//
//            telemetry.addData("ATag x", poseEstimate.getX());
//            telemetry.addData("ATag y", poseEstimate.getY());
//            telemetry.addData("ATag heading", poseEstimate.getHeading());

            telemetry.update();
        }
    }
}
