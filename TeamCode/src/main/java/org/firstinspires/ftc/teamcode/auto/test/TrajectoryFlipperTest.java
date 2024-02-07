package org.firstinspires.ftc.teamcode.auto.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.util.RoadRunnerLocations;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//@Photon
@Disabled
@Autonomous(name="Traj Flip Test", group="Auto")
public class TrajectoryFlipperTest extends LinearOpMode {

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setFlipPose(true);
        drive.setPoseEstimate(RoadRunnerLocations.BlueLocations.CLOSE_STARTING.getPose());

        TrajectorySequence testTrajectory = drive.trajectorySequenceBuilder(RoadRunnerLocations.RedLocations.CLOSE_STARTING.getPose())
//                .forward(8)
                // spike mark
                .lineToConstantHeading(RoadRunnerLocations.RedLocations.CLOSE_MIDDLE_SPIKE.getPosition())
//
//                // backdrop
//                .splineToSplineHeading(new Pose2d(14, -40, Math.toRadians(225)), Math.toRadians(315))
//                .splineToSplineHeading(new Pose2d(RoadRunnerLocations.RedLocations.BACKDROP_MIDDLE.getPosition(), Math.toRadians(180)), Math.toRadians(0))
//
//                .waitSeconds(0.5)
//
//                // stack
//                .forward(3)
//                .splineToSplineHeading(new Pose2d(20, -11.5, Math.toRadians(180.0)), Math.toRadians(180.0))
//                .lineToConstantHeading(RoadRunnerLocations.RedLocations.STACK.getPosition())
//
//                .waitSeconds(1)
//
//                // backdrop
//                .lineToConstantHeading(new Vector2d(15, -11.5))
//                .splineToConstantHeading(RoadRunnerLocations.RedLocations.BACKDROP_MIDDLE.getPosition()
//                        .minus(new Vector2d(0, 2)), 0)
//
//                .waitSeconds(0.5)
//
//                // parking
//                .forward(8)
//                .strafeLeft(23)
//                .back(10)

                .build();

        waitForStart();

        drive.followTrajectorySequenceAsync(testTrajectory);

        while(opModeIsActive() && !isStopRequested()) {
//            drive.updateTelemetry();
            drive.update();
        }
    }
}
