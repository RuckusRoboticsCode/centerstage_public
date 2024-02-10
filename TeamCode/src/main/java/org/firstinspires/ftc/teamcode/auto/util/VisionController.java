package org.firstinspires.ftc.teamcode.auto.util;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.util.CustomTypes.PropLocation;
import org.firstinspires.ftc.teamcode.helper.Constants;
import org.firstinspires.ftc.teamcode.helper.CustomAprilTagLibrary;
import org.firstinspires.ftc.teamcode.vision.PropDetection.HSV.LeftPropProcessor;
import org.firstinspires.ftc.teamcode.vision.PropDetection.HSV.RightPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class VisionController {
	private final VisionPortal visionPortal;
	private final RightPropProcessor rightPropProcessor;
	private final LeftPropProcessor leftPropProcessor;
	private final CustomAprilTagProcessor aprilTagProcessor;
	private final Telemetry telemetry;

	public VisionController(HardwareMap hardwareMap, boolean isDetectingAprilTags,
							boolean detectRed, boolean detectRight,
							Telemetry telemetry, FtcDashboard ftcDashboard) {

		aprilTagProcessor = new CustomAprilTagProcessor.Builder()
				.setDrawAxes(true)
				.setDrawCubeProjection(true)
				.setDrawTagOutline(true)
				.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
				.setTagLibrary(CustomAprilTagLibrary.getCenterStageTagLibrary())
				.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
				.setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)
				.build();

//		aprilTagProcessor = new AprilTagProcessor.Builder()
//				.setDrawAxes(true)
//				.setDrawCubeProjection(true)
//				.setDrawTagOutline(true)
//				.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//				.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
//				.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
//				.setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)
//				.build();

		aprilTagProcessor.setDecimation(1);

		if(detectRight) {
			leftPropProcessor = null;
			rightPropProcessor = new RightPropProcessor();
			rightPropProcessor.setTelemetry(telemetry);
			rightPropProcessor.setDetectionColor(detectRed);
			this.visionPortal = new VisionPortal.Builder()
					.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
					.setCameraResolution(new Size(640, 480))
					.addProcessors(rightPropProcessor, aprilTagProcessor)
					.build();

			if (ftcDashboard != null) {
				if (isDetectingAprilTags) {
					ftcDashboard.startCameraStream(aprilTagProcessor, 0);
				} else {
					ftcDashboard.startCameraStream(rightPropProcessor, 0);
				}
			}

			visionPortal.setProcessorEnabled(rightPropProcessor, !isDetectingAprilTags);
			visionPortal.setProcessorEnabled(aprilTagProcessor, isDetectingAprilTags);
		} else {
			rightPropProcessor = null;
			leftPropProcessor = new LeftPropProcessor();
			leftPropProcessor.setTelemetry(telemetry);
			leftPropProcessor.setDetectionColor(detectRed);
			this.visionPortal = new VisionPortal.Builder()
					.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
					.setCameraResolution(new Size(640, 480))
					.addProcessors(leftPropProcessor, aprilTagProcessor)
					.build();

			if (ftcDashboard != null) {
				if (isDetectingAprilTags) {
					ftcDashboard.startCameraStream(aprilTagProcessor, 0);
				} else {
					ftcDashboard.startCameraStream(leftPropProcessor, 0);
				}
			}

			visionPortal.setProcessorEnabled(leftPropProcessor, !isDetectingAprilTags);
			visionPortal.setProcessorEnabled(aprilTagProcessor, isDetectingAprilTags);
		}

		this.telemetry = telemetry;
	}

	public VisionController(HardwareMap hardwareMap, boolean isDetectingAprilTags,
							boolean detectRed, boolean detectRight, Telemetry telemetry) {
		this(hardwareMap, isDetectingAprilTags, detectRed, detectRight, telemetry, null);
	}

	public PropLocation getLocationOnce() {
		if(leftPropProcessor != null) {
			return leftPropProcessor.getPropLocation();
		}
		return rightPropProcessor.getPropLocation();
	}

	public void updateTelemetry() {
		double[] percs;
		if(leftPropProcessor != null) {
			percs = leftPropProcessor.getPercents();
			telemetry.addData("Left", percs[0]);
		} else {
			percs = rightPropProcessor.getPercents();
			telemetry.addData("Right", percs[0]);
		}
		telemetry.addData("Middle", percs[1]);
	}

	public PropLocation getPropLocation() {
		int leftCount = 0;
		int middleCount = 0;
		int rightCount = 0;
		PropLocation pos;
		for(int i = 0; i < 50; i++) {
			if(leftPropProcessor != null) {
				pos = leftPropProcessor.getPropLocation();
			} else {
				pos = rightPropProcessor.getPropLocation();
			}
			switch (pos) {
				case LEFT:
					leftCount++;
					break;
				case MIDDLE:
					middleCount++;
					break;
				default:
					rightCount++;
					break;
			}
		}

		telemetry.addData("Left Count", leftCount);
		telemetry.addData("Middle Count", middleCount);
		telemetry.addData("Right Count", rightCount);

		if (leftCount > middleCount && leftCount > rightCount) {
			return PropLocation.LEFT;
		} else if(middleCount > leftCount && middleCount > rightCount) {
			return PropLocation.MIDDLE;
		} else {
			return PropLocation.RIGHT;
		}
	}

	public ArrayList<Pose2d> getRaw() {
		ArrayList<AprilTagDetection> detections = aprilTagProcessor.getDetections();
		ArrayList<Pose2d> pose2ds = new ArrayList<>();

		for (AprilTagDetection detection : detections) {
			if(detection.id <= 6 && detection.id >= 0) {
				double relativeHeading = -detection.ftcPose.yaw;
				double xRelative = detection.ftcPose.x - Constants.cameraLateralOffset; // relative position (lateral)
				double yRelative = detection.ftcPose.y + Constants.cameraPerpendicularOffset; // relative position (forward)

				pose2ds.add(
						new Pose2d(yRelative, xRelative, relativeHeading)
				);
			}
		}

		return pose2ds;
	}

	public void enableAprilTags() {
		if(leftPropProcessor == null) {
			visionPortal.setProcessorEnabled(rightPropProcessor, false);
		} else {
			visionPortal.setProcessorEnabled(leftPropProcessor, false);
		}
		visionPortal.setProcessorEnabled(aprilTagProcessor, true);
	}

	public ArrayList<Pose2d> getPositions(double heading) {

		ArrayList<AprilTagDetection> detections = aprilTagProcessor.getDetections();
		ArrayList<Pose2d> poses = new ArrayList<>();

		for (AprilTagDetection detection : detections) {
			if (detection.metadata != null) {
				if (detection.id <= 6 && detection.id >= 0) {
//				double relativeHeading = Math.PI - heading;
					double botHeading = -heading;
					VectorF tagPosition = detection.metadata.fieldPosition; // absolute position
					double xRelative = detection.ftcPose.x - Constants.cameraLateralOffset; // relative position (lateral)
					double yRelative = detection.ftcPose.y - Constants.cameraPerpendicularOffset; // relative position (forward)

					// translate to FTC/RR co-ordinate system
//				double botHeading = -heading;
//				double xAbsolute = xRelative * Math.cos(botHeading) + yRelative * Math.sin(botHeading);
//				double yAbsolute = yRelative * Math.cos(botHeading) - xRelative * Math.sin(botHeading);
//				double xAbsolute = xRelative * Math.sin(heading) + yRelative * Math.cos(heading);
//				double yAbsolute = yRelative * Math.sin(heading) - xRelative * Math.cos(heading);
//				double xAbsolute = yRelative * Math.cos(relativeHeading) - xRelative * Math.sin(relativeHeading);
//				double yAbsolute = yRelative * Math.sin(relativeHeading) + xRelative * Math.cos(relativeHeading);
					double xAbsolute = xRelative * Math.cos(botHeading) + yRelative * Math.sin(botHeading);
					double yAbsolute = xRelative * -Math.sin(botHeading) + yRelative * Math.cos(botHeading);

					Pose2d pose = new Pose2d(
						/*
						perpendicular offset is front/back (front is positive)
						lateral offset is left/right (right is positive)
						 */
							tagPosition.get(0) + yAbsolute,
							tagPosition.get(1) - xAbsolute,
//						tagPosition.get(0) + xAbsolute,
//						tagPosition.get(1) - yAbsolute,
//						tagPosition.get(0) - yAbsolute,
//						tagPosition.get(1) + xAbsolute,
//						relativeHeading
//							Math.toDegrees(relativeHeading)
							heading
					);

					poses.add(pose);
				}
			}
		}
		return poses;
	}

	public Pose2d filterPoses(ArrayList<Pose2d> poses, Pose2d currentEstimate) {
		int count = 0;
		if(poses.size() == 0) {
			return currentEstimate;
		}
		Pose2d estimatedPose = new Pose2d(0, 0, 0);
		for(Pose2d pose : poses) {
			if(Math.abs(pose.getX()) > 72 || Math.abs(pose.getY()) > 72) {
				continue;
			}
			Pose2d error = pose.minus(currentEstimate);
			if(Math.abs(error.getX()) > 2 || Math.abs(error.getY()) > 2 || Math.abs(error.getHeading()) > Math.toRadians(15)) {
				continue;
			}
			estimatedPose = estimatedPose.plus(pose);
			count ++;
		}
		estimatedPose = estimatedPose.div(count);
		return estimatedPose;
	}

	public Pose2d relocalize(Pose2d currentEstimate, double imuHeading) {
		return filterPoses(getPositions(imuHeading), currentEstimate);
	}
}
