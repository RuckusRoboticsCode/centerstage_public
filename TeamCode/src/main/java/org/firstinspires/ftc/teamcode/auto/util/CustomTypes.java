package org.firstinspires.ftc.teamcode.auto.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class CustomTypes {

	public static final double ROBOT_WIDTH = 14.0;
	public static final double ROBOT_LENGTH = 12.0;
	public static final double ALLOWABLE_HEADING_ERROR = 3.0;

	public enum PropLocation {
		LEFT("left"),
		MIDDLE("middle"),
		RIGHT("right");

		public final String location;

		PropLocation(String location) {
			this.location = location;
		}

		public String getLocation() {
			return this.location;
		}
	}

	public enum AutonomousStates {
		INIT,
		START,
		DRIVE_TO_SPIKE_MARK,
		DELAY,
		DRIVE_TO_BACKDROP,
		SCORE_ON_BACKDROP,
		DROP_PIXEL,
		LIFT_SLIDES_AFTER_SCORE,
		DRIVE_FORWARD_FROM_BACKDROP,
		RELOCALIZE,
		DRIVE_TO_CYCLE_START_AND_STACK,
		INTAKE,
		DRIVE_THROUGH_TRUSS_TO_BACKDROP,
		PARK,
		STOP;
	}
}