package org.firstinspires.ftc.teamcode.auto.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class RoadRunnerLocations {

	public enum BlueLocations {

		CLOSE_STARTING(new Pose2d(15, 62, Math.toRadians(90))),
		FAR_STARTING(new Pose2d(-39, 62, Math.toRadians(90))),

		CLOSE_RIGHT_SPIKE(new Pose2d(5.5, 41)),
		CLOSE_MIDDLE_SPIKE(new Pose2d(12, 34)),
		CLOSE_LEFT_SPIKE(new Pose2d(24, 41)),

		FAR_RIGHT_SPIKE(new Pose2d(-47, 32)),
		FAR_MIDDLE_SPIKE(new Pose2d(-35, 33)),
		FAR_LEFT_SPIKE(new Pose2d(-28.5, 41)),

		CYCLE_START(new Pose2d(36, 11.5, Math.toRadians(180))),
		STACK(new Pose2d(-61, 11.5)),

		BACKDROP_RIGHT(new Pose2d(51.75, 29.5)),
		BACKDROP_MIDDLE(new Pose2d(51.75, 37)),
		BACKDROP_LEFT(new Pose2d(51.75, 41));

		public final Pose2d pose;

		BlueLocations(Pose2d pose) {
			this.pose = pose;
		}

		public Vector2d getPosition() {
			return this.pose.vec();
		}

		public Pose2d getPose() {
			return this.pose;
		}
	}

	public enum RedLocations {

		CLOSE_STARTING(new Pose2d(15, -62, Math.toRadians(270))),
		FAR_STARTING(new Pose2d(-39, -62, Math.toRadians(270))),

		CLOSE_LEFT_SPIKE(new Pose2d(5.5, -41)),
		CLOSE_MIDDLE_SPIKE(new Pose2d(12, -34)),
		CLOSE_RIGHT_SPIKE(new Pose2d(24, -41)),

		FAR_LEFT_SPIKE(new Pose2d(-46, -41)),
		FAR_MIDDLE_SPIKE(new Pose2d(-36, -34)),
		FAR_RIGHT_SPIKE(new Pose2d(-28.5, -41)),

		CYCLE_START(new Pose2d(36, -11.5, Math.toRadians(180))),
		STACK(new Pose2d(-61, -11.5)),

		BACKDROP_LEFT(new Pose2d(51.75, -29.5)),
		BACKDROP_MIDDLE(new Pose2d(51.75, -37.5)),
		BACKDROP_RIGHT(new Pose2d(51.75, -43.5));

		public final Pose2d pose;

		RedLocations(Pose2d pose) {
			this.pose = pose;
		}

		public Vector2d getPosition() {
			return this.pose.vec();
		}

		public Pose2d getPose() {return this.pose;}
	}
}