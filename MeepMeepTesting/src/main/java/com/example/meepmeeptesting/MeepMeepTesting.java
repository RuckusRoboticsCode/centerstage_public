package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
	public static void main(String[] args) {
		MeepMeep meepMeep = new MeepMeep(640);
		Pose2d starting = new Pose2d(16, -62, Math.toRadians(270));
		RoadRunnerBotEntity myBot = new RoadRunnerBotEntity(
				meepMeep,
//				new Constraints(60, 40, Math.toRadians(200), Math.toRadians(100), 21.02),
//				new Constraints(48, 32, Math.toRadians(150), Math.toRadians(80), 21.02),
				new Constraints(55, 35, Math.toRadians(240), Math.toRadians(60), 21.06),
				16.0,
				17.0,
				starting,
				new ColorSchemeBlueLight(),
				1,
				DriveTrainType.MECANUM,
				true
		);

		RoadRunnerBotEntity myBot2 = new RoadRunnerBotEntity(
				meepMeep,
//				new Constraints(60, 40, Math.toRadians(200), Math.toRadians(100), 21.02),
//				new Constraints(48, 32, Math.toRadians(150), Math.toRadians(80), 21.02),
				new Constraints(55, 35, Math.toRadians(240), Math.toRadians(60), 21.06),
				16.0,
				17.0,
				starting,
				new ColorSchemeBlueLight(),
				1,
				DriveTrainType.MECANUM,
				true
		);

		double CYCLE_OFFSET = 2.0;

//		Vector2d closeLeft = new Vector2d(6.5, -35);
		Pose2d startingRef = new Pose2d(-39, -62, Math.toRadians(270));

		myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-36, -62, Math.toRadians(270)))

//				.splineToSplineHeading(new Pose2d(-20, -40, Math.toRadians(45)), Math.toRadians(90))
						.strafeToLinearHeading(new Vector2d(-33, -42), Math.toRadians(225))
						.strafeToLinearHeading(new Vector2d(-29, -37), Math.toRadians(200))

				.build());

		myBot2.runAction(myBot2.getDrive().actionBuilder(new Pose2d(-36, 62, -Math.toRadians(270)))

				.splineToSplineHeading(new Pose2d(-20, 40, -Math.toRadians(45)), -Math.toRadians(90))

				.build());

//		myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-39, -62, Math.toRadians(270)))
//
//// spike mark
//				.strafeToLinearHeading(new Vector2d(-35, -43), Math.toRadians(-135))
//				.strafeToLinearHeading(new Vector2d(-28, -37), Math.toRadians(-120))
//
//				.waitSeconds(0.001)
//
//// truss
//				.strafeToConstantHeading(new Vector2d(-31, -37))
//				.splineToSplineHeading(new Pose2d(-34, -11.5, Math.toRadians(180)), Math.toRadians(90))
//				.strafeToConstantHeading(new Vector2d(30, -11.5))
//
//// backdrop
//				.splineToConstantHeading(new Vector2d(51.5, -41.5), Math.toRadians(0))
//
//				.waitSeconds(0.001)
//
//// parking
//				.strafeToConstantHeading(new Vector2d(43.5, -41.5))
//				.strafeToConstantHeading(new Vector2d(43.5, -12.5))
//				.strafeToConstantHeading(new Vector2d(54, -12.5))
//
//				// waits
//				.waitSeconds(1)
//
//				.build());

		meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
				.setDarkMode(true)
				.setBackgroundAlpha(0.95f)
				.addEntity(myBot)
				.addEntity(myBot2)
				.start();
	}
}