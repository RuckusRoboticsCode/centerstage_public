package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helper.OpModeEx;
import org.firstinspires.ftc.teamcode.tele.subsystems.Robot;
import org.firstinspires.ftc.teamcode.tele.subsystems.RobotBuilder;

import java.util.List;

//@Disabled
//@Photon
@TeleOp(name="Tele", group="Tele")
public class Tele extends OpModeEx {

	private Robot robot;

	@Override
	public void init() {

		List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

		for (LynxModule hub : allHubs) {
			hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
		}

		FtcDashboard dashboard = FtcDashboard.getInstance();
		telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
		GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
		GamepadEx gamepadEx2 = new GamepadEx(gamepad2);
		gamepadEx1.readButtons();
		gamepadEx2.readButtons();
//		telemetry.setMsTransmissionInterval(10);

		robot = new RobotBuilder(hardwareMap, telemetry, gamepadEx1, gamepadEx2, false)
				.addDrivetrain()
				.addLinearSlides()
				.addIntake()
				.addDeposit()
				.addDepositRelease()
//				.addAprilTag()
				.addDroneLauncher()
				.build();

		robot.init();
		resetRuntime();
	}

	@Override
	public void loop() {
		resetRuntime();
		robot.update();
		int hertz = (int)(1.0 / getRuntime());
		telemetry.addData("Loop Frequency", "%d", hertz);
	}
}
