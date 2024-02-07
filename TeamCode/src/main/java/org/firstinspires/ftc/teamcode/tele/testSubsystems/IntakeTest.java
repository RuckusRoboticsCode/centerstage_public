package org.firstinspires.ftc.teamcode.tele.testSubsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helper.OpModeEx;
import org.firstinspires.ftc.teamcode.tele.subsystems.Robot;
import org.firstinspires.ftc.teamcode.tele.subsystems.RobotBuilder;

@Disabled
@TeleOp(name="Intake Test", group="Tests")
public class IntakeTest extends OpModeEx {

    GamepadEx gamepadEx1;
    GamepadEx gamepadEx2;
    Robot robot;

    @Override
    public void init() {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        robot = new RobotBuilder(hardwareMap, telemetry, gamepadEx1, gamepadEx2, false)
                .addIntake()
                .build();
        robot.init();
    }

    @Override
    public void loop() {
        robot.update();
    }
}
