package org.firstinspires.ftc.teamcode.tele.testSubsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helper.OpModeEx;
import org.firstinspires.ftc.teamcode.tele.subsystems.Robot;
import org.firstinspires.ftc.teamcode.tele.subsystems.RobotBuilder;

@Disabled
@TeleOp(name="Drivetrain Test", group="Tests")
public class DrivetrainTest extends OpModeEx {

    GamepadEx gamepadEx1;
    GamepadEx gamepadEx2;
    Robot robot;

    @Override
    public void init() {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        robot = new RobotBuilder(hardwareMap, telemetry, gamepadEx1, gamepadEx2, false)
                .addDrivetrain()
                .build();
        robot.init();
    }

    @Override
    public void loop() {
        resetRuntime();
        robot.update();
        int hertz = (int)(1.0 / getRuntime());
        telemetry.addData("Loop Frequency", "%d", hertz);
    }
}
