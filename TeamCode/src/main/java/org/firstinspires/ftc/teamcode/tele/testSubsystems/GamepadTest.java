package org.firstinspires.ftc.teamcode.tele.testSubsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helper.OpModeEx;

@Disabled
@TeleOp(name="Gamepad Test", group="Tests")
public class GamepadTest extends OpModeEx {

    GamepadEx gamepadEx1;
    GamepadEx gamepadEx2;

    @Override
    public void init() {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
    }

    @Override
    public void loop() {

        telemetry.addLine("Gamepad 1")
                .addData("Gamepad 1 Left Y", "%.2f", gamepadEx1.getLeftY())
                .addData("Gamepad 1 Left X", "%.2f", gamepadEx1.getLeftX())
                .addData("Gamepad 1 Right X", "%.2f", gamepadEx1.getRightX());

        telemetry.addLine("Gamepad 2")
                .addData("Gamepad 2 Left Y", "%.2f", gamepadEx2.getLeftY())
                .addData("Gamepad 2 Left X", "%.2f", gamepadEx2.getLeftX())
                .addData("Gamepad 2 Right X", "%.2f", gamepadEx2.getRightX());
    }
}
