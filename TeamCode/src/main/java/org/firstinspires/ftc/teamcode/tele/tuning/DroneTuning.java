package org.firstinspires.ftc.teamcode.tele.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.helper.OpModeEx;

@Config
@Disabled
@TeleOp(name="Drone Tuning", group="Tuning")
public class DroneTuning extends OpModeEx {

    public static double shootPosition = 0.5;
    public static double loadPosition = 0.5;
    GamepadEx gamepadEx1;
    Servo drone;

    @Override
    public void init() {
        drone = hardwareMap.get(Servo.class, "drone");
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx1.readButtons();
    }

    @Override
    public void loop() {
        gamepadEx1.readButtons();

        if (gamepadEx1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            drone.setPosition(loadPosition);
        } else if (gamepadEx1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            drone.setPosition(shootPosition);
        }

    }
}
