package org.firstinspires.ftc.teamcode.tele.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.caching.CachingServo;
import org.firstinspires.ftc.teamcode.helper.Hardware;
import org.firstinspires.ftc.teamcode.helper.OpModeEx;

@Config
@Disabled
@TeleOp(name="Deposit Tuning", group="Tuning")
public class DepositTuning extends OpModeEx {

    public static double intakePosition = 0.5;
    public static double depositPosition = 0.5;
    GamepadEx gamepadEx1;
    Servo deposit;

    @Override
    public void init() {
        deposit = new CachingServo(hardwareMap.get(Servo.class, Hardware.DEPOSIT_SERVO_RIGHT));
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx1.readButtons();
    }

    @Override
    public void loop() {
        gamepadEx1.readButtons();

        if (gamepadEx1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            deposit.setPosition(intakePosition);
        } else if (gamepadEx1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            deposit.setPosition(depositPosition);
        }
    }
}
