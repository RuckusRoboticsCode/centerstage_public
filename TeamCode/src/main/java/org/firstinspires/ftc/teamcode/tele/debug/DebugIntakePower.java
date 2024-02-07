package org.firstinspires.ftc.teamcode.tele.debug;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.helper.Hardware;
import org.firstinspires.ftc.teamcode.helper.OpModeEx;

@Disabled
@TeleOp(name="Debug Intake Power", group="Debug")
public class DebugIntakePower extends OpModeEx {

    DcMotorEx intake;
    double power = 0.0;
    GamepadEx gamepadEx;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, Hardware.INTAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx.readButtons();
    }

    @Override
    public void loop() {
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            power += 0.01;
        } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            power -= 0.01;
        }
        power = Range.clip(power, -1.0, 1.0);
        intake.setPower(power);
    }
}
