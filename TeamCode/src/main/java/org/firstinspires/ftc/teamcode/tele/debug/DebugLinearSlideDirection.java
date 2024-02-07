package org.firstinspires.ftc.teamcode.tele.debug;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.helper.Hardware;
import org.firstinspires.ftc.teamcode.helper.OpModeEx;

@Disabled
@TeleOp(name="Debug Linear Slide Direction", group="Debug")
public class DebugLinearSlideDirection extends OpModeEx {

    DcMotorEx slideLeft, slideRight;
    GamepadEx gamepadEx;

    @Override
    public void init() {
        slideLeft = hardwareMap.get(DcMotorEx.class, Hardware.LINEAR_SLIDE_LEFT);
        slideRight = hardwareMap.get(DcMotorEx.class, Hardware.LINEAR_SLIDE_RIGHT);

        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx.readButtons();
    }

    @Override
    public void loop() {
        gamepadEx.readButtons();

        if (gamepadEx.getLeftY() > 0.05) {
            slideRight.setPower(0.0);
            slideLeft.setPower(0.25);
        } else if (gamepadEx.getRightY() > 0.05) {
            slideRight.setPower(0.25);
            slideLeft.setPower(0.0);
        } else {
            slideRight.setPower(0);
            slideLeft.setPower(0);
        }
    }
}
