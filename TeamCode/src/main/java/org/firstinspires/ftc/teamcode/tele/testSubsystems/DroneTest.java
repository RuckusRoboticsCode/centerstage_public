package org.firstinspires.ftc.teamcode.tele.testSubsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.caching.CachingServo;
import org.firstinspires.ftc.teamcode.helper.Hardware;
import org.firstinspires.ftc.teamcode.helper.OpModeEx;
import org.firstinspires.ftc.teamcode.tele.subsystems.DroneLauncher;
import org.firstinspires.ftc.teamcode.tele.subsystems.Robot;
import org.firstinspires.ftc.teamcode.tele.subsystems.RobotBuilder;

@Disabled
@TeleOp(name="Drone Test", group="Tests")
public class DroneTest extends OpModeEx {

    Robot robot;

    @Override
    public void init() {
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);
        robot = new RobotBuilder(hardwareMap, telemetry, gamepadEx1, gamepadEx2, false)
                .addDroneLauncher()
                .build();
        robot.init();
    }

    @Override
    public void loop() {
        robot.update();
    }
}
