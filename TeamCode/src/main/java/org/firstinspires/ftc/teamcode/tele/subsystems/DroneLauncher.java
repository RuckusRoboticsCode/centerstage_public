package org.firstinspires.ftc.teamcode.tele.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.caching.CachingServo;
import org.firstinspires.ftc.teamcode.helper.Hardware;

public class DroneLauncher implements Subsystem {

    private GamepadEx gamepadEx;
    private Servo droneServo;
    private Telemetry telemetry;
    private static final double loaded = 0.45;
    private static final double launch = 0.85;
    private boolean atLaunch = false;
    private boolean isRunning = false;
    private static double MAX_CURRENT_DRAW = 0.0;

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.droneServo = new CachingServo(hardwareMap.get(Servo.class, Hardware.DRONE_LAUNCHER), 0.01);
        this.telemetry = telemetry;
        this.atLaunch = false;
        load();
    }

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry, boolean adjustForVoltage) {
        init(hardwareMap, telemetry);
    }

    public void launch() {
        droneServo.setPosition(launch);
    }

    public void load() {
        droneServo.setPosition(loaded);
    }

    public void setGamepadEx(GamepadEx gamepadEx) {
        this.gamepadEx = gamepadEx;
    }

    @Override
    public void updateNoVoltage() {
        if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) ||
        gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_LEFT) ||
        gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_UP) ||
        gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            if(atLaunch) {
                load();
            } else {
                launch();
            }
            atLaunch = !atLaunch;
        }
    }

    @Override
    public void updateWithVoltage(double startingVoltage) {
        updateNoVoltage();
    }

    @Override
    public void update(double startingVoltage) {
        updateNoVoltage();
//        updateTelemetry();
    }

    @Override
    public void updateTelemetry() {
        telemetry.addData("Drone Launcher", " ");
        telemetry.addData("At Loaded Position", !atLaunch);
        telemetry.addData("Drone Servo Position", droneServo.getPosition());
    }

    @Override
    public boolean isRunning() {
        return isRunning;
    }

    @Override
    public double getMaxCurrent() {
        return MAX_CURRENT_DRAW;
    }

    @Override
    public void stop() {
        isRunning = false;
    }
}
