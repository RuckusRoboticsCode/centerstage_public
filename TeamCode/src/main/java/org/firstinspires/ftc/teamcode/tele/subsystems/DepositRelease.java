package org.firstinspires.ftc.teamcode.tele.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.caching.CachingServo;
import org.firstinspires.ftc.teamcode.helper.Hardware;

public class DepositRelease implements Subsystem {

    private GamepadEx gamepadEx2 = null;
    private Telemetry telemetry;

    private Servo depositRelease;
    private static final double BLOCKING_POSITION = 0.65;
    private static final double RELEASE_POSITION = 0.49;
    private static final double MAX_CURRENT_DRAW = 0.0;

    private boolean atBlocking = true;
    private boolean isRunning = false;

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        depositRelease = new CachingServo(hardwareMap.get(Servo.class, Hardware.DEPOSIT_RELEASE_SERVO));
        this.telemetry = telemetry;
        block();
        this.isRunning = false;
        this.atBlocking = true;
    }

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry, boolean adjustForVoltage) {
        init(hardwareMap, telemetry);
    }

    public void setGamepadEx(GamepadEx gamepadEx) {
        this.gamepadEx2 = gamepadEx;
    }

    public void block() {
        depositRelease.setPosition(BLOCKING_POSITION);
    }

    public void release() {
        depositRelease.setPosition(RELEASE_POSITION);
    }

    @Override
    public void updateNoVoltage() {
        if (gamepadEx2 == null) {
            return;
        }
        if (gamepadEx2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            if (atBlocking) {
                release();
            } else {
                block();
            }
            atBlocking = !atBlocking;
        }
    }

    @Override
    public void updateWithVoltage(double startingVoltage) {
        updateNoVoltage();
    }

    @Override
    public void update(double startingVoltage) {
        updateNoVoltage();
        if (!atBlocking) {
            gamepadEx2.gamepad.rumble(50);
        } else {
            gamepadEx2.gamepad.stopRumble();
        }
    }

    @Override
    public void updateTelemetry() {

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

    @Override
    public String getName() {
        return "depositRelease";
    }
}
