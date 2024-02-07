package org.firstinspires.ftc.teamcode.tele.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.caching.CachingServo;
import org.firstinspires.ftc.teamcode.helper.Hardware;

public class Deposit implements Subsystem {

    private Servo leftDeposit;
    private Servo rightDeposit;
    private GamepadEx gamepadEx2 = null;
    private Telemetry telemetry;

    private static final double LEFT_INPUT = 0.23;
    private static final double RIGHT_INPUT = 0.71;
    private static final double LEFT_OUTPUT = 0.74;
    private static final double RIGHT_OUTPUT = 0.18;
    private static final double MAX_CURRENT_DRAW = 0.0;

    private boolean atIntake;
    private boolean isRunning;

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.leftDeposit = new CachingServo(hardwareMap.get(Servo.class, Hardware.DEPOSIT_SERVO_LEFT));
        this.rightDeposit = new CachingServo(hardwareMap.get(Servo.class, Hardware.DEPOSIT_SERVO_RIGHT));
        this.telemetry = telemetry;
        this.atIntake = true;
        this.isRunning = true;

        intake();
    }

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry, boolean adjustForVoltage) {
        init(hardwareMap, telemetry);
    }

    public void setGamepadEx(GamepadEx gamepadEx) {
        this.gamepadEx2 = gamepadEx;
    }

    public void intake() {
        leftDeposit.setPosition(LEFT_INPUT);
        rightDeposit.setPosition(RIGHT_INPUT);
    }

    public void deposit() {
        leftDeposit.setPosition(LEFT_OUTPUT);
        rightDeposit.setPosition(RIGHT_OUTPUT);
    }

    @Override
    public void updateNoVoltage() {
        if(gamepadEx2 == null) {
//            if(atIntake) {
//                deposit();
//                atIntake = !atIntake;
//                return;
//            }
//            intake();
//            atIntake = !atIntake;
            return;
        }
        if(gamepadEx2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            if (atIntake) {
                deposit();
            } else {
                intake();
            }
            atIntake = !atIntake;
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
        telemetry.addData("Deposit", " ");
        telemetry.addData("Left | Right Target", "%.2f | %.2f", leftDeposit.getPosition(), rightDeposit.getPosition());
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
