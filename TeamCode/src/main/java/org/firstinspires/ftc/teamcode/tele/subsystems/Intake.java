package org.firstinspires.ftc.teamcode.tele.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.caching.CachingDcMotorEX;
import org.firstinspires.ftc.teamcode.helper.Hardware;

public class Intake implements Subsystem {

    private DcMotorEx intake;
    private GamepadEx gamepadEx;
    private Telemetry telemetry;

    private static final double MAX_CURRENT_DRAW = 0.0;
    private boolean adjustForVoltage = false;
    private boolean isRunning;

    private double power = 0.0;

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.intake = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, Hardware.INTAKE));
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.telemetry = telemetry;
    }

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry, boolean adjustForVoltage) {
        init(hardwareMap, telemetry);
        this.adjustForVoltage = adjustForVoltage;
    }

    @Override
    public void updateNoVoltage() {
        if(gamepadEx.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            power = 1.0;
            this.isRunning = true;
        } else if (gamepadEx.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            power = -1.0;
            this.isRunning = true;
        } else {
            power = 0.0;
            this.isRunning = false;
        }
        intake.setPower(power);
    }

    @Override
    public void updateWithVoltage(double startingVoltage) {
        if(gamepadEx.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            power = 1.1 - (startingVoltage / 28.0);
            this.isRunning = true;
        } else if(gamepadEx.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            power = (startingVoltage / 50.0) - 1.1;
            this.isRunning = true;
        } else {
            power = 0.0;
            this.isRunning = false;
        }
        intake.setPower(power);
    }

    @Override
    public void update(double startingVoltage) {
        if(adjustForVoltage) {
            updateWithVoltage(startingVoltage);
            return;
        }
        updateNoVoltage();
//        updateTelemetry();
    }

    public void setGamepadEx(GamepadEx gamepadEx) {
        this.gamepadEx = gamepadEx;
    }

    @Override
    public void updateTelemetry() {
        telemetry.addData("Intake", " ");
        telemetry.addData("Intake Power", "%.2f", Range.clip(power, -1, 1));
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
        power = 0.0;
        intake.setPower(power);
        isRunning = false;
    }
}
