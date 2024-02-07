package org.firstinspires.ftc.teamcode.auto.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.caching.CachingServo;
import org.firstinspires.ftc.teamcode.helper.Hardware;

public class DepositAuto {

    private final CachingServo leftServo;
    private final CachingServo rightServo;
    private final CachingServo depositRelease;
    private static final double LEFT_INPUT = 0.23;
    private static final double RIGHT_INPUT = 0.71;
    private static final double LEFT_OUTPUT = 0.74;
    private static final double RIGHT_OUTPUT = 0.18;
    private static final double BLOCKING_POSITION = 0.65;
    private static final double RELEASE_POSITION = 0.49;

    public DepositAuto(HardwareMap hardwareMap, Telemetry telemetry) {
        this.leftServo = new CachingServo(hardwareMap.get(Servo.class, Hardware.DEPOSIT_SERVO_LEFT));
        this.rightServo = new CachingServo(hardwareMap.get(Servo.class, Hardware.DEPOSIT_SERVO_RIGHT));
        this.depositRelease = new CachingServo(hardwareMap.get(Servo.class, Hardware.DEPOSIT_RELEASE_SERVO));
        block();
    }

    public void intake() {
        leftServo.setPosition(LEFT_INPUT);
        rightServo.setPosition(RIGHT_INPUT);
    }

    public void deposit() {
        leftServo.setPosition(LEFT_OUTPUT);
        rightServo.setPosition(RIGHT_OUTPUT);
    }

    public void release() {
        depositRelease.setPosition(RELEASE_POSITION);
    }

    public void block() {
        depositRelease.setPosition(BLOCKING_POSITION);
    }
}
