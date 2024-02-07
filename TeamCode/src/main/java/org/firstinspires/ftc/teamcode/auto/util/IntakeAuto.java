package org.firstinspires.ftc.teamcode.auto.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.caching.CachingDcMotorEX;
import org.firstinspires.ftc.teamcode.helper.Hardware;

public class IntakeAuto {

    private final DcMotorEx intakeMotor;

    public IntakeAuto(HardwareMap hardwareMap, Telemetry telemetry) {
        intakeMotor = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, Hardware.INTAKE));
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void intake() {
        intakeMotor.setPower(0.9);
    }

    public void outtake() {
        intakeMotor.setPower(-0.9);
    }

    public void stop() {
        intakeMotor.setPower(0.0);
    }
}
