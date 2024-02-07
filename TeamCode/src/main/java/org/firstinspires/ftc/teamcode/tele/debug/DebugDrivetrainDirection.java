package org.firstinspires.ftc.teamcode.tele.debug;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.helper.Hardware;
import org.firstinspires.ftc.teamcode.helper.OpModeEx;

@Disabled
@TeleOp(name="Drivetrain Direction Debug", group="Debug")
public class DebugDrivetrainDirection extends OpModeEx {

    DcMotorEx FL, FR, BL, BR;
    double powerFL, powerFR, powerBL, powerBR;

    @Override
    public void init() {
        FL = hardwareMap.get(DcMotorEx.class, Hardware.DRIVETRAIN_FL);
        FR = hardwareMap.get(DcMotorEx.class, Hardware.DRIVETRAIN_FR);
        BL = hardwareMap.get(DcMotorEx.class, Hardware.DRIVETRAIN_BL);
        BR = hardwareMap.get(DcMotorEx.class, Hardware.DRIVETRAIN_BR);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        powerFL = gamepad1.x ? 0 : 0.25;
        powerFR = gamepad1.y ? 0 : 0.25;
        powerBL = gamepad1.a ? 0 : 0.25;
        powerBR = gamepad1.b ? 0 : 0.25;

        FL.setPower(powerFL);
        FR.setPower(powerFR);
        BL.setPower(powerBL);
        BR.setPower(powerBR);
    }
}
