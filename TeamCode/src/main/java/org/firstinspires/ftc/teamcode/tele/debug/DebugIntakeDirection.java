package org.firstinspires.ftc.teamcode.tele.debug;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.helper.Hardware;
import org.firstinspires.ftc.teamcode.helper.OpModeEx;

@Disabled
@TeleOp(name="Debug Intake Direction", group="Debug")
public class DebugIntakeDirection extends OpModeEx {

    DcMotorEx intake;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, Hardware.INTAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper) {
            intake.setPower(0.25);
        } else {
            intake.setPower(0.0);
        }
    }
}
