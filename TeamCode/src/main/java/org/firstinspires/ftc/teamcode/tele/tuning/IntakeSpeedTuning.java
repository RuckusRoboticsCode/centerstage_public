package org.firstinspires.ftc.teamcode.tele.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.helper.Hardware;
import org.firstinspires.ftc.teamcode.helper.OpModeEx;

@Config
@Disabled
@TeleOp(name="Intake Speed Tuning", group="Tuning")
public class IntakeSpeedTuning extends OpModeEx {

    public static double intakePower = 1.0;
    public static double outtakePower = -1.0;
    DcMotorEx intake;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, Hardware.INTAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper) {
            intake.setPower(intakePower);
        } else if (gamepad1.right_bumper) {
            intake.setPower(outtakePower);
        } else {
            intake.setPower(0);
        }
    }
}
