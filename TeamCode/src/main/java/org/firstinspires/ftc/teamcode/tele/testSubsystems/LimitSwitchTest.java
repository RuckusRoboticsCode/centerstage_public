package org.firstinspires.ftc.teamcode.tele.testSubsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.helper.OpModeEx;

@Disabled
@TeleOp(name="Limit Switch Test", group="Tests")
public class LimitSwitchTest extends OpModeEx {

    TouchSensor limitSwitch;
    FtcDashboard ftcDashboard;

    @Override
    public void init() {
        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, ftcDashboard.getTelemetry());
        limitSwitch = hardwareMap.get(TouchSensor.class, "limit");
    }

    @Override
    public void loop() {
        telemetry.addData("Value", limitSwitch.getValue());
        telemetry.addData("Pressed", limitSwitch.isPressed());
        telemetry.update();
    }
}
