package org.firstinspires.ftc.teamcode.tele.testSubsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helper.OpModeEx;
import org.firstinspires.ftc.teamcode.tele.subsystems.DepositColorSensor;

@TeleOp(name="Deposit Color Sensor Test", group="Test")
public class DepositColorSensorTest extends OpModeEx {

    DepositColorSensor depositColorSensor;

    @Override
    public void init() {
        depositColorSensor = new DepositColorSensor(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        depositColorSensor.read();
        telemetry.addData("Num", depositColorSensor.getNumDetections());
    }
}
