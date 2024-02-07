package org.firstinspires.ftc.teamcode.tele.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Subsystem {
    default void read() {}

    public void init(HardwareMap hardwareMap, Telemetry telemetry);
    public void init(HardwareMap hardwareMap, Telemetry telemetry, boolean adjustForVoltage);
    void updateNoVoltage();
    void updateWithVoltage(double startingVoltage);
    public void update(double startingVoltage);
    public void updateTelemetry();

    public boolean isRunning();

    public double getMaxCurrent();
    public void stop();

    default String getName() {return "";}

    default void setMaxPower(double MAX_POWER) {return;}

}
