package org.firstinspires.ftc.teamcode.tele.subsystems;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.helper.Hardware;

public class DepositColorSensor {

    private final ColorRangeSensor sensor;
    private int numDetections = 0;
    private boolean wasJustDetected = false;
    private boolean isDetected = false;
    private final double DETECTED_THRESHOLD = 1.0;
    private final double TIME_THRESHOLD = 0.5;
    private final ElapsedTime time = new ElapsedTime();

    public DepositColorSensor(HardwareMap hardwareMap) {
        sensor = hardwareMap.get(ColorRangeSensor.class, Hardware.COLOR_RANGE_SENSOR);
        sensor.enableLed(true);
        time.reset();
    }

    public void read() {
        isDetected = sensor.getDistance(DistanceUnit.INCH) > DETECTED_THRESHOLD;

        if (isDetected && !wasJustDetected) {
            numDetections += 1;
        } else if (isDetected && time.seconds() > TIME_THRESHOLD) {
            numDetections += 1;
        }

//        if (numDetections > 2) {
//            numDetections %= 2;
//        }

        if (!isDetected) {
            time.reset();
        }

        wasJustDetected = isDetected;
    }

    public int getNumDetections() {
        return numDetections;
    }
}
