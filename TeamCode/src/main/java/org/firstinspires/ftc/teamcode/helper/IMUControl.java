package org.firstinspires.ftc.teamcode.helper;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.helper.Hardware;

public class IMUControl {

    private final Telemetry telemetry;
    private final IMU imu;
    private double yawOffset;

    public IMUControl(HardwareMap hardwareMap, Telemetry telemetry, double startHeadingRads) {
        this.telemetry = telemetry;
        this.imu = hardwareMap.get(IMU.class, Hardware.IMU);
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                Hardware.IMU_LOGO_DIRECTION,
                Hardware.IMU_USB_DIRECTION
        ));
        imu.initialize(parameters);
        imu.resetYaw();
        this.yawOffset = startHeadingRads;
    }

    public double getHeading() {
        double heading = angleWrapper(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + yawOffset);
//        telemetry.addData("Heading", Math.toDegrees(heading));
        return heading;
    }

    private double angleWrapper(double radians) {
        while (radians < 0) {
            radians += 2 * Math.PI;
        }
        radians %= (2 * Math.PI);
//        while (radians > Math.PI) {
//            radians -= 2 * Math.PI;
//        }
//        while (radians < -Math.PI) {
//            radians += 2 * Math.PI;
//        }
        return radians;
    }

    public void setYawOffset(double yawOffset) {
        this.yawOffset = yawOffset;
    }
}
