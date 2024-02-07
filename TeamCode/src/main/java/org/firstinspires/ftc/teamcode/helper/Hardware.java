package org.firstinspires.ftc.teamcode.helper;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class Hardware {
    public static String DRIVETRAIN_FL = "FL";
    public static String DRIVETRAIN_FR = "FR";
    public static String DRIVETRAIN_BL = "BL";
    public static String DRIVETRAIN_BR = "BR";

    public static String LINEAR_SLIDE_LEFT = "slideLeft";
    public static String LINEAR_SLIDE_RIGHT = "slideRight";

    public static String DEPOSIT_SERVO_LEFT = "leftDeposit";
    public static String DEPOSIT_SERVO_RIGHT = "rightDeposit";
    public static String DEPOSIT_RELEASE_SERVO = "depositRelease";

    public static String DRONE_LAUNCHER = "drone";

    public static String INTAKE = "intake";

    public static String IMU = "imu";

    public static RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.UP;
}
