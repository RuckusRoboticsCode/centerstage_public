package org.firstinspires.ftc.teamcode.tele.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tele.vision.VisionControllerTele;

public class RobotBuilder {

    private final Robot robot;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    public RobotBuilder(HardwareMap hardwareMap, Telemetry telemetry,
                        GamepadEx gamepadEx1, GamepadEx gamepadEx2, boolean adjustForVoltage) {
        this.robot = new Robot(hardwareMap, telemetry, gamepadEx1, gamepadEx2, adjustForVoltage);
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    public RobotBuilder addDeposit() {
        this.robot.setDeposit(
                new Deposit()
        );
        return this;
    }

    public RobotBuilder addDrivetrain() {
        this.robot.setDrivetrain(
                new Drivetrain()
        );
        return this;
    }

    public RobotBuilder addDroneLauncher() {
        this.robot.setDroneLauncher(
                new DroneLauncher()
        );
        return this;
    }

    public RobotBuilder addIntake() {
        this.robot.setIntake(
                new Intake()
        );
        return this;
    }

    public RobotBuilder addLinearSlides() {
        this.robot.setLinearSlides(
                new LinearSlides()
        );
        return this;
    }

    public RobotBuilder addDepositRelease() {
        this.robot.setDepositRelease(
                new DepositRelease()
        );
        return this;
    }

    public RobotBuilder addAprilTag() {
        this.robot.setAprilTag(
                new VisionControllerTele(hardwareMap, telemetry)
        );
        return this;
    }

    public Robot build() {
        return this.robot;
    }
}
