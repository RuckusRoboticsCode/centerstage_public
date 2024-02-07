package org.firstinspires.ftc.teamcode.tele.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tele.vision.VisionControllerTele;

import java.util.ArrayList;

public class Robot {

    private final HardwareMap hardwareMap;
    private final ArrayList<Subsystem> subsystems;
    private final Telemetry telemetry;
    private final GamepadEx gamepadEx1;
    private final GamepadEx gamepadEx2;
    private final boolean adjustForVoltage;
    private double totalCurrentDraw = 0.0;
    private VoltageSensor voltageSensor= null;
    private VisionControllerTele visionControllerTele = null;
    private ElapsedTime timer;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry,
                 GamepadEx gamepadEx1, GamepadEx gamepadEx2, boolean adjustForVoltage) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepadEx1 = gamepadEx1;
        this.gamepadEx2 = gamepadEx2;
        this.adjustForVoltage = adjustForVoltage;
        this.subsystems = new ArrayList<>();
        this.timer = new ElapsedTime();

        if(adjustForVoltage) {
            voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        }
    }

    protected void setDeposit(Deposit deposit) {
        deposit.setGamepadEx(gamepadEx2);
        subsystems.add(deposit);
    }

    protected void setDrivetrain(Drivetrain drivetrain) {
        drivetrain.setGamepadEx1(gamepadEx1);
        drivetrain.setUseSmoothing(true);
        subsystems.add(drivetrain);
    }

    protected void setDroneLauncher(DroneLauncher droneLauncher) {
        droneLauncher.setGamepadEx(gamepadEx2);
        subsystems.add(droneLauncher);
    }

    protected void setIntake(Intake intake) {
        intake.setGamepadEx(gamepadEx1);
        subsystems.add(intake);
    }

    protected void setLinearSlides(LinearSlides linearSlides) {
        linearSlides.setGamepadEx(gamepadEx2);
//        linearSlides.setMaxROC(1.25);
        subsystems.add(linearSlides);
    }

    protected void setDepositRelease(DepositRelease depositRelease) {
        depositRelease.setGamepadEx(gamepadEx2);
        subsystems.add(depositRelease);
    }

    public void setAprilTag(VisionControllerTele controller) {
        visionControllerTele = controller;
    }

    public void init() {
        gamepadEx1.readButtons();
        gamepadEx2.readButtons();
        if(adjustForVoltage) {
            for (Subsystem mechanism : subsystems) {
                mechanism.init(this.hardwareMap, this.telemetry, this.adjustForVoltage);
            }
        } else {
            for (Subsystem mechanism : subsystems) {
                mechanism.init(this.hardwareMap, this.telemetry);
            }
        }
    }

    public void update() {
        gamepadEx1.readButtons();
        gamepadEx2.readButtons();

        totalCurrentDraw = 0.0;
        double currentVoltage;
        if(adjustForVoltage) {
            currentVoltage = voltageSensor.getVoltage();
        } else {
            currentVoltage = 12.0;
        }
        for (Subsystem mechanism : subsystems) {
//            if (totalCurrentDraw > 13.0) {
//                mechanism.stop();
//            }
            totalCurrentDraw += mechanism.isRunning() ? mechanism.getMaxCurrent() : 0.0;

            if (visionControllerTele != null && mechanism.getName() == "dt") {
                visionControllerTele.updateDetections();
                if (visionControllerTele.isAtBackdrop()) {
                    mechanism.setMaxPower(0.5);
                    telemetry.addData("At Backdrop", "yes");
                    timer.reset();
                } else {
                    if (timer.seconds() > 0.5) {
                        telemetry.addData("At Backdrop", "no");
                        mechanism.setMaxPower(1.0);
                    }
                }
            }
            mechanism.read();
            mechanism.update(currentVoltage);
        }
//        telemetry.addData("Total Current Draw", "%.2f", totalCurrentDraw);
    }

    public double getTotalCurrentDraw() {
        return this.totalCurrentDraw;
    }
}
