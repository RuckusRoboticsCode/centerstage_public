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
//    private final ArrayList<Subsystem> subsystems;
    private final Telemetry telemetry;
    private final GamepadEx gamepadEx1;
    private final GamepadEx gamepadEx2;
//    private final boolean adjustForVoltage;
    private double totalCurrentDraw = 0.0;
//    private VoltageSensor voltageSensor= null;
//    private VisionControllerTele visionControllerTele = null;
//    private ElapsedTime timer;

    private Deposit deposit = null;
    private DepositRelease depositRelease = null;
    private Drivetrain drivetrain = null;
    private DroneLauncher droneLauncher = null;
    private Intake intake = null;
    private LinearSlides linearSlides = null;
    private DepositColorSensor depositColorSensor = null;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry,
                 GamepadEx gamepadEx1, GamepadEx gamepadEx2, boolean adjustForVoltage) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepadEx1 = gamepadEx1;
        this.gamepadEx2 = gamepadEx2;
//        this.adjustForVoltage = adjustForVoltage;
//        this.timer = new ElapsedTime();

//        if(adjustForVoltage) {
//            voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
//        }
    }

    protected void setDeposit(Deposit deposit) {
        deposit.setGamepadEx(gamepadEx2);
        this.deposit = deposit;
//        subsystems.add(deposit);
    }

    protected void setDrivetrain(Drivetrain drivetrain) {
        drivetrain.setGamepadEx1(gamepadEx1);
        drivetrain.setUseSmoothing(true);
        this.drivetrain = drivetrain;
//        subsystems.add(drivetrain);
    }

    protected void setDroneLauncher(DroneLauncher droneLauncher) {
        droneLauncher.setGamepadEx(gamepadEx2);
        this.droneLauncher = droneLauncher;
//        subsystems.add(droneLauncher);
    }

    protected void setIntake(Intake intake) {
        intake.setGamepadEx(gamepadEx1);
        this.intake = intake;
//        subsystems.add(intake);
    }

    protected void setLinearSlides(LinearSlides linearSlides) {
        linearSlides.setGamepadEx(gamepadEx2);
//        linearSlides.setMaxROC(1.25);
        this.linearSlides = linearSlides;
//        subsystems.add(linearSlides);
    }

    protected void setDepositRelease(DepositRelease depositRelease) {
        depositRelease.setGamepadEx(gamepadEx2);
        this.depositRelease = depositRelease;
//        subsystems.add(depositRelease);
    }

    protected void setDepositColorSensor(DepositColorSensor depositColorSensor) {
        this.depositColorSensor = depositColorSensor;
    }

//    public void setAprilTag(VisionControllerTele controller) {
//        visionControllerTele = controller;
//    }

    public void init() {
        gamepadEx1.readButtons();
        gamepadEx2.readButtons();

        deposit.init(hardwareMap, telemetry);
        depositRelease.init(hardwareMap, telemetry);
        drivetrain.init(hardwareMap, telemetry);
        droneLauncher.init(hardwareMap, telemetry);
        intake.init(hardwareMap, telemetry);
        linearSlides.init(hardwareMap, telemetry);
    }

    public void update() {
        gamepadEx1.readButtons();
        gamepadEx2.readButtons();

//        depositColorSensor.read();

        deposit.read();
        deposit.update(0.0 );

        depositRelease.read();
        depositRelease.update(0.0 );

        drivetrain.read();
        drivetrain.update(0.0 );

        droneLauncher.read();
        droneLauncher.update(0.0 );

        intake.read();
        intake.update(0.0 );

        linearSlides.read();
        linearSlides.update(0.0);
    }

    public double getTotalCurrentDraw() {
        return this.totalCurrentDraw;
    }
}
