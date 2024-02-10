package org.firstinspires.ftc.teamcode.tele.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.caching.CachingDcMotorEX;
import org.firstinspires.ftc.teamcode.helper.Constants;
import org.firstinspires.ftc.teamcode.helper.Hardware;
import org.firstinspires.ftc.teamcode.helper.IMUControl;
import org.firstinspires.ftc.teamcode.helper.PIDFController;

public class Drivetrain implements Subsystem {

    private DcMotorEx FL, FR , BL, BR;
    private GamepadEx gamepadEx1;
    private Telemetry telemetry;
    private boolean useSmoothing;
    private double MAX_POWER = 1.0;
    private static double targetAngleDeg = 180.0;
    private IMUControl imuControl;
    private PIDFController pidfController;

    private double powerFL, powerFR, powerBL, powerBR;

    private boolean isRunning;
    private boolean headingLock = false;
    private boolean adjustForVoltage = false;
    public static final double MAX_CURRENT_DRAW = 10.0;

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        FL = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, Hardware.DRIVETRAIN_FL));
        FR = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, Hardware.DRIVETRAIN_FR));
        BL = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, Hardware.DRIVETRAIN_BL));
        BR = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, Hardware.DRIVETRAIN_BR));

        configureMotors();

        this.telemetry = telemetry;
        this.isRunning = false;
        imuControl = new IMUControl(hardwareMap, telemetry, Math.toRadians(180));
        this.pidfController = new PIDFController(
                Constants.HeadingPIDF.kp.value,
                Constants.HeadingPIDF.ki.value,
                Constants.HeadingPIDF.kd.value);
        pidfController.setTargetTolerance(Math.toRadians(3));
        pidfController.setTargetPosition(Math.toRadians(targetAngleDeg));
    }

    private void configureMotors() {
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry, boolean adjustForVoltage) {
        init(hardwareMap, telemetry);
        this.adjustForVoltage = adjustForVoltage;
    }

    public void setUseSmoothing(boolean useSmoothing) {
        this.useSmoothing = useSmoothing;
    }

    public void setMaxPower(double MAX_POWER) {
        this.MAX_POWER = MAX_POWER;
    }

    public void setGamepadEx1(GamepadEx gamepadEx1) {
        this.gamepadEx1 = gamepadEx1;
    }

    @Override
    public void updateNoVoltage() {
        double leftY = gamepadEx1.getLeftY();
        double leftX = gamepadEx1.getLeftX();
        double rightX = gamepadEx1.getRightX();

        if (gamepadEx1.wasJustPressed(GamepadKeys.Button.Y)) {
            headingLock = !headingLock;
        }

        if (gamepadEx1.wasJustPressed(GamepadKeys.Button.X)) {
//            imuControl.setYawOffset(imuControl.getHeading());
            double heading = imuControl.getHeading();
            imuControl.setYawOffset((2 * Math.PI) - Math.toRadians(heading));
        }

        if (this.useSmoothing) {
            leftY = applySmoothPower(leftY);
            leftX = applySmoothPower(leftX);
            rightX = applySmoothPower(rightX);
        }
        leftX *= 1.1;
        rightX *= 0.75;

        double denominator = Math.max(Math.abs(leftY) + Math.abs(rightX) + Math.abs(leftX), 1.0);
        if (gamepadEx1.gamepad.right_trigger > 0.25) {
            MAX_POWER = 0.3;
            leftX *= (MAX_POWER * 1.5);
            leftY *= MAX_POWER;
            rightX *= MAX_POWER;
        } else {
            MAX_POWER = 1.0;
        }

        if (!headingLock) {
            powerFL = ((leftX + leftY + rightX) / denominator);
            powerFR = ((leftY - leftX - rightX) / denominator);
            powerBL = ((leftY - leftX + rightX) / denominator);
            powerBR = ((leftY + leftX - rightX) / denominator);
        } else {
            double heading = imuControl.getHeading();
            telemetry.addData("Heading", Math.toDegrees(heading));
            double pidfOutput = pidfController.update(heading);

            denominator = Math.max(Math.abs(leftY) + Math.abs(leftX) + Math.abs(pidfOutput), 1.0);
//            powerFL = ((leftX + leftY + rightX - pidfOutput) / denominator);
//            powerFR = ((leftY - leftX - rightX + pidfOutput) / denominator);
//            powerBL = ((leftY - leftX + rightX - pidfOutput) / denominator);
//            powerBR = ((leftY + leftX - rightX + pidfOutput) / denominator);
            powerFL = ((leftX + leftY - pidfOutput) / denominator);
            powerFR = ((leftY - leftX + pidfOutput) / denominator);
            powerBL = ((leftY - leftX - pidfOutput) / denominator);
            powerBR = ((leftY + leftX + pidfOutput) / denominator);
        }

        if (((Math.abs(powerFL) + Math.abs(powerFR) + Math.abs(powerBL) + Math.abs(powerBR)) / 4) > 0.1) {
            isRunning = true;
        }

        FL.setPower(powerFL);
        FR.setPower(powerFR);
        BL.setPower(powerBL);
        BR.setPower(powerBR);
    }

    private double applySmoothPower(double value) {
        return (value < 0) ? -Math.pow(value, 2) : Math.pow(value, 2);
    }

    @Override
    public void updateWithVoltage(double startingVoltage) {
        updateNoVoltage();
    }

    @Override
    public void update(double startingVoltage) {
        if(adjustForVoltage) {
            updateWithVoltage(startingVoltage);
//            updateTelemetry();
            return;
        }
        updateNoVoltage();
//        updateTelemetry();
    }

    @Override
    public void updateTelemetry() {
        telemetry.addData("Drivetrain", " ");
        telemetry.addData("Power: FL | BL", "%.2f | %.2f", powerFL, powerBL);
        telemetry.addData("Power: FR | BR", "%.2f | %.2f", powerFR, powerBR);
    }

    @Override
    public boolean isRunning() {
        return isRunning;
    }

    @Override
    public double getMaxCurrent() {
        return MAX_CURRENT_DRAW * MAX_POWER;
    }

    @Override
    public void stop() {
        powerFL = 0.0;
        powerFR = 0.0;
        powerBL = 0.0;
        powerBR = 0.0;

        FL.setPower(powerFL);
        FR.setPower(powerFR);
        BL.setPower(powerBL);
        BR.setPower(powerBR);

        isRunning = false;
    }

    @Override
    public String getName() {
        return "dt";
    }
}
