package org.firstinspires.ftc.teamcode.tele.subsystems;

import static org.firstinspires.ftc.teamcode.helper.Constants.DEADZONE;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.caching.CachingDcMotorEX;
import org.firstinspires.ftc.teamcode.helper.Constants;
import org.firstinspires.ftc.teamcode.helper.Hardware;
import org.firstinspires.ftc.teamcode.helper.PIDFController;

public class LinearSlides implements Subsystem {

    private DcMotorEx leftMotor, rightMotor;
    private GamepadEx gamepadEx;
    private TouchSensor limitSwitch;

    private Telemetry telemetry;
    private int targetPosition = 0;
    private ElapsedTime timer;
    private int maxEncoders;
    private int leftPos = 0;
    private int rightPos = 0;
    private int avgPos = 0;
    private final double MAX_CURRENT_DRAW = 4.0;
    private double power = 0.0;
    private double derivativeLimit = 0.0;
    private PIDFController pidfController;

    private boolean goingToTarget = false;
    private boolean allowedToReset = true;
    private boolean isRunning = false;
    private boolean adjustForVoltage = false;
    private boolean override = false;

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.leftMotor = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, Hardware.LINEAR_SLIDE_LEFT));
        this.rightMotor = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, Hardware.LINEAR_SLIDE_RIGHT));
        this.limitSwitch = hardwareMap.get(TouchSensor.class, "limit");

        this.telemetry = telemetry;
        this.maxEncoders = Constants.SlidePositions.LIMIT.position;
        this.pidfController = new PIDFController(
                Constants.SlidesPIDF.kp.value,
                Constants.SlidesPIDF.ki.value,
                Constants.SlidesPIDF.kd.value,
                Constants.SlidesPIDF.kg.value
        );
        pidfController.setTargetTolerance(20);
        pidfController.setOutputDerivativeLimit(derivativeLimit);

        configureMotors();
    }

    private void configureMotors() {
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        resetPosition();
    }

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry, boolean adjustForVoltage) {
        init(hardwareMap, telemetry);
        this.adjustForVoltage = adjustForVoltage;
    }

    public void getAveragePosition() {
//        avgPos = (leftPos + rightPos) / 2;
        avgPos = leftPos;
    }

    public void resetPosition() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void read() {
        leftPos = leftMotor.getCurrentPosition();
//        rightPos = rightMotor.getCurrentPosition();
        getAveragePosition();
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public void setTargetPosition(int targetPosition) {
        this.targetPosition = targetPosition;
        this.pidfController.setTargetPosition(targetPosition);
    }

    public void setMaxROC(double maxROC) {
//        pidfController.setOutputDerivativeLimit(maxROC);
        this.derivativeLimit = maxROC;
    }

    public void setGamepadEx(GamepadEx gamepadEx) {
        this.gamepadEx = gamepadEx;
    }

    @Override
    public void updateNoVoltage() {
        boolean pressed = limitSwitch.isPressed();
//        boolean pressed = false;

        if (gamepadEx.wasJustPressed(GamepadKeys.Button.BACK)) {
            override = true;
        }

        if (allowedToReset && pressed) {
            resetPosition();
            allowedToReset = false;
        } else if (!pressed){
            allowedToReset = true;
        }

        double leftY = gamepadEx.getLeftY();
        double rightY = -gamepadEx.getRightY();
        power = 0.0;

        if (Math.abs(leftY) > DEADZONE || Math.abs(rightY) > DEADZONE) {
            goingToTarget = false;
            isRunning = true;
        }

        if((avgPos > maxEncoders || avgPos < -25) && !override) {
            power = 0.0;
            isRunning = false;
            return;
        }

        if (!override) {
            handlePresets();
        } else {
            goingToTarget = false;
        }

        if ((Math.abs(leftY) > DEADZONE || Math.abs(rightY) > DEADZONE) && !goingToTarget) {

            if (Math.abs(rightY) > DEADZONE) {
                if (avgPos >= 0 && rightY < -DEADZONE) {
                    if (avgPos <= 150) {
                        power += rightY * 0.75;
                    } else {
                        power += rightY;
                    }
                } else if (avgPos <= maxEncoders && rightY > DEADZONE) {
                    if ((maxEncoders - avgPos) <= 150) {
                        power += rightY * 0.75;
                    } else {
                        power += rightY;
                    }
                }
                power *= 0.3;
            } else if (Math.abs(leftY) > DEADZONE){
                if (avgPos >= 0 && leftY < -DEADZONE) {
                    if (avgPos <= 150) {
                        power += leftY * 0.75;
                    } else {
                        power += leftY;
                    }
                } else if (avgPos <= maxEncoders && leftY > DEADZONE) {
                    if ((maxEncoders - avgPos) <= 150) {
                        power += leftY * 0.75;
                    } else {
                        power += leftY;
                    }
                }
            }

            if(avgPos > 200) {
                power += Constants.SlidesPIDF.kg.value;
            }

            targetPosition = avgPos;
        } else {
            if(Math.abs(avgPos - targetPosition) < 20) {
                isRunning = false;
            }
            power = pidfController.update(avgPos);
        }
        pidfController.setTargetPosition(targetPosition);
    }

    private void handlePresets() {
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.Y)) {
            this.setTargetPosition(Constants.SlidePositions.HIGH.getPosition());
            goingToTarget = true;
            isRunning = true;
        } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.A)) {
            this.setTargetPosition(Constants.SlidePositions.DOWN.getPosition());
            goingToTarget = true;
            isRunning = true;
        } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.B)) {
            this.setTargetPosition(Constants.SlidePositions.HANGING.getPosition());
            goingToTarget = true;
            isRunning = true;
        }
//        if (gamepadEx.wasJustPressed(GamepadKeys.Button.X)) {
//            this.setTargetPosition(Constants.SlidePositions.LOW.getPosition());
//            goingToTarget = true;
//            isRunning = true;
//        } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.Y)) {
//            this.setTargetPosition(Constants.SlidePositions.MIDDLE.getPosition());
//            goingToTarget = true;
//            isRunning = true;
//        } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.B)) {
//            this.setTargetPosition(Constants.SlidePositions.HIGH.getPosition());
//            goingToTarget = true;
//            isRunning = true;
//        } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.A)) {
//            this.setTargetPosition(Constants.SlidePositions.DOWN.getPosition());
//            goingToTarget = true;
//            isRunning = true;
//        }
    }

    @Override
    public void updateWithVoltage(double startingVoltage) {
        updateNoVoltage();
        power *= (12.0 / startingVoltage);
    }

    @Override
    public void update(double startingVoltage) {
        if (adjustForVoltage) {
            updateWithVoltage(startingVoltage);
        } else {
            updateNoVoltage();
        }
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        updateTelemetry();
    }

    @Override
    public void updateTelemetry() {
        telemetry.addData("Linear Slides", " ");
        telemetry.addData("Power", "%.2f", power);
        telemetry.addData("LS Target Position", targetPosition);
        telemetry.addData("LS Current Position", avgPos);
    }

    @Override
    public boolean isRunning() {
        return isRunning;
    }

    @Override
    public double getMaxCurrent() {
        return MAX_CURRENT_DRAW;
    }

    @Override
    public void stop() {
        read();
        getAveragePosition();
        targetPosition = avgPos;
        power = Constants.SlidesPIDF.kg.value;
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        isRunning = false;
    }
}
