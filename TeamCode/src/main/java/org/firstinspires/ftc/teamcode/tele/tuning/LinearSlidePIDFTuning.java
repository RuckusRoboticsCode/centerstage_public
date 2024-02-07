package org.firstinspires.ftc.teamcode.tele.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.caching.CachingDcMotorEX;
import org.firstinspires.ftc.teamcode.helper.Hardware;
import org.firstinspires.ftc.teamcode.helper.OpModeEx;

@Config
@Disabled
@TeleOp(name="Linear Slides PIDF Tuning", group="Tuning")
public class LinearSlidePIDFTuning extends OpModeEx {

    public static double kp = 0.0;
    public static double ki = 0.0;
    public static double kd = 0.0;
    public static double kf = 0.0;
    public static int targetPosition;
    public static double maxROC = 0.0;
    public double lastOutput = 0.0;
    public int lastError = 0;
    public int avgPos;
    public int tolerance = 10;
    public int sumError;
    public double power = 0.0;
    ElapsedTime timer;

    DcMotorEx leftMotor;
    DcMotorEx rightMotor;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        leftMotor = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, Hardware.LINEAR_SLIDE_LEFT));
        rightMotor = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, Hardware.LINEAR_SLIDE_RIGHT));

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        avgPos = getAveragePosition();

        timer = new ElapsedTime();
    }

    @Override
    public void loop() {
        avgPos = getAveragePosition();
        power = getPower(avgPos, targetPosition);
        leftMotor.setPower(power);
        rightMotor.setPower(power);

        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Current Position", avgPos);
        telemetry.addData("Power", power);
    }

    public int getAveragePosition() {
//        return (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;
        return leftMotor.getCurrentPosition();
    }

    public double getPower(int currentPosition, int targetPosition) {
        int error = targetPosition - currentPosition;
        double output = kf;

        if(Math.abs(error) > tolerance) {
            double derivative = (error - lastError) / timer.seconds();

            output = (kp * error) + (ki * sumError) + (kd * derivative);

            if(currentPosition > 200) {output += kf;}

            if (maxROC != 0) {
                double outputDerivative = (output - lastOutput) / timer.seconds(); // akin to acceleration

                if (outputDerivative > maxROC) { // acceleration is positive
                    output = lastOutput + (maxROC * timer.seconds());
                } else if (outputDerivative < -maxROC) { // acceleration is negative
                    output = lastOutput - (maxROC * timer.seconds());
                }
            }
            sumError += error;
        }

        lastError = error;
        lastOutput = output;
        timer.reset();
        return output;
    }
}
