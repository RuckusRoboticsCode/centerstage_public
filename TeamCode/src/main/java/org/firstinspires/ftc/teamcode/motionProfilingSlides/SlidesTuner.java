package org.firstinspires.ftc.teamcode.motionProfilingSlides;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name="slides tuner MP")
public class SlidesTuner extends OpMode {
    
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;

    int lowerLimit = 500;
    int upperLimit = 2000;
    SlidesMotionProfiling slidesMotionProfiling;

    GamepadEx controller1;
    FtcDashboard ftcDashboard;
    TelemetryPacket telemetryPacket;

    public static int targetPosition = 0;
    ElapsedTime timer;

    @Override
    public void init() {

        slidesMotionProfiling = new SlidesMotionProfiling();

        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        controller1 = new GamepadEx(gamepad1);
        controller1.readButtons();

        ftcDashboard = FtcDashboard.getInstance();
        telemetryPacket = new TelemetryPacket();

        timer = new ElapsedTime();
    }

    public int getAveragePosition() {
        return (leftSlide.getCurrentPosition() + rightSlide.getCurrentPosition()) / 2;
    }

    @Override
    public void loop() {
        timer.reset();
        controller1.readButtons();
        int averagePosition = getAveragePosition();
        int error = targetPosition - averagePosition;

        if(averagePosition < lowerLimit || averagePosition > upperLimit) {
            requestOpModeStop();
        } else {

            if (controller1.wasJustPressed(GamepadKeys.Button.A)) {
                slidesMotionProfiling.setMotionProfile(averagePosition, targetPosition);
            }

            double power = slidesMotionProfiling.getPower(averagePosition);

            telemetry.addData("Power", power);
            telemetry.addData("Average Position", averagePosition);
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Error", error);
            telemetry.addData("Loop Time", timer.milliseconds());

            telemetryPacket.put("Power", power);
            telemetryPacket.put("Average Position", averagePosition);
            telemetryPacket.put("Target Position", targetPosition);
            telemetryPacket.put("Error", error);
            telemetryPacket.put("Loop Time", timer.milliseconds());

            ftcDashboard.sendTelemetryPacket(telemetryPacket);
            telemetry.update();
        }
    }
}
