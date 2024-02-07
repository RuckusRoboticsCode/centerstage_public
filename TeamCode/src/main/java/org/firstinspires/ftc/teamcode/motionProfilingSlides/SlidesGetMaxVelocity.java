package org.firstinspires.ftc.teamcode.motionProfilingSlides;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;

import java.util.ArrayList;

@Disabled
@TeleOp(name="Max Slides Speed")
public class SlidesGetMaxVelocity extends OpMode {

    DcMotorEx leftSlide;
    DcMotorEx rightSlide;

    int upperLimit = 2000;
    boolean timeUp = false;
    boolean valsFound = false;
    double MAX_VELOCITY = 0;
    double MAX_ACCEL = 0;

    ElapsedTime timer;
    ArrayList<Double> velocities = new ArrayList<>();
    ArrayList<Double> accelerations = new ArrayList<>();

    @Override
    public void init() {
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

        timer = new ElapsedTime();
    }

    public int getAveragePosition() {
        return (leftSlide.getCurrentPosition() + rightSlide.getCurrentPosition()) / 2;
    }

    @Override
    public void loop() {
        if (!timeUp) {
            int averagePosition = getAveragePosition();
            if (averagePosition < 0 || averagePosition > upperLimit) {
                leftSlide.setPower(0);
                rightSlide.setPower(0);
//            requestOpModeStop();
            } else if (timer.seconds() < 2) {
                leftSlide.setPower(1);
                rightSlide.setPower(1);

                Double velocityLeft = leftSlide.getVelocity();
                Double velocityRight = rightSlide.getVelocity();
                Double averageVelocity = (velocityLeft + velocityRight) / 2.0;

                velocities.add(averageVelocity);

            } else {
                leftSlide.setPower(0);
                rightSlide.setPower(0);
                timeUp = true;
            }
        } else {
            for(int i = 0; i < velocities.size(); i++) {
                if (velocities.get(i) > MAX_VELOCITY) {
                    MAX_VELOCITY = velocities.get(i);
                }
            }
            for(int i = 0; i < accelerations.size(); i++) {
                if (accelerations.get(i) > MAX_ACCEL) {
                    MAX_ACCEL = accelerations.get(i);
                }
            }
            valsFound = true;
        }
        telemetry.addData("Max Velocity", MAX_VELOCITY);
        telemetry.addData("75% Velocity", MAX_VELOCITY * 0.75);
        telemetry.addData("Max Acceleration", MAX_ACCEL);
        telemetry.addData("75% Acceleration", MAX_ACCEL * 0.75);

        telemetry.update();
    }
}
