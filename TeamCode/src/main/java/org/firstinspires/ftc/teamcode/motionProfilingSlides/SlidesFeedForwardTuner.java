package org.firstinspires.ftc.teamcode.motionProfilingSlides;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name="Slides Feedforward Tuner")
public class SlidesFeedForwardTuner extends OpMode {

    SlidesMotionProfiling motionProfiling;
    MotionProfile upwardProfile;
    MotionProfile downwardProfile;

    int lowerSlides = 500;
    int upperSlides = 2000;

    DcMotorEx leftSlide;
    DcMotorEx rightSlide;

    @Override
    public void init() {
        motionProfiling = new SlidesMotionProfiling();
        motionProfiling.setMotionProfile(lowerSlides, upperSlides);
        upwardProfile = motionProfiling.getMotionProfile();
        motionProfiling.setMotionProfile(upperSlides, lowerSlides);
        downwardProfile = motionProfiling.getMotionProfile();

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
    }

    @Override
    public void loop() {

    }
}
