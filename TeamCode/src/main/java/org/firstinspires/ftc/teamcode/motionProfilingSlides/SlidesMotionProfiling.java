package org.firstinspires.ftc.teamcode.motionProfilingSlides;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SlidesMotionProfiling {

    private MotionProfile motionProfile;
    private ElapsedTime motionTimer;
    private SlidesPIDController pidController;

    public SlidesMotionProfiling() {
        this.motionProfile = null;
        this.motionTimer = new ElapsedTime();
        this.pidController = new SlidesPIDController();
    }

    public void setMotionProfile(int startingPosition, int endingPosition) {
        this.motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(startingPosition, 0),
                new MotionState(endingPosition, 0),
                SlidesConstants.getMaxVelocity(),
                SlidesConstants.getMaxAcceleration()
        );
        this.motionTimer.reset();
    }

    public MotionProfile getMotionProfile() {return this.motionProfile;}

    public void maintainPosition() {
        this.motionProfile = null;
    }

    public double getPower(int currentPosition) {
        double power = 0;
        if(this.motionProfile != null) {
            MotionState targetMotionState = motionProfile.get(motionTimer.seconds());
            double targetVelocity = targetMotionState.getV();
            double targetAcceleration = targetMotionState.getA();
            int targetPosition = (int)targetMotionState.getX();
            double feedback = pidController.getPower(currentPosition, targetPosition);
            double feedforward = (SlidesConstants.getKv() * targetVelocity) + (SlidesConstants.getKa() * targetAcceleration);
            power = feedback + feedforward;
        }
        power += SlidesConstants.getKg();
        return power;
    }
}
