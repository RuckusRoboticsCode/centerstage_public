package org.firstinspires.ftc.teamcode.motionProfilingSlides;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SlidesPIDController {

	public double kp;
	public double ki;
	public double kd;
	public int targetVelocity;
	private int lastError;
	private double integralSum;
	private double derivative;
	private ElapsedTime timer;

	public SlidesPIDController() {
		this.kp = SlidesConstants.getKp();
		this.ki = SlidesConstants.getKi();
		this.kd = SlidesConstants.getKd();
		this.targetVelocity = 0;
		this.lastError = 0;
		this.timer = new ElapsedTime();
	}

	public void setCoefficients(double kp, double ki, double kd) {
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
	}

	public void updateCoefficients() {
		this.kp = SlidesConstants.getKp();
		this.ki = SlidesConstants.getKi();
		this.kd = SlidesConstants.getKd();
	}

	public void updateTargetPosition(int targetVelocity) {
		this.integralSum = 0;
		this.targetVelocity = targetVelocity;
	}

	public double getPower(int current, int target) {
		int error = target - current;

		derivative = (lastError - error) / timer.seconds();
		integralSum += (error * timer.seconds());

		double output = (error * kp) + (derivative * kd) + (integralSum * ki);

		lastError = error;
		timer.reset();
		return output;
	}
}
