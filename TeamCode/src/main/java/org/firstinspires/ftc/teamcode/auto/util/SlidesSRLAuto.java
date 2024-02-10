package org.firstinspires.ftc.teamcode.auto.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helper.Constants;
import org.firstinspires.ftc.teamcode.caching.CachingDcMotorEX;
import org.firstinspires.ftc.teamcode.helper.Hardware;
import org.firstinspires.ftc.teamcode.helper.PIDFController;

public class SlidesSRLAuto {

	private final DcMotorEx leftMotor;
	private final DcMotorEx rightMotor;

	private final Telemetry telemetry;
	private int targetPosition = 0;
	private final int maxEncoders;
	private int avgPos = 0;
	private double power = 0.0;
	private PIDFController pidfController;

	public SlidesSRLAuto(HardwareMap hardwareMap, Telemetry telemetry, int maxEncoders, double maxROC) {
		this.leftMotor = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, Hardware.LINEAR_SLIDE_LEFT));
		this.rightMotor = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, Hardware.LINEAR_SLIDE_RIGHT));

		this.telemetry = telemetry;
		this.maxEncoders = maxEncoders;
		this.pidfController = new PIDFController(
				Constants.SlidesPIDF.kp.value * 1.25,
				Constants.SlidesPIDF.ki.value,
				Constants.SlidesPIDF.kd.value,
				Constants.SlidesPIDF.kg.value);

		pidfController.setTargetTolerance(20);
//		pidfController.setOutputDerivativeLimit(maxROC);

		leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
		rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

		leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}

	public int getAveragePosition() {
//		return (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;
		return leftMotor.getCurrentPosition();
	}

	public void updatePower() {
		avgPos = getAveragePosition();
		power = pidfController.update(avgPos);

		if (avgPos > maxEncoders || avgPos < -10) {
			power = Constants.SlidesPIDF.kg.value;
		}

		updateTelemetry();
		leftMotor.setPower(power);
		rightMotor.setPower(power);
	}

	public boolean atTarget() {
		return pidfController.atTarget();
	}

	public void updateTelemetry() {
		telemetry.addLine("Linear Slides")
				.addData("Power", power)
				.addData("Current Position", avgPos)
				.addData("Target Position", targetPosition);
	}

	public void setTargetPosition(int targetPosition) {
		this.targetPosition = targetPosition;
		this.pidfController.setTargetPosition(targetPosition);
	}
}
