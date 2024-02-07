package org.firstinspires.ftc.teamcode.tele.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.caching.CachingDcMotorEX;
import org.firstinspires.ftc.teamcode.helper.Hardware;
import org.firstinspires.ftc.teamcode.helper.IMUControl;
import org.firstinspires.ftc.teamcode.helper.OpModeEx;
import org.firstinspires.ftc.teamcode.helper.PIDFController;

//@Disabled
@Config
@TeleOp(name="Heading PID Tuning", group="Tuning")
public class HeadingPIDTuning extends OpModeEx {

    private DcMotorEx FL, FR , BL, BR;
    private PIDFController pidfController;

    private IMUControl imuControl;

    public static double targetHeadingDeg = 0.0;
    public static double outputLimit = 0.3;
    public static double kp = 0.0;
    public static double ki = 0.0;
    public static double kd = 0.0;

    @Override
    public void init() {
        FL = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, Hardware.DRIVETRAIN_FL));
        FR = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, Hardware.DRIVETRAIN_FR));
        BL = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, Hardware.DRIVETRAIN_BL));
        BR = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, Hardware.DRIVETRAIN_BR));

        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imuControl = new IMUControl(hardwareMap, telemetry, 0);
        imuControl.setYawOffset(imuControl.getHeading());

        pidfController = new PIDFController(0, 0, 0);
        pidfController.setTargetTolerance(0);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
    }

    @Override
    public void loop() {
        double currentHeading = Math.toDegrees(imuControl.getHeading());
        pidfController.setCoefficients(kp, ki, kd);
        pidfController.setTargetPosition(targetHeadingDeg);
        pidfController.setOutputDerivativeLimit(outputLimit);

        double power = pidfController.update(currentHeading);

        FL.setPower(-power);
        BL.setPower(-power);
        FR.setPower(power);
        BR.setPower(power);

        telemetry.addData("Power", power);
        telemetry.addData("Current Heading", currentHeading);
        telemetry.addData("Target Heading", targetHeadingDeg);
        telemetry.addData("Error", targetHeadingDeg - currentHeading);
    }
}
