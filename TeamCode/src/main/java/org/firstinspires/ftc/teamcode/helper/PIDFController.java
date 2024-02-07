package org.firstinspires.ftc.teamcode.helper;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFController {

    private double previousOutput = 0.0;
    private double previousError = 0.0;
    private double sumError = 0;
    private double targetPosition = 0;
    private double targetTolerance = 0.0;
    private double outputDerivativeLimit = 0.0;
    private final ElapsedTime time;

    private double kp = 0.0;
    private double ki = 0.0;
    private double kd = 0.0;
    private double kf = 0.0;

    public PIDFController(PIDFCoefficients pidfCoefficients) {
        this(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.p, pidfCoefficients.f);
    }

    public PIDFController(double kp) {
        this(kp, 0, 0, 0);
    }

    public PIDFController(double kp, double ki) {
        this(kp, ki, 0, 0);
    }

    public PIDFController(double kp, double ki, double kd) {
        this(kp, ki, kd, 0);
    }

    public PIDFController(double kp, double ki, double kd, double kf) {
        this.setCoefficients(kp, ki, kd, kf);
        time = new ElapsedTime();
    }

    public void setCoefficients(double kp) {
        setCoefficients(kp, 0, 0, 0);
    }

    public void setCoefficients(double kp, double ki) {
        setCoefficients(kp, ki, 0, 0);
    }

    public void setCoefficients(double kp, double ki, double kd) {
        setCoefficients(kp, ki, kd, 0);
    }

    public void setCoefficients(double kp, double ki, double kd, double kf) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
    }

    public void setOutputDerivativeLimit(double outputDerivativeLimit) {
        this.outputDerivativeLimit = outputDerivativeLimit;
    }

    public void setTargetTolerance(double targetTolerance) {
        this.targetTolerance = targetTolerance;
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
        sumError = 0;
        time.reset();
    }

    public boolean atTarget() {
        return Math.abs(previousError) < targetTolerance;
    }

    public double update(double processVariable, double feedforwardMultiplier) {
        double error = targetPosition - processVariable;
        double output = kf * feedforwardMultiplier;

        if(Math.abs(error) > targetTolerance) {
            double timeElapsed = time.seconds();
            double derivative = (error - previousError) / timeElapsed;

            output = (kp * error) + (ki * sumError) + (kd * derivative) + (kf * feedforwardMultiplier);

            if (Math.abs(output) < 1) {
                sumError += error;
            }

            if (outputDerivativeLimit != 0.0) {
                double outputDerivative = (output - previousOutput) / timeElapsed;

                if (outputDerivative > outputDerivativeLimit) { // acceleration is positive
                    output = previousOutput + (outputDerivativeLimit * timeElapsed);
                } else if (outputDerivative < -outputDerivativeLimit) { // acceleration is negative
                    output = previousOutput - (outputDerivativeLimit * timeElapsed);
                }
            }
        }

        previousOutput = output;
        previousError = error;
        time.reset();
        return output;
    }

    public double update(double processVariable) {
        double error = targetPosition - processVariable;
        double output = kf;

        if(Math.abs(error) > targetTolerance) {
//            double timeElapsed = time.seconds();
            double derivative = (error - previousError) / time.seconds();

            output = (kp * error) + (ki * sumError) + (kd * derivative) + kf;

            if (Math.abs(output) < 1) {
                sumError += error;
            }

            if (outputDerivativeLimit != 0.0) {
                double outputDerivative = (output - previousOutput) / time.seconds();

                if (outputDerivative > outputDerivativeLimit) { // acceleration is positive
                    output = previousOutput + (outputDerivativeLimit * time.seconds());
                } else if (outputDerivative < -outputDerivativeLimit) { // acceleration is negative
                    output = previousOutput - (outputDerivativeLimit * time.seconds());
                }
            }
        }

        previousOutput = output;
        previousError = error;
        time.reset();
        return output;
    }
}
