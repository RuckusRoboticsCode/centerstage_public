package org.firstinspires.ftc.teamcode.motionProfilingSlides;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;

@Config
public class SlidesConstants {

    private static double Kv = 0.0;
    private static double Ka = 0.0;
    private static double Kg = 0.0;
    private static double Kp = 0.0;
    private static double Ki = 0.0;
    private static double Kd = 0.0;
    private static double MAX_VELOCITY = 0.0;
    private static double MAX_ACCELERATION = 0.0;

    public static double getKv() {
        return Kv;
    }

    public static void setKv(double kv) {
        Kv = kv;
    }

    public static double getKa() {
        return Ka;
    }

    public static void setKa(double ka) {
        Ka = ka;
    }

    public static double getKg() {
        return Kg;
    }

    public static void setKg(double kg) {
        Kg = kg;
    }

    public static double getKp() {
        return Kp;
    }

    public static void setKp(double kp) {
        Kp = kp;
    }

    public static double getKi() {
        return Ki;
    }

    public static void setKi(double ki) {
        Ki = ki;
    }

    public static double getKd() {
        return Kd;
    }

    public static void setKd(double kd) {
        Kd = kd;
    }

    public static double getMaxVelocity() {
        return MAX_VELOCITY;
    }

    public static void setMaxVelocity(double maxVelocity) {
        MAX_VELOCITY = maxVelocity;
    }

    public static double getMaxAcceleration() {
        return MAX_ACCELERATION;
    }

    public static void setMaxAcceleration(double maxAcceleration) {
        MAX_ACCELERATION = maxAcceleration;
    }
}
