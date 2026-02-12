package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double kp, ki, kd;
    private double integralSum = 0;
    private double lastError = 0;

    ElapsedTime timer;

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        timer = new ElapsedTime();
    }

    public void setConstants(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public double calculate(double pos, double target) {
        double error = target - pos;
        integralSum = integralSum + (error * timer.seconds());
        double derivative = (error - lastError) / timer.seconds();
        double power = this.kp*error + this.ki*integralSum + this.kd*derivative;
        lastError = error;
        this.timer.reset();
        return power;
    }
}
