package org.firstinspires.ftc.teamcode.newpid;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.newpid.interpolators.Interpolator2D;

import java.util.function.Supplier;

public class DanielPIDController<T extends Interpolator2D> {
    private double kp, ki, kd;
    private double integralSum = 0;
    private double lastError = 0;
    private T predictError;

    ElapsedTime timer;

    @SuppressWarnings("unchecked")
    public DanielPIDController(Supplier<T> interpFactory, double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        timer = new ElapsedTime();
        try {
            predictError = (T) interpFactory.get().fromFile("tests/predictError.json");
        } catch (Exception e) {
            predictError = null;
        }
    }

    public double calculate(double pos, double target) {
        double error = target - pos;
        integralSum += (error * timer.seconds());
        double derivative = (error - lastError) / timer.seconds();
        double power = this.kp*error + this.ki*integralSum + this.kd*derivative;
        if (predictError != null) {
            power += predictError.getZ(derivative, error) * this.kp;
        }
        lastError = error;
        this.timer.reset();
        return power;
    }
}
