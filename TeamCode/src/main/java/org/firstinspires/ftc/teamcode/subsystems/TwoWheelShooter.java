package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.util.ConfigNames;

public class TwoWheelShooter extends SubsystemBase {
    public enum RunMode {
        RawPower,
        VelocityControl
    }

    InterpLUT distToVeloLut;

    // fill in later
    public static double[] distArr = {};
    public static double[] velocityArr = {};

    private final MotorEx low;
    private final MotorEx high;
    RunMode runMode;

    public TwoWheelShooter(HardwareMap hardwareMap, RunMode runMode) {
        low = new MotorEx(hardwareMap, ConfigNames.lowFlywheel);
        high = new MotorEx(hardwareMap, ConfigNames.highFlywheel);
        setRunMode(runMode);

        distToVeloLut = new InterpLUT();
        for (int i = 0; i < distArr.length; i++) {
            distToVeloLut.add(distArr[i], velocityArr[i]);
        }
        distToVeloLut.createLUT();
    }

    public void setLowDirection(Motor.Direction direction) {
        low.encoder.setDirection(direction);
    }

    public void setHighDirection(Motor.Direction direction) {
        high.encoder.setDirection(direction);
    }

    public void setRunMode(RunMode runMode) {
        this.runMode = runMode;
        if (runMode == RunMode.RawPower) {
            low.setRunMode(Motor.RunMode.RawPower);
            high.setRunMode(Motor.RunMode.RawPower);
        } else {
            low.setRunMode(Motor.RunMode.VelocityControl);
            high.setRunMode(Motor.RunMode.VelocityControl);
        }
    }

    public void setPidCoefficients(double kp, double ki, double kd) {
        low.setVeloCoefficients(kp, ki, kd);
        high.setVeloCoefficients(kp, ki, kd);
    }

    public void setFlywheels(double dist) {
        double velocity;
        switch (runMode) {
            case VelocityControl:
                velocity = distToVeloLut.get(dist);
                low.set(velocity); high.set(velocity);
                break;

            case RawPower:
                velocity = distToVeloLut.get(dist);
                low.set(velocity / low.ACHIEVABLE_MAX_TICKS_PER_SECOND);
                high.set(velocity / high.ACHIEVABLE_MAX_TICKS_PER_SECOND);
                break;
        }
    }

    public void stopFlywheels() {
        low.motor.setPower(0);
        high.motor.setPower(0);
    }
}
