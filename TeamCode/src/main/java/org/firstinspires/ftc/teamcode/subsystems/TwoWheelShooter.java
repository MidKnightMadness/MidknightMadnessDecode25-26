package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.OutdatedPrograms.DoubleFlywheelCustomPID.leftForward;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.util.ShootSide;

import java.util.Map;

@Configurable
@Config
public class TwoWheelShooter extends SubsystemBase {
    public enum RunMode {
        RawPower,
        VelocityControl
    }

    InterpLUT distToBottomVel;
    InterpLUT distToTopVel;

    // fill in later
    public static double[] distArr = {};
    public static double[] bottomVel = {}; // Ticks per second when 1:1 gear ratio

    public static double[] topVel = {};
    public static double gearRatio = 3;
    public final MotorEx low;
    public final MotorEx high;
    private RunMode runMode;
    public static double minDistanceThreshold = 10;//INCH
    public static Pose leftShootPose = new Pose(0, 144, Math.toRadians(90));
    public static Pose rightShootPose = new Pose(144, 144, Math.toRadians(90));
    public static Map<Double, Double> grToMultiplier = Map.of(
            3., 2.89,
            4., 3.61,
            5., 5.23
    ); // Unused for now

    public static boolean lowMotorDirForward = true;
    public static boolean highMotorDirForward = false;

    public TwoWheelShooter(HardwareMap hardwareMap, RunMode runMode) {
        low = new MotorEx(hardwareMap, ConfigNames.lowFlywheelMotor);
        high = new MotorEx(hardwareMap, ConfigNames.highFlywheelMotor);
        setRunMode(runMode);

        distToBottomVel = new InterpLUT();
        distToTopVel = new InterpLUT();
        for (int i = 0; i < distArr.length; i++) {
            distToBottomVel.add(distArr[i], bottomVel[i]);
            distToTopVel.add(distArr[i], topVel[i]);
        }

        distToBottomVel.createLUT();
        low.motor.setDirection(lowMotorDirForward ? DcMotorEx.Direction.FORWARD : DcMotorEx.Direction.REVERSE);
        high.motor.setDirection(highMotorDirForward ? DcMotorEx.Direction.FORWARD : DcMotorEx.Direction.REVERSE);

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

    public void setPid(double kp, double ki, double kd) {
        low.setVeloCoefficients(kp, ki, kd);
        high.setVeloCoefficients(kp, ki, kd);
    }
    public void setFeedforward(double kS, double kV, double kA){
        low.setFeedforwardCoefficients(kS, kV, kA);
        high.setFeedforwardCoefficients(kS, kV, kA);
    }

    public boolean setFlywheelsPower(double dist) {//assuming facing the shooting area
        if(dist < minDistanceThreshold ){//not possible to make it in the goal
            return false;
        }
        double topVel = distToBottomVel.get(dist) * grToMultiplier.getOrDefault(gearRatio, 3.0);
        double bottomVel = distToTopVel.get(dist) * grToMultiplier.getOrDefault(gearRatio, 3.0);
        switch (runMode) {
            case VelocityControl:
                low.set(bottomVel); high.set(topVel);
                break;

            case RawPower:
                low.set(bottomVel / low.ACHIEVABLE_MAX_TICKS_PER_SECOND);
                high.set(topVel / high.ACHIEVABLE_MAX_TICKS_PER_SECOND);
                break;
        }
        return true;
    }

    public void setFlywheelPower(Pose robotPose, ShootSide side){
        setFlywheelsPower(getDistance(robotPose, side));
    }
    public double getDistance(Pose robotPose, ShootSide side){
        double xDist = Math.abs(robotPose.getX() - ((side == ShootSide.LEFT) ? leftShootPose.getX() : rightShootPose.getX()));
        double yDist = Math.abs(robotPose.getY() - ((side == ShootSide.LEFT) ? leftShootPose.getY() : rightShootPose.getY()));
        return Math.hypot(xDist, yDist);
    }

    public void stopFlywheels() {
        low.motor.setPower(0);
        high.motor.setPower(0);
    }
}
