package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.game.ShootSide;

import java.util.Map;

@Configurable
@Config
public class TwoWheelShooter extends SubsystemBase {
    public enum RunMode {
        RawPower,
        VelocityControl
    }

    public enum ShootDist{
        Close,
        Far
    }
    InterpLUT distToLowVel;
    InterpLUT distToHighVel;

    // fill in later
    public static double[] distArr = {58, 70.5, 88.6, 107};
    public static double[] bottomVel = {2000, 1900, 1700, 2100}; // Ticks per second when 1:1 gear ratio
    public static double[] topVel = {1850, 1950, 1950, 2150};

    public static double gearRatio = 3;
    public final MotorEx low;
    public final MotorEx high;
    public RunMode runMode;
    public static double minDistanceThreshold = 10;//INCH
    public static Pose leftShootPose = new Pose(0, 144, Math.toRadians(90));
    public static Pose rightShootPose = new Pose(144, 144, Math.toRadians(90));
    public static Map<Double, Double> grToMultiplier = Map.of(
            3., 2.89,
            4., 3.61,
            5., 5.23
    ); // Unused for now

    public static boolean lowMotorDirForward = true;
    public static boolean highMotorDirForward = true;
    public static double topVelocityOffset = 500;
    double predictedTopVel = 0;
    double predictedBotVel = 0;

    public static double topRecoveryFactor = 1.18;//TUNE
    public static double botRecoveryFactor = 1.17;//TUNE
    double currTopFactor = 1;
    double currBotFactor = 1;
    public static double recoveryTime = 150;
    double recoveryEndTime = 0;

    public static double targetVoltage = 12.0;

//    public static double[] closeTargetVelocities = new double[] {1800, 1900};
    public static double[] closeTargetVelocities = new double[] {1600, 1750};
    public static double[] farTargetVelocities = new double[]{2300, 2500};
    public static double[] closeTargetPowers = new double[]{0.75, 0.95};
    public static double[] farTargetPowers = new double[]{1, 0.95};
    HardwareMap map;


    public TwoWheelShooter(HardwareMap hardwareMap, RunMode runMode) {
        low = new MotorEx(hardwareMap, ConfigNames.lowFlywheelMotor);
        high = new MotorEx(hardwareMap, ConfigNames.highFlywheelMotor);
        hardwareMap = map;
        setRunMode(runMode);

//        distToLowVel = new InterpLUT();
//        distToHighVel = new InterpLUT();
//        for (int i = 0; i < distArr.length; i++) {
//            distToLowVel.add(distArr[i], bottomVel[i]);
//            distToHighVel.add(distArr[i], topVel[i]);
//        }
//
//        distToLowVel.createLUT();
//        distToHighVel.createLUT();

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
    public boolean setFlywheelsPowerVoltage(ShootDist dist) {//assuming facing the shooting area
        double currVolt = map.voltageSensor.iterator().next().getVoltage();
        if(currVolt < 10){currVolt = 10;}

        if(runMode == RunMode.VelocityControl){
            if (dist == ShootDist.Close) {
//                low.set(closeTargetVelocities[0] + 30 );//may need to do set instead
//                high.set(closeTargetVelocities[1] + topVelocityOffset + 30);
                low.set(closeTargetVelocities[0] * (targetVoltage / currVolt));
                high.set(closeTargetVelocities[1] * (targetVoltage / currVolt));
            }
            else{
                low.set(farTargetVelocities[0] * (targetVoltage / currVolt));
                high.set(farTargetVelocities[1] * (targetVoltage / currVolt));
            }
        }
        else{
//            if (dist == ShootDist.Close) setCustomPower(0.75, 0.95);
//            else setCustomPower(1, 1);
            if (dist == ShootDist.Close) setCustomPower(closeTargetPowers[0], closeTargetPowers[1]);
            else setCustomPower(farTargetPowers[0], farTargetPowers[1]);
        }

        return true;
    }

    public boolean setFlywheelsPower(double dist) {//assuming facing the shooting area
//        if(System.currentTimeMillis() > recoveryTime){
//            currBotFactor = 0;
//            currTopFactor = 0;
//        }
//
//        predictedBotVel = distToLowVel.get(dist) * currBotFactor;
//        predictedTopVel = distToHighVel.get(dist) * currTopFactor;
//        switch (runMode) {
//            case VelocityControl:
//                low.set(predictedBotVel); high.set(predictedTopVel);
//                break;
//
//            case RawPower:
//                low.set(predictedBotVel / low.ACHIEVABLE_MAX_TICKS_PER_SECOND);
//                high.set(predictedTopVel / high.ACHIEVABLE_MAX_TICKS_PER_SECOND);
//                break;
//        }
        return true;
    }

    public void triggerBallShot(){//every time ball is shot
        currBotFactor = botRecoveryFactor;
        currTopFactor = topRecoveryFactor;
        recoveryEndTime = System.currentTimeMillis() + recoveryTime;
    }


    public double getPredictedTopVel(){
        return predictedTopVel;
    }
    public double getPredictedBotVel(){
        return predictedBotVel;
    }
    public void setFlywheelsPower(ShootDist dist) {
//        setRunMode(runMode);
        if(runMode == RunMode.VelocityControl){
            if (dist == ShootDist.Close) {
                low.set(closeTargetVelocities[0]);//may need to do set instead
                high.set(closeTargetVelocities[1] + topVelocityOffset);
            }
            else{
                low.set(farTargetVelocities[0]);
                high.set(farTargetVelocities[1]);
            }
        }
        else{
//            if (dist == ShootDist.Close) setCustomPower(0.75, 0.95);
//            else setCustomPower(1, 1);
            if (dist == ShootDist.Close) setCustomPower(closeTargetPowers[0], closeTargetPowers[1]);
            else setCustomPower(farTargetPowers[0], farTargetPowers[1]);
        }
    }

    public void setFlywheelsPower(ShootDist dist, TwoWheelShooter.RunMode runMode) {
//        setRunMode(runMode);
        if(this.runMode == RunMode.VelocityControl){
            if (dist == ShootDist.Close) {
                low.setVelocity(closeTargetVelocities[0]);
                high.setVelocity(closeTargetVelocities[1] + 500);
            }
            else{
                low.setVelocity(farTargetVelocities[0]);
                high.setVelocity(farTargetVelocities[1]);
            }
        }
        else{
//            if (dist == ShootDist.Close) setCustomPower(0.75, 0.95);
//            else setCustomPower(1, 1);
            if (dist == ShootDist.Close) setCustomPower(closeTargetPowers[0], closeTargetPowers[1]);
            else setCustomPower(farTargetPowers[0], farTargetPowers[1]);
        }
    }


    public void setCustomPower(double lowPower, double highPower) {
        if(runMode == RunMode.VelocityControl){
//            low.setRunMode(Motor.RunMode.VelocityControl);
//            high.setRunMode(Motor.RunMode.VelocityControl);
            low.set(lowPower);
            high.set(highPower + topVelocityOffset);//account for belted motor
        }
        else {
            low.set(lowPower);
            high.set(highPower);
        }
    }

    public boolean setFlywheelsPower(Pose robotPose, ShootSide side){
        return setFlywheelsPower(getDistance(robotPose, side));
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
