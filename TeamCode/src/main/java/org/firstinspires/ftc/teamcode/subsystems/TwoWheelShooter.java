package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;


import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.MotorEx;
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

    //DEFAULT GAINS
//    public static double[] pidBotGains = new double[]{0.0004, 0, 0.00001};
   //public static double[] kBotGains = new double[]{0, 0.00005, 0};///change -> 0.00005 to 0.0004 mybe
    public static double[] kBotGains = new double[]{120, 0.00035, 0};
    public static double[] pidTopGains = new double[]{0.0005, 0, 0.00001};
    public static double[] pidBotGains = new double[]{0.00006, 0.00003, 0};
    public static double[] kTopGains = new double[]{0.02, 0.00005, 0};

    public static boolean useAggressiveRecovery = false;
    public boolean inRecoveryMode = false;
    //AGGRESSIVE GAINS: FOR RECOVERY
    public static double[] pidBotAggressiveGains = new double[]{0.0008, 0, 0.00005};
    public static double[] pidTopAggressiveGains = new double[]{0.0008, 0, 0.00005};

    InterpLUT distToLowVel;
    InterpLUT distToHighVel;

    public static double[] dist= {58, 70.5, 88.6, 107};
    public static double[] bottomVel = {2000, 1900, 1700, 2100}; // Ticks per second when 1:1 gear ratio
    public static double[] topVel = {1850, 1950, 1950, 2150};


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
    public static double gearRatio = 3;

    public static boolean lowMotorDirForward = true;
    public static boolean highMotorDirForward = false;
    public static double topVelocityOffset = 0;
    double predictedTopVel = 1500;
    double predictedBotVel = 1500;
    double predictedTopPower = 0.8;
    double predictedBotPower = 0.8;

    public static double topRecoveryFactor = 1.1;//TUNE
    public static double botRecoveryFactor = 1.1;//TUNE
    double currTopFactor = 1;
    double currBotFactor = 1;
    public static double recoveryBoostTime = 1000;
    double actualRecoveryTime = 0;
    double recoveryStartTime = 0;
    double recoveryEndTime = 0;

    public static double targetVoltage = 12.5;

//    public static double[] closeTargetVelocities = new double[] {1800, 1900};
    public static double[] closeTargetVelocities = new double[] {1600, 1750};
    public static double[] farTargetVelocities = new double[]{2200, 1800};
    public static double[] closeTargetPowers = new double[]{0.7, 0.8};
    public static double[] farTargetPowers = new double[]{0.9, 0.85};

    double currVolt = 0;
    HardwareMap map;
    public static double kBotShootMovingFactor = 1;
    public static double kTopShootMovingFactor = 1;
    public static double velBotTolerance = 100;
    public static double velTopTolerance = 100;
    public static double velMovingThreshold = 2;//in per sec
    double topMultiplier = 0;
    double botMultiplier = 0;



    public double getTargetVoltage(){
        return targetVoltage;
    }

    public TwoWheelShooter(HardwareMap hardwareMap, RunMode runMode) {
        low = new MotorEx(hardwareMap, ConfigNames.lowFlywheelMotor);
        high = new MotorEx(hardwareMap, ConfigNames.highFlywheelMotor);
        low.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        high.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        low.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        high.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.map = hardwareMap;
        setRunMode(runMode);

//        distToLowVel = new InterpLUT();
//        distToHighVel = new InterpLUT();
//        for (int i = 0; i < dist.length; i++) {
//            distToLowVel.add(dist[i], bottomVel[i]);
//            distToHighVel.add(dist[i], topVel[i]);
//        }
//
//        distToLowVel.createLUT();
//        distToHighVel.createLUT();

        low.motor.setDirection(lowMotorDirForward ? DcMotorEx.Direction.FORWARD : DcMotorEx.Direction.REVERSE);
        high.motor.setDirection(highMotorDirForward ? DcMotorEx.Direction.FORWARD : DcMotorEx.Direction.REVERSE);
        resetDefaultGains();
    }

    public void resetEncoders() {
        low.stopAndResetEncoder();
        high.stopAndResetEncoder();
    }
    public void resetDefaultGains(){
        low.setVeloCoefficients(pidBotGains[0], pidBotGains[1], pidBotGains[2]);
        low.setFeedforwardCoefficients(kBotGains[0], kBotGains[1], kBotGains[2]);
        high.setVeloCoefficients(pidTopGains[0], pidTopGains[1], pidTopGains[2]);
        high.setFeedforwardCoefficients(kTopGains[0], kTopGains[1], kTopGains[2]);
    }
    public void setAggressiveGains(){
        low.setVeloCoefficients(pidBotAggressiveGains[0], pidBotAggressiveGains[1], pidBotAggressiveGains[2]);
        high.setVeloCoefficients(pidTopAggressiveGains[0], pidTopAggressiveGains[1], pidTopAggressiveGains[2]);
    }

    public double getCurrVoltage(){
        return currVolt;
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
    public void updateRecoveryState(){
        if(inRecoveryMode && System.currentTimeMillis() > recoveryEndTime){
            actualRecoveryTime = System.currentTimeMillis() - recoveryStartTime;
            resetRecoveryFactors();
            resetDefaultGains();
            inRecoveryMode = false;
        }
    }

    //thresholds whether robot is currently moving
    public void setFlywheelPresets(ShootDist shootDist, Follower follower, ShootSide shootSide,  boolean voltageUse){
//        Vector robotVelocity = follower.getVelocity();
//        boolean isMoving = true;
//        if(robotVelocity.getMagnitude() <= velMovingThreshold){
//            isMoving = false;
//        }
//        if(!isMoving){
            setFlywheelStaticPresets(shootDist, voltageUse);
//        } else{
//            setFlywheelMovingPresets(follower.getPose(), shootDist, shootSide, robotVelocity, voltageUse);
//        }
    }

    //thresholds whether robot is currently moving
    public void setFlywheelLUT(Follower follower, ShootSide shootSide, boolean voltageUse){
        Vector robotVelocity = follower.getVelocity();
        boolean isMoving = true;
        if(robotVelocity.getMagnitude() <= velMovingThreshold){
            isMoving = false;
        }
//        if(!isMoving){
            setFlywheelMovingLUT(follower.getPose(), shootSide, robotVelocity, voltageUse);
//        } else{
//            setFlywheelStaticLUT(follower.getPose(), shootSide, voltageUse);
//        }
    }



    public void setFlywheelStaticPresets(ShootDist shootDist, boolean voltageUse) {//assuming facing the shooting area
        setFlywheel(0, shootDist, 0, voltageUse, false);
    }
    public void setFlywheelStaticLUT(Pose robotPose, ShootSide shootSide, boolean voltageUse){
        setFlywheel(getDistance(robotPose, shootSide), null, 0, voltageUse, true);
    }
    public void setFlywheelStaticLUT(double distToGoal, boolean voltageUse){
        setFlywheel(distToGoal, null, 0, voltageUse, true);
    }
    public void setFlywheelMovingLUT(Pose robotPose, ShootSide shootSide, Vector velVector, boolean voltageUse){
        double dist = getDistance(robotPose, shootSide);
        Pose targetPose = getShootPose(shootSide);
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();

        double vparallel = (velVector.getXComponent() * dx + velVector.getYComponent() * dy)
                / dist;

        setFlywheel(dist, null, vparallel, voltageUse, true);
    }

    public void setFlywheelMovingPresets(Pose robotPose, ShootDist shootDist, ShootSide shootSide, Vector velVector, boolean voltageUse){

        double dist = getDistance(robotPose, shootSide);
        Pose targetPose = getShootPose(shootSide);
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();

        double vparallel = (velVector.getXComponent() * dx + velVector.getYComponent() * dy)
                / dist;

        setFlywheel(dist, shootDist, vparallel, voltageUse, false);
    }
    public void setFlywheel(double dist, ShootDist shootDist, double vParallel, boolean voltageUse, boolean useLUT){
        updateRecoveryState();
        currVolt = map.voltageSensor.iterator().next().getVoltage();
        double ratio = voltageUse ? (targetVoltage / currVolt) : 1;
        ratio = Math.min(ratio, 1.35);

        double botVel, topVel;
        if(useLUT){
            botVel = distToLowVel.get(dist);
            topVel = distToHighVel.get(dist);
            if(runMode != RunMode.VelocityControl) setRunMode(RunMode.VelocityControl);
        }
        else{
            double[] preset;
            if(runMode == RunMode.VelocityControl){
                preset = (shootDist == ShootDist.Close) ? closeTargetVelocities : farTargetVelocities;
            } else{
                preset = (shootDist == ShootDist.Close) ? closeTargetPowers : farTargetPowers;
            }
            botVel = preset[0];
            topVel = preset[1];
        }

        botVel -= kBotShootMovingFactor * vParallel;
        topVel -= kTopShootMovingFactor * vParallel;

        topMultiplier = ratio * currTopFactor;
        botMultiplier = ratio * currBotFactor;

        if(runMode == RunMode.VelocityControl) {
            predictedBotVel = botVel;
            predictedTopVel = topVel;
        } else{
            predictedBotPower = botVel;
            predictedTopPower = topVel;
        }

        low.set(botVel, botMultiplier);
        high.set(topVel, topMultiplier);
    }

    //set flywheel by distance -> static, not moving, lut use
    private void setFlywheel(double dist, ShootDist shootDist, boolean voltageUse, boolean useLUT, Pose robotPose, Vector velVector, ShootSide shootSide){
        updateRecoveryState();
        currVolt = map.voltageSensor.iterator().next().getVoltage();

        double ratio = voltageUse ? (targetVoltage / currVolt) : 1;
        ratio = Math.min(ratio, 1.25);

        boolean isMoving = true;
        if(velVector.getXComponent() <= velMovingThreshold && velVector.getYComponent() <= velMovingThreshold){
            isMoving = false;
        }
        double botVel, topVel;
        if(useLUT){
            botVel = distToLowVel.get(dist);
            topVel = distToHighVel.get(dist);
            setRunMode(RunMode.VelocityControl);
        } else{
            double[] preset = (shootDist == ShootDist.Close) ? closeTargetVelocities : farTargetVelocities;
            botVel = preset[0];
            topVel = preset[1];
        }

        if(isMoving){
            double distGoal = getDistance(robotPose, shootSide);
            Pose targetPose = getShootPose(shootSide);
            double dx = targetPose.getX() - robotPose.getX();
            double dy = targetPose.getY() - robotPose.getY();

            double vparallel = (velVector.getXComponent() * dx + velVector.getYComponent() * dy)
                    / distGoal;
            botVel -= kBotShootMovingFactor * vparallel;
            topVel -= kTopShootMovingFactor * vparallel;
        }
        botVel *= ratio * currBotFactor;
        topVel *= ratio * currTopFactor;
        setCustomPower(botVel, topVel);

    }


    public boolean readyToShoot(){
        return Math.abs(low.getVelocity() - predictedBotVel) <= velBotTolerance &&
                Math.abs(high.getVelocity() - predictedTopVel) <= velTopTolerance;
    }

    public double getRecoveryTime(){
        return actualRecoveryTime;
    }

    public double getCurrBotFactor(){
        return currBotFactor;
    }
    public double getCurrTopFactor(){
        return currTopFactor;
    }


    public void triggerBallShot(boolean recover){//every time ball is shot
        if(useAggressiveRecovery){
            setAggressiveGains();
        }
        //override curr recovery if it is in one
        if(recover) {
            currBotFactor = botRecoveryFactor;
            currTopFactor = topRecoveryFactor;
        }
        recoveryStartTime = System.currentTimeMillis();

        if(!inRecoveryMode) {
            recoveryEndTime = System.currentTimeMillis() + recoveryBoostTime;
        }
        else{//trying to recover and shoot another ball
            recoveryEndTime = System.currentTimeMillis() + recoveryBoostTime * 1.2;
        }

        inRecoveryMode = true;
    }

    public void simpleTrigger(){
        recoveryStartTime = System.currentTimeMillis();
        recoveryEndTime = System.currentTimeMillis() + recoveryBoostTime;
    }

    public double getTopRecoveryFactor(){
        return topRecoveryFactor;
    }
    public double getBotRecoveryFactor(){
        return botRecoveryFactor;
    }


    public double getPredictedTopVel(){
        return predictedTopVel;
    }
    public double getPredictedBotVel(){
        return predictedBotVel;
    }


    public void setCustomPower(double lowPower, double highPower) {
        predictedBotVel = lowPower;
        predictedTopVel = highPower;
        if(runMode == RunMode.VelocityControl){
            low.set(lowPower);
            high.set(highPower + topVelocityOffset);//account for belted motor
        }
        else {
            low.set(lowPower);
            high.set(highPower);
        }
    }


    public void setRawPower(double lowPower, double highPower){
        low.set(lowPower);
        high.set(highPower);
    }

    public void resetRecoveryFactors(){
        currBotFactor = 1;
        currTopFactor = 1;
    }


    public double getDistance(Pose robotPose, ShootSide side){
        double xDist = Math.abs(robotPose.getX() - ((side == ShootSide.LEFT) ? leftShootPose.getX() : rightShootPose.getX()));
        double yDist = Math.abs(robotPose.getY() - ((side == ShootSide.LEFT) ? leftShootPose.getY() : rightShootPose.getY()));
        return Math.hypot(xDist, yDist);
    }

    public Pose getShootPose(ShootSide shootSide){
        return shootSide == ShootSide.LEFT ? leftShootPose : rightShootPose;
    }

    public void stopFlywheels() {
        low.motor.setPower(0);
        high.motor.setPower(0);
    }
}
