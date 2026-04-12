package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;


import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.MotorEx;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.util.ExtraFns;

import java.util.Map;

@Configurable
@Config
//naohs custom shooter w/ transfer wheel
public class TwoWheelShooter2 extends SubsystemBase {

    public enum RunMode {
        RawPower,
        VelocityControl
    }

    public enum ShootDist{
        Close,
        Far
    }


    public static double transferPower = 0.7;
    public static double transferVelocity = 1300;
    //DEFAULT GAINS
    public static double[] kTransferGains = new double[]{0.0313141, 0.000353782, 0};
    public static double[] pidTransferGains = new double[]{0.002, 0, 0};
    public static double[] kBotGains = new double[]{0.058, 0.000393, 0};
    public static double[] pidTopGains = new double[]{0.008, 0, 0};
    public static double[] pidBotGains = new double[]{0.008, 0, 0};
    public static double[] kTopGains = new double[]{0.033, 0.000427, 0};

    //AGGRESSIVE GAINS
    public static double[] pidBotAggressiveGains = new double[]{0.01, 0, 0};
    public static double[] pidTopAggressiveGains = new double[]{0.01, 0, 0};
    public static double[] pidTransferAggressiveGains = new double[]{0.003, 0, 0};
    public boolean useAggressiveRecovery = true;
    public boolean inRecoveryMode = false;
    //AGGRESSIVE GAINS: FOR RECOVERY - gain scheduling


    InterpLUT distToLowVel;
    InterpLUT distToHighVel;


    public final MotorEx low;
    public final MotorEx high;
    public final MotorEx transfer;
    public RunMode shooterRunMode;//low & high
    public RunMode transferRunMode;

    public static double minDistanceThreshold = 10;//INCH
    public static Pose leftShootPose = new Pose(0, 144, Math.toRadians(90));
    public static Pose rightShootPose = new Pose(144, 144, Math.toRadians(90));
    public static Map<Double, Double> grToMultiplier = Map.of(
            3., 2.89,
            4., 3.61,
            5., 5.23
    ); // Unused for now
    public static double gearRatio = 3;

    public static boolean lowMotorDirForward = false;
    public static boolean highMotorDirForward = true;
    public static boolean transferMotorDirForward = true;
    double predictedTopVel = 2000;
    double predictedBotVel = 2000;
    double predictedTransferVel = 2000;
    double predictedTopPower = 0.8;
    double predictedBotPower = 0.8;

    public static double topRecoveryFactor = 1.1;//TUNE
    public static double botRecoveryFactor = 1.1;//TUNE
    double currTopFactor = 1;
    double currBotFactor = 1;

    double actualRecoveryTime = 0;
    double recoveryStartTime = 0;
    double recoveryEndTime = 0;

    public static double targetVoltage = 12.5;

    //    public static double[] closeTargetVelocities = new double[] {1800, 1900};
    public static double[] closeTargetVelocities = new double[] {1600, 1750};
    public static double[] farTargetVelocities = new double[]{1700, 2000};
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
    public AimCalculator aimCalculator;

    public static class AimCalculator {
        InterpLUT distToLowVel;
        InterpLUT distToHighVel;
        InterpLUT distToKCorrection;

        //ticks in sec for 3: 1 direct driven gear ratios
        public static int iterations = 10; // For tuning targetDistance
        public static double[] dist = {60, 70, 80, 90, 100, 112, 128, 149.5, 156.0};//inches
        public static double[] bottomVel = {700, 740, 790, 820, 860, 900, 930, 980, 1000};
        public static double[] topVel = {800, 820, 860, 890, 905, 940, 980, 1000};
        public static double[] velCorrectionFactor = {0.8, 0.85, 0.9, 0.95, 1.0, 1.07, 1.15, 1.25, 1.3}; // take time in the air and then subtract a bit

        public AimCalculator() {
            distToLowVel = new InterpLUT();
            distToHighVel = new InterpLUT();
            distToKCorrection = new InterpLUT();
            for (int i = 0; i < dist.length; i++) {
                distToLowVel.add(dist[i], bottomVel[i]);
                distToHighVel.add(dist[i], topVel[i]);
                distToKCorrection.add(dist[i], velCorrectionFactor[i]);
            }
            distToLowVel.createLUT();
            distToHighVel.createLUT();
            distToKCorrection.createLUT();
        }

        /**
         * Calculate target powers and heading.<br>
         * Algorithm:<br>
         * 1. Act like you're aiming for a distance d<br>
         * 2. Simulate error for shooting with distance d, making sure the parallel component of the error is 0<br>
         * 3. Update d by adding the error<br>
         * 4. Repeat steps 1-3 to iterative refine d
         * @param pose     The robot pose
         * @param velocity The robot velocity
         * @return bottom velocity, top velocity, heading
         */
        public double[] targetPowersHeading(Pose pose, Vector velocity, Pose targetPose) {
            Vector gap = targetPose.minus(pose).getAsVector();
            Vector dirParallel = gap.normalize();
            Vector dirPerp = dirParallel.copy();
            dirPerp.rotateVector(-Math.PI / 2);
            double velParallel = velocity.dot(dirParallel); // component parallel to line from robot to target pose
            double velPerp = velocity.dot(dirPerp); // left is negative, right is positive

            double realDist = targetPose.distanceFrom(pose);
            double targetDist = realDist;
            double headingCorrection = 0;
            for (int i = 0; i < iterations; i++) {
                // Steps 1-2: simulation
                targetDist = MathUtils.clamp(targetDist, dist[0]+0.01, dist[dist.length - 1]-0.01); // find a better way later
                double kCorr = distToKCorrection.get(targetDist);
                double predict = targetDist + kCorr * velParallel;
                double predictPerp = kCorr * velPerp;
                double predictParallel = Math.sqrt(
                        Math.max(0, predict * predict - predictPerp * predictPerp)
                );
                headingCorrection = Math.atan2(predictPerp, predictParallel);
                // Steps 3-4: error and update
                double distanceIdeal = (realDist / predictParallel) * predict;
                // d + correction + error = ideal d
                // (d + error) + correction = ideal d, update d += error
                targetDist += distanceIdeal - predict;
            }

            targetDist = MathUtils.clamp(targetDist, dist[0]+0.01, dist[dist.length - 1]-0.01); // find a better way later

            return new double[]{
                    distToLowVel.get(targetDist),
                    distToHighVel.get(targetDist),
                    MathFunctions.normalizeAngle(gap.getTheta() + headingCorrection)
            };
        }
    }


    public static double shotDropThreshold = 50;
    public double bottomError;
    public double topError;
    public double transferError;
    public void updateRecoveryState() {
        bottomError = low.getVelocity() - predictedBotVel;
        topError = high.getVelocity() - predictedTopVel;
        transferError = transfer.getVelocity() - predictedTransferVel;
        if(useAggressiveRecovery) {
            if (Math.abs(bottomError) > shotDropThreshold) {
                setAggressiveGainsBottom();
            } else {
                resetDefaultGainsBottom();
            }

            if (Math.abs(topError) > shotDropThreshold) {
                setAggressiveGainsTop();
            } else {
                resetDefaultGainsTop();
            }

            if(Math.abs(transferError) > shotDropThreshold){
                setAggressiveGainsTransfer();
            } else{
                resetDefaultGainsTransfer();
            }
        }
    }

    public void setAggressiveRecovery(boolean agressiveUse){
        useAggressiveRecovery = agressiveUse;
    }


    public double getTargetVoltage(){
        return targetVoltage;
    }

    public TwoWheelShooter2(HardwareMap hardwareMap, RunMode shooterRunMode, RunMode transferRunMode) {
        this.map = hardwareMap;
        low = new MotorEx(hardwareMap, ConfigNames.lowFlywheelMotor);
        high = new MotorEx(hardwareMap, ConfigNames.highFlywheelMotor);
        transfer = new MotorEx(hardwareMap, ConfigNames.transferMotor);

        low.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        high.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        setRunMode(shooterRunMode, transferRunMode);

        aimCalculator = new AimCalculator();

        low.motor.setDirection(lowMotorDirForward ? DcMotorEx.Direction.FORWARD : DcMotorEx.Direction.REVERSE);
        high.motor.setDirection(highMotorDirForward ? DcMotorEx.Direction.FORWARD : DcMotorEx.Direction.REVERSE);
        transfer.motor.setDirection(transferMotorDirForward ? DcMotorEx.Direction.FORWARD : DcMotorEx.Direction.REVERSE);
        resetDefaultGains();
    }

    public void resetEncoders() {
        low.stopAndResetEncoder();
        high.stopAndResetEncoder();
    }
    public void resetDefaultGains(){
        inRecoveryMode = false;
        low.setVeloCoefficients(pidBotGains[0], pidBotGains[1], pidBotGains[2]);
        low.setFeedforwardCoefficients(kBotGains[0], kBotGains[1], kBotGains[2]);
        high.setVeloCoefficients(pidTopGains[0], pidTopGains[1], pidTopGains[2]);
        high.setFeedforwardCoefficients(kTopGains[0], kTopGains[1], kTopGains[2]);
        transfer.setFeedforwardCoefficients(kTransferGains[0], kTransferGains[1], kTransferGains[2]);
        transfer.setVeloCoefficients(pidTransferGains[0], pidTransferGains[1], pidTransferGains[2]);
    }

    public void resetDefaultGainsBottom(){
        inRecoveryMode = false;
        low.setVeloCoefficients(pidBotGains[0], pidBotGains[1], pidBotGains[2]);
        low.setFeedforwardCoefficients(kBotGains[0], kBotGains[1], kBotGains[2]);
    }
    public void resetDefaultGainsTop(){
        inRecoveryMode = false;
        high.setVeloCoefficients(pidTopGains[0], pidTopGains[1], pidTopGains[2]);
        high.setFeedforwardCoefficients(kTopGains[0], kTopGains[1], kTopGains[2]);
    }

    public void resetDefaultGainsTransfer(){
        inRecoveryMode = false;
        transfer.setVeloCoefficients(pidTransferGains[0], pidTransferGains[1], pidTransferGains[2]);
        transfer.setFeedforwardCoefficients(kTransferGains[0], kTransferGains[1], kTransferGains[2]);
    }
    public void setAggressiveGainsBottom(){
        inRecoveryMode = true;
        low.setVeloCoefficients(pidBotAggressiveGains[0], pidBotAggressiveGains[1], pidBotAggressiveGains[2]);
    }
    public void setAggressiveGainsTop(){
        inRecoveryMode = true;
        high.setVeloCoefficients(pidTopAggressiveGains[0], pidTopAggressiveGains[1], pidTopAggressiveGains[2]);
    }
    public void setAggressiveGainsTransfer(){
        inRecoveryMode = true;
        transfer.setVeloCoefficients(pidTransferAggressiveGains[0], pidTransferAggressiveGains[1], pidTransferAggressiveGains[2]);
    }

    public double getCurrVoltage(){
        return currVolt;
    }


    public void setRunMode(RunMode shooterRunMode, RunMode transferRunMode) {
        this.shooterRunMode = shooterRunMode;
        this.transferRunMode = transferRunMode;

        low.setRunMode(shooterRunMode == RunMode.RawPower ? Motor.RunMode.RawPower : Motor.RunMode.VelocityControl);
        high.setRunMode(shooterRunMode == RunMode.RawPower ? Motor.RunMode.RawPower : Motor.RunMode.VelocityControl);
        transfer.setRunMode(transferRunMode == RunMode.RawPower ? Motor.RunMode.RawPower : Motor.RunMode.VelocityControl);
    }

    public void setPid(double kp, double ki, double kd) {
        low.setVeloCoefficients(kp, ki, kd);
        high.setVeloCoefficients(kp, ki, kd);
        transfer.setVeloCoefficients(kp, ki, kd);
    }
    public void setFeedforward(double kS, double kV, double kA){
        low.setFeedforwardCoefficients(kS, kV, kA);
        high.setFeedforwardCoefficients(kS, kV, kA);
        transfer.setFeedforwardCoefficients(kS, kV, kA);
    }


    //thresholds whether robot is currently moving
    public void setFlywheelPresets(ShootDist shootDist, Follower follower, ShootSide shootSide,  boolean voltageUse, double currVolt){
//        Vector robotVelocity = follower.getVelocity();
//        boolean isMoving = true;
//        if(robotVelocity.getMagnitude() <= velMovingThreshold){
//            isMoving = false;
//        }
//        if(!isMoving){
        setFlywheelStaticPresets(shootDist, voltageUse, currVolt);
//        } else{
//            setFlywheelMovingPresets(follower.getPose(), shootDist, shootSide, robotVelocity, voltageUse);
//        }
    }

    //thresholds whether robot is currently moving
    public void setFlywheelLUT(Follower follower, ShootSide shootSide, boolean voltageUse, double currVolt){
//        Vector robotVelocity = follower.getVelocity();
//        boolean isMoving = true;
//        if(robotVelocity.getMagnitude() <= velMovingThreshold){
//            isMoving = false;
//        }
//        if(!isMoving){
//            setFlywheelMovingLUT(follower.getPose(), shootSide, robotVelocity, voltageUse, currVolt);
//        } else{
        setFlywheelStaticLUT(getDistance(follower.getPose(), shootSide), voltageUse, currVolt);
//        }
    }



    public void setFlywheelStaticPresets(ShootDist shootDist, boolean voltageUse, double currVolt) {//assuming facing the shooting area
        setFlywheel(0, shootDist, 0, voltageUse, false, currVolt);
    }
    public void setFlywheelStaticLUT(Pose robotPose, ShootSide shootSide, boolean voltageUse, double currVolt){
        setFlywheel(getDistance(robotPose, shootSide), null, 0, voltageUse, true, currVolt);
    }
    public void setFlywheelStaticLUT(double distToGoal, boolean voltageUse, double currVolt){
        setFlywheel(distToGoal, null, 0, voltageUse, true, currVolt);
    }
    public void setFlywheelMovingLUT(Pose robotPose, ShootSide shootSide, Vector velVector, boolean voltageUse, double currVolt){
        double dist = getDistance(robotPose, shootSide);
        Pose targetPose = getShootPose(shootSide);
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();

        double vparallel = (velVector.getXComponent() * dx + velVector.getYComponent() * dy)
                / dist;

        setFlywheel(dist, null, vparallel, voltageUse, true, currVolt);
    }

    public void setFlywheelMovingPresets(Pose robotPose, ShootDist shootDist, ShootSide shootSide, Vector velVector, boolean voltageUse, double currVolt){

        double dist = getDistance(robotPose, shootSide);
        Pose targetPose = getShootPose(shootSide);
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();

        double vparallel = (velVector.getXComponent() * dx + velVector.getYComponent() * dy)
                / dist;

        setFlywheel(dist, shootDist, vparallel, voltageUse, false, currVolt);
    }
    public void setFlywheel(double dist, ShootDist shootDist, double vParallel, boolean voltageUse, boolean useLUT, double currVolt){
//        double ratio = voltageUse ? (targetVoltage / currVolt) : 1;
//        ratio = Math.min(ratio, 1.35);
        double ratio = 1;

        double botVelocity, topVelocity;
        if(useLUT){
            if(dist >= 156){
                botVelocity = AimCalculator.bottomVel[AimCalculator.bottomVel.length - 1];
                topVelocity = AimCalculator.topVel[AimCalculator.topVel.length - 1];
            } else if(dist <= 60) {
                botVelocity = AimCalculator.bottomVel[0];
                topVelocity = AimCalculator.topVel[0];
            }
            else {
                botVelocity = aimCalculator.distToLowVel.get(dist);
                topVelocity = aimCalculator.distToHighVel.get(dist);
            }
            if(shooterRunMode != RunMode.VelocityControl) setRunMode(RunMode.VelocityControl, transferRunMode);
        }
        else{
            double[] preset;
            if(shooterRunMode == RunMode.VelocityControl){
                preset = (shootDist == ShootDist.Close) ? closeTargetVelocities : farTargetVelocities;
            } else{
                preset = (shootDist == ShootDist.Close) ? closeTargetPowers : farTargetPowers;
            }
            botVelocity = preset[0];
            topVelocity = preset[1];
        }

        botVelocity -= kBotShootMovingFactor * vParallel;
        topVelocity -= kTopShootMovingFactor * vParallel;

        topMultiplier = ratio * currTopFactor;
        botMultiplier = ratio * currBotFactor;

        if(shooterRunMode == RunMode.VelocityControl) {
            predictedBotVel = botVelocity;
            predictedTopVel = topVelocity;
        } else{
            predictedBotPower = botVelocity;
            predictedTopPower = topVelocity;
        }

        low.set(botVelocity, botMultiplier, currVolt);
        high.set(topVelocity, topMultiplier, currVolt);
        updateRecoveryState();
    }


    public void setTransfer(double targetOutput){
        transfer.set(targetOutput);
    }
    /**
     * Cuberobot simulation go brrrrrrr
     *
     * @param robotPose The current robot pose
     * @param robotVel The current robot velocity
     * @param shootSide The shooting side
     * @param currVolt Current voltage of robot
     * @return The target heading for the robot
     */
    public double setFlywheelNew(Pose robotPose, Vector robotVel, ShootSide shootSide, double currVolt){
        updateRecoveryState();
//        double ratio = voltageUse ? (targetVoltage / currVolt) : 1;
//        ratio = Math.min(ratio, 1.35);
        double ratio = 1;

        double botVelocity, topVelocity;
        double[] aimData = aimCalculator.targetPowersHeading(
                robotPose,
                robotVel,
                getShootPoseNew(robotPose, shootSide)
        );
        botVelocity = aimData[0];
        topVelocity = aimData[1];
        if (shooterRunMode != RunMode.VelocityControl) setRunMode(RunMode.VelocityControl, transferRunMode);

        topMultiplier = ratio * currTopFactor;
        botMultiplier = ratio * currBotFactor;

        if(shooterRunMode == RunMode.VelocityControl) {
            predictedBotVel = botVelocity;
            predictedTopVel = topVelocity;
        } else{
            predictedBotPower = botVelocity;
            predictedTopPower = topVelocity;
        }

        low.set(botVelocity, botMultiplier, currVolt);
        high.set(topVelocity, topMultiplier, currVolt);

        return aimData[2];
    }

    public boolean readyToShoot(){
        return Math.abs(low.getVelocity() - predictedBotVel) <= velBotTolerance &&
                Math.abs(high.getVelocity() - predictedTopVel) <= velTopTolerance;
    }
    public boolean readyToShoot(double lowTolerance, double highTolerance){
        return Math.abs(low.getVelocity() - predictedBotVel) <= lowTolerance &&
                Math.abs(high.getVelocity() - predictedTopVel) <= highTolerance;
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


    public void setCustomPower(double lowPower, double highPower, double currVolt) {
        predictedBotVel = lowPower;
        predictedTopVel = highPower;
        if(shooterRunMode == RunMode.VelocityControl){
            low.set(lowPower, currVolt);
            high.set(highPower, currVolt);//account for belted motor
        }
        else {
            low.set(lowPower);
            high.set(highPower);
        }
    }

    public void setTransferPower(double power, double currVolt){
        predictedTransferVel = power;
        if(transferRunMode == RunMode.VelocityControl){
            transfer.set(power, 1, currVolt);
        }
        else {
            transfer.set(power);
        }
    }


    public void setRawPower(double lowPower, double highPower){
        low.set(lowPower, 0);
        high.set(highPower, 0);
    }


    public double getDistance(Pose robotPose, ShootSide side){
        double xDist = Math.abs(robotPose.getX() - ((side == ShootSide.LEFT) ? leftShootPose.getX() : rightShootPose.getX()));
        double yDist = Math.abs(robotPose.getY() - ((side == ShootSide.LEFT) ? leftShootPose.getY() : rightShootPose.getY()));
        return Math.hypot(xDist, yDist);
    }

    public static Pose getShootPose(ShootSide shootSide){
        return shootSide == ShootSide.LEFT ? leftShootPose : rightShootPose;
    }

    public static Pose getShootPoseNew(Pose robotPose, ShootSide shootSide) {
        Pose control1, control2;
        if (shootSide == ShootSide.LEFT) {
            control1 = new Pose(14, 144);
            control2 = new Pose(0, 130);
        } else {
            control1 = new Pose(130, 144);
            control2 = new Pose(144, 130);
        }
        double angle1 = ExtraFns.getTargetAngle(robotPose, control1);
        double angle2 = ExtraFns.getTargetAngle(robotPose, control2);
        Vector displacement = new Vector(
                robotPose.distanceFrom(getShootPose(shootSide)),
                (angle1 + angle2) / 2
        );
        return robotPose.plus(new Pose(
                displacement.getXComponent(),
                displacement.getYComponent(),
                displacement.getTheta()
        ));
    }
//
    public static double getShootHeading(Pose robotPose, ShootSide shootSide) {
        Pose control1, control2;
        if (shootSide == ShootSide.LEFT) {
            control1 = new Pose(14, 144);
            control2 = new Pose(0, 130);
        } else {
            control1 = new Pose(130, 144);
            control2 = new Pose(144, 130);
        }

        double angle1 = ExtraFns.getTargetAngle(robotPose, control1);
        double angle2 = ExtraFns.getTargetAngle(robotPose, control2);
        Vector displacement = new Vector(
                robotPose.distanceFrom(getShootPose(shootSide)),
                (angle1 + angle2) / 2
        );
        return displacement.getTheta();
    }

    public void stopFlywheels() {
        low.setPower(0);
        high.setPower(0);
    }
}
