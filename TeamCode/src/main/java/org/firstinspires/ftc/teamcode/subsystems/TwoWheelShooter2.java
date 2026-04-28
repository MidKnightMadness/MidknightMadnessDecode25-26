package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
public class TwoWheelShooter2 extends SubsystemBase {

    public enum RunMode {
        RawPower,
        VelocityControl
    }

    public enum ShootDist{
        Close,
        Far
    }


    public static double transferPower = 1.0;
    public static double transferVelocity = 1300;//1300
    //DEFAULT GAINS
    public static double[] kTransferGains = new double[]{0.0313141, 0.000353782, 0};
    public static double[] pidTransferGains = new double[]{0.0012, 0, 0};
    public static double[] kBotGains = new double[]{0.058, 0.000393, 0};
    public static double[] pidTopGains = new double[]{0.008, 0, 0};
    public static double[] pidBotGains = new double[]{0.008, 0, 0};
    public static double[] kTopGains = new double[]{0.033, 0.000427, 0};

    //AGGRESSIVE GAINS
    public static double[] pidBotAggressiveGains = new double[]{0.04, 0, 0};
    public static double[] pidTopAggressiveGains = new double[]{0.04, 0, 0};
    public static double[] pidTransferAggressiveGains = new double[]{0.008, 0, 0};
    public boolean useAggressiveRecovery = true;
    public boolean inRecoveryMode = false;

    public final MotorEx low;
    public final MotorEx high;
    public final MotorEx transfer;
    public RunMode shooterRunMode;//low & high
    public RunMode transferRunMode;

    public static Pose leftShootPose = new Pose(0, 144, Math.toRadians(90));
    public static Pose rightShootPose = new Pose(141.5, 144, Math.toRadians(90));

    public static boolean lowMotorDirForward = false;
    public static boolean highMotorDirForward = true;
    public static boolean transferMotorDirForward = true;
    double predictedTopVel = 2000;
    double predictedBotVel = 2000;
    double predictedTransferVel = 2000;
    double predictedTopPower = 0.8;
    double predictedBotPower = 0.8;
    double currTopFactor = 1;
    double currBotFactor = 1;

    double actualRecoveryTime = 0;
    HardwareMap map;
    //ready to shoot
    public static double readyToShootBot = 50;
    public static double readyToShootTop = 50;
    public static double readyToShootTransfer = 50;
    double topMultiplier = 0;
    double botMultiplier = 0;
    public AimCalculator aimCalculator;

    public static class AimCalculator {
        InterpLUT distToLowVel;
        InterpLUT distToHighVel;
        InterpLUT distToKCorrection;

        //ticks in sec for 3: 1 direct driven gear ratios
        public static int iterations = 10; // For tuning targetDistance
        public static double[] dist =                {40,  50,  60,  70,  80,  90,  100, 112, 120, 128, 130, 145,  156.0, 180};//inches
//        public static double[] bottomVel =           {530, 560, 590, 600, 630, 650, 700, 700, 710, 720, 730, 750,  760,  780};
//        public static double[] topVel =              {650, 610, 590, 610, 630, 680, 720, 760, 840, 890, 960, 1000, 1030, 1060};
        public static double[] bottomVel =           {530, 540, 550, 560, 570, 570, 570, 600, 610, 620, 630, 650,  660,  680};
        public static double[] topVel =              {650, 650, 650, 670, 690, 750, 850, 860, 940, 990, 1060, 1100, 1130, 1200};
        public static double[] velCorrectionFactor = {0.4, 0.5, 0.6,0.54, 0.85,0.9,0.96,1.02,1.07, 1.1, 1.12,1.14,1.15,1.16}; // take time in the air and then subtract a bit

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
    public static double transferShotDropThreshold = 50;
    public double bottomError;
    public double topError;
    public double transferError;
    public void updateRecoveryState() {
        bottomError = low.getVelocity() - predictedBotVel;
        topError = high.getVelocity() - predictedTopVel;
        transferError = transfer.getVelocity() - predictedTransferVel;

        if(useAggressiveRecovery) {
            if (Math.abs(bottomError) > shotDropThreshold && bottomError < 0) {
                setAggressiveGainsBottom();
            } else {
                resetDefaultGainsBottom();
            }

            if (Math.abs(topError) > shotDropThreshold  && topError < 0) {
                setAggressiveGainsTop();
            } else {
                resetDefaultGainsTop();
            }

            if(Math.abs(transferError) > transferShotDropThreshold){
                setAggressiveGainsTransfer();
            } else{
                resetDefaultGainsTransfer();
            }
        }
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

    public void resetDefaultGains(){
        low.setVeloCoefficients(pidBotGains[0], pidBotGains[1], pidBotGains[2]);
        low.setFeedforwardCoefficients(kBotGains[0], kBotGains[1], kBotGains[2]);
        high.setVeloCoefficients(pidTopGains[0], pidTopGains[1], pidTopGains[2]);
        high.setFeedforwardCoefficients(kTopGains[0], kTopGains[1], kTopGains[2]);
        transfer.setFeedforwardCoefficients(kTransferGains[0], kTransferGains[1], kTransferGains[2]);
        transfer.setVeloCoefficients(pidTransferGains[0], pidTransferGains[1], pidTransferGains[2]);
    }

    public void resetDefaultGainsBottom(){
        low.setVeloCoefficients(pidBotGains[0], pidBotGains[1], pidBotGains[2]);
    }
    public void resetDefaultGainsTop(){
        high.setVeloCoefficients(pidTopGains[0], pidTopGains[1], pidTopGains[2]);
    }

    public void resetDefaultGainsTransfer(){
        transfer.setVeloCoefficients(pidTransferGains[0], pidTransferGains[1], pidTransferGains[2]);
    }
    public void setAggressiveGainsBottom(){
        low.setVeloCoefficients(pidBotAggressiveGains[0], pidBotAggressiveGains[1], pidBotAggressiveGains[2]);
    }
    public void setAggressiveGainsTop(){
        high.setVeloCoefficients(pidTopAggressiveGains[0], pidTopAggressiveGains[1], pidTopAggressiveGains[2]);
    }
    public void setAggressiveGainsTransfer(){
        transfer.setVeloCoefficients(pidTransferAggressiveGains[0], pidTransferAggressiveGains[1], pidTransferAggressiveGains[2]);
    }


    public void setRunMode(RunMode shooterRunMode, RunMode transferRunMode) {
        this.shooterRunMode = shooterRunMode;
        this.transferRunMode = transferRunMode;

        low.setRunMode(shooterRunMode == RunMode.RawPower ? Motor.RunMode.RawPower : Motor.RunMode.VelocityControl);
        high.setRunMode(shooterRunMode == RunMode.RawPower ? Motor.RunMode.RawPower : Motor.RunMode.VelocityControl);
        transfer.setRunMode(transferRunMode == RunMode.RawPower ? Motor.RunMode.RawPower : Motor.RunMode.VelocityControl);
    }

    public void setFlywheelLUT(Follower follower, ShootSide shootSide, double currVolt){
        setFlywheelStaticLUT(getDistance(follower.getPose(), shootSide), currVolt);
    }

    public void setFlywheelStaticLUT(double distToGoal, double currVolt){
        setFlywheel(distToGoal, null, true, currVolt);
    }
    public void setFlywheel(double dist, ShootDist shootDist, boolean useLUT, double currVolt){
        updateRecoveryState();
        double botVelocity, topVelocity;
        if(dist >= AimCalculator.dist[AimCalculator.dist.length - 1]){
            botVelocity = AimCalculator.bottomVel[AimCalculator.bottomVel.length - 1];
            topVelocity = AimCalculator.topVel[AimCalculator.topVel.length - 1];
        } else if(dist <= AimCalculator.dist[0]) {
            botVelocity = AimCalculator.bottomVel[0];
            topVelocity = AimCalculator.topVel[0];
        }
        else {
            botVelocity = aimCalculator.distToLowVel.get(dist);
            topVelocity = aimCalculator.distToHighVel.get(dist);
        }
        if(shooterRunMode != RunMode.VelocityControl) setRunMode(RunMode.VelocityControl, transferRunMode);

        if(shooterRunMode == RunMode.VelocityControl) {
            predictedBotVel = botVelocity;
            predictedTopVel = topVelocity;
        } else{
            predictedBotPower = botVelocity;
            predictedTopPower = topVelocity;
        }

        low.set(botVelocity, 1, currVolt);
        high.set(topVelocity, 1, currVolt);
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

        double botVelocity, topVelocity;
        double[] aimData = aimCalculator.targetPowersHeading(
                robotPose,
                robotVel,
                getShootPoseNew(robotPose, shootSide)
        );
        botVelocity = aimData[0];
        topVelocity = aimData[1];
        if (shooterRunMode != RunMode.VelocityControl) setRunMode(RunMode.VelocityControl, transferRunMode);


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
        return Math.abs(low.getVelocity() - predictedBotVel) <= readyToShootBot &&
                Math.abs(high.getVelocity() - predictedTopVel) <= readyToShootTop &&
                Math.abs(transfer.getVelocity() - predictedTransferVel)<= readyToShootTransfer;
    }
    public boolean readyToShoot(double lowTolerance, double highTolerance, double transferTolerance){
        return Math.abs(low.getVelocity() - predictedBotVel) <= lowTolerance &&
                Math.abs(high.getVelocity() - predictedTopVel) <= highTolerance &&
                Math.abs(transfer.getVelocity() - predictedTransferVel) <= transferTolerance;
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
            low.set(lowPower, 1,currVolt);
            high.set(highPower, 1, currVolt);//account for belted motor
        }
        else {
            low.set(lowPower);
            high.set(highPower);
        }
    }

    public void setTransferPower(double power, double currVolt){
        predictedTransferVel = power;
        if(transferRunMode == RunMode.VelocityControl){
            transfer.set(power, currVolt);
        }
        else {
            transfer.set(power);
        }
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
        boolean close = false;
        if(robotPose.getY() > 50){//close side
            close = true;
        }
        Pose control1;
        if (shootSide == ShootSide.LEFT) {
            control1 = close ? new Pose(0,140) : new Pose(0, 144);
        } else {
            control1 = close ? new Pose(141.5,140) : new Pose(136, 144);

        }
        double angle1 = ExtraFns.getTargetAngle(robotPose, control1);

        Vector displacement = new Vector(
                robotPose.distanceFrom(getShootPose(shootSide)),
                angle1
        );

        return robotPose.plus(new Pose(
                displacement.getXComponent(),
                displacement.getYComponent(),
                displacement.getTheta()
        ));

    }
//
    public static double getShootHeading(Pose robotPose, ShootSide shootSide) {
        boolean close = false;
        if(robotPose.getY() > 50){//close side
            close = true;
        }
        Pose control1;
        if (shootSide == ShootSide.LEFT) {
            control1 = close ? new Pose(0,144) : new Pose(2, 142);//5
        } else {
            control1 = close ? new Pose(141.5,144) : new Pose(136, 144);
        }
        double angle1 = ExtraFns.getTargetAngle(robotPose, control1);
        Vector displacement = new Vector(
                robotPose.distanceFrom(getShootPose(shootSide)),
                angle1
        );
        return displacement.getTheta();
    }

    public void stopFlywheels() {
        low.setPower(0);
        high.setPower(0);
    }
    public void stopAll() {
        low.setPower(0);
        high.setPower(0);
        transfer.setPower(0);
    }
}
