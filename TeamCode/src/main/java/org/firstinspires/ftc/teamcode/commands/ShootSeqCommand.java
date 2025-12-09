package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.Timer;

@Config
@Configurable
public class ShootSeqCommand extends CommandBase {
    Spindexer spindexer;
    TwoWheelShooter shooter;
    SpindexerSpot[] seq;
    Follower follower;
    ShootSide shootSide;
    int currBallIndex = 0;
    public static double angleTolerance = 20;
    boolean mapDistToShoot;
    Timer timer;

    public static double goToGreenSpotWait = 1000;
    public static double flywheelSpinupWait = 1500;
    public static double betweenShotsWait = 1000;
    boolean goneToStartSpot = false;
    double goneToStartSpotTime = 0;
    boolean flywheelSpinupStarted = false;
    double flywheelSpinupStartTime = 0;

    public boolean spinStarted = false;
    boolean spinEnded = false;
    boolean removedBall = false;
    double spinStartTime = 0;
    boolean lastBallRemoved = false;

    public boolean farthestMoved = false;
    boolean powerFlywheel;
    TwoWheelShooter.ShootDist shootDist;
    public static double maxTimeShoot = 6700;
    public ShootSeqCommand(Spindexer spindexer, TwoWheelShooter shooter, SpindexerSpot[] seq, Follower follower, ShootSide shootSide, boolean mapDistToShoot, TwoWheelShooter.ShootDist shootDist, boolean powerFlywheel){
        this.spindexer = spindexer;
        this.shooter = shooter;
        this.seq = seq;
        this.follower = follower;
        this.shootSide = shootSide;
        this.shootDist = shootDist;
        this.powerFlywheel = powerFlywheel;
        this.mapDistToShoot = mapDistToShoot;
        addRequirements(spindexer, shooter);
        timer = new Timer();
    }
    @Override
    public void initialize(){
        timer.restart();
    }

    @Override
    public void execute(){
        if(currBallIndex >= seq.length) return;

        //have spindexer go to start spot if farthest from outtake
//        if(!goneToStartSpot) {
//            farthestMoved = false;
//            if(spindexer.farthestFromAngle(spindexer.getCurrentAngle(), SpotType.OUTTAKE) == seq[0]){
//                spindexer.goToSpot(seq[0], SpotType.PREOUTTAKE, CRServoEx2.RunMode.OptimizedPositionalControl);
//                farthestMoved = true;
//            }
//
//            if(!farthestMoved){//no need to go to green
//                goneToStartSpot = true;
//                goneToStartSpotTime = timer.getTime();
//                goToGreenSpotWait = 0;
//            }
//            else {
//                goneToStartSpot = true;
//                goneToStartSpotTime = timer.getTime();
//            }
//        }

//        if(!(timer.getTime() - goneToStartSpotTime >= goToGreenSpotWait)) {
//            return;
//        }

        //update distance to goal and reset flywheel powers/velocity
        if(powerFlywheel) {
            if (mapDistToShoot) {
                follower.update();
                Pose robotPose = follower.getPose();
                double distToGoal = shooter.getDistance(robotPose, shootSide);
                shooter.setFlywheelsPower(distToGoal);
            } else {
                if (shootDist == TwoWheelShooter.ShootDist.Close) {
                    shooter.setFlywheelsPower(TwoWheelShooter.ShootDist.Close, TwoWheelShooter.RunMode.RawPower);
                } else {
                    shooter.setFlywheelsPower(TwoWheelShooter.ShootDist.Far, TwoWheelShooter.RunMode.RawPower);
                }
            }
        }

        //start the flywheel
        if(!flywheelSpinupStarted){
            flywheelSpinupStarted = true;
            flywheelSpinupStartTime = timer.getTime();
        }

        if(!(flywheelSpinupStarted && timer.getTime() - flywheelSpinupStartTime > flywheelSpinupWait)){
            return;
        }
        //once flywheel gets to the right power/velocity, now go to each position w/ wait time for each wait position

        SpindexerSpot currSpot = seq[currBallIndex];
        spindexer.goToSpot(currSpot, SpotType.OUTTAKE, CRServoEx2.RunMode.OptimizedPositionalControl);
        if(!spinStarted){
            spinStarted = true;
        }

        //remove the ball if possible
        if(spinStarted && !removedBall){
            Angle diff = spindexer.getCurrentAngle().absGap(currSpot.getOuttakeAngle());
            if (diff.toDegrees() < angleTolerance) {
//                spindexer.removeBall(currSpot.getIndex());
                spindexer.getTurner().stop();
                removedBall = true;
                if (currBallIndex != seq.length - 1) {
//                shooter.triggerBallShot();
                }
            }
        }

        if(removedBall && !spinEnded) {//just removed a ball can perform wait time
            spinStartTime = timer.getTime();
            spinEnded = true;
        }

        if(currBallIndex == seq.length - 1 && removedBall){
            lastBallRemoved = true;
        }
        //move on, reset
        if(spinStarted && spinStartTime != 0 && timer.getTime() - spinStartTime > betweenShotsWait) {
            currBallIndex++;
            spinStarted = false;
            spinStartTime = 0;
            removedBall = false;
            spinEnded = false;
        }
    }

    @Override
    public boolean isFinished(){
        if((currBallIndex >= (seq.length) && lastBallRemoved) || timer.getTime() > maxTimeShoot) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        shooter.stopFlywheels();
        spindexer.goToSpot(SpindexerSpot.SPOT0, SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl);

    }
}
