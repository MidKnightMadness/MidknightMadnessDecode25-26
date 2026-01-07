package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.Timer;

@Config
@Configurable
public class ShootSeqCommand extends CommandBase {
    Spindexer spindexer;
    TwoWheelShooter shooter;
    SpindexerSpot[] seq;
    Follower follower;
    ShootSide shootSide;
    public int currBallIndex = 0;
    public static double angleTolerance = 20;
    boolean mapDistToShoot;
    Timer timer;

    public static double goToGreenSpotWait = 1000;
    public static double flywheelSpinupWait = 2500;
    public static double betweenShotsWait = 500;
    public static double betweenShotsWaitArr[] = new double[]{2000, 1000, 1000};
    boolean goneToStartSpot = false;
    double goneToStartSpotTime = 0;
    boolean flywheelSpinupStarted = false;

//    public boolean spinStarted = false;
    boolean spinEnded = false;
    boolean removedBall = false;
    double spinStartTime = 0;
    boolean lastBallRemoved = false;

    public boolean farthestMoved = false;
    boolean powerFlywheel;
    TwoWheelShooter.ShootDist shootDist;
    public static double maxTimeShoot = 8000;

    public static boolean voltageUse = false;
    public static boolean useLUT = true;
    public int getCurrBallIndex(){
        return currBallIndex;
    }

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

        if(powerFlywheel) {
            if (mapDistToShoot) {
                follower.update();
                shooter.setFlywheelStaticLUT(follower.getPose(), shootSide, false);
            } else {
                shooter.setFlywheelStaticPresets(shootDist, false);
            }
        }

        if(timer.getTime() < flywheelSpinupWait){
            return;
        }

        SpindexerSpot currSpot = seq[currBallIndex];

        spindexer.goToSpot(currSpot, SpotType.OUTTAKE, CRServoEx2.RunMode.OptimizedPositionalControl);

        if(!removedBall) {
            if(spindexer.isAtSpot(currSpot, SpotType.OUTTAKE)) {
                // Ball is in position to shoot!
                removedBall = true;
                spinStartTime = timer.getTime();  // ← START THE TIMER NOW

                if(currBallIndex == seq.length - 1) {
                    lastBallRemoved = true;
                }
            }
            return;
        }

        if(!spinEnded) {
            double waitTime = betweenShotsWaitArr[currBallIndex];
            if(timer.getTime() - spinStartTime >= waitTime) {
                // Shot complete, move to next ball
                currBallIndex++;
                removedBall = false;
                spinEnded = false;
            }
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
//        spindexer.goToSpot(SpindexerSpot.SPOT0, SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl);
        spindexer.getTurner().getServo().setPower(0);
    }
}
