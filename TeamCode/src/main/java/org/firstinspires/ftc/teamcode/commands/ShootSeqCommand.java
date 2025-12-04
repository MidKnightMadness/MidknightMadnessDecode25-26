package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.Angle;

@Config
@Configurable
public class ShootSeqCommand extends CommandBase {
    Spindexer spindexer;
    TwoWheelShooter shooter;
    SpindexerSpot[] seq;
    Follower follower;
    ShootSide shootSide;
    int currBallIndex = 0;
    boolean goingToSpot = false;
    double angleTolerance = 10;
    boolean mapDistToShoot;
    public ShootSeqCommand(Spindexer spindexer, TwoWheelShooter shooter, SpindexerSpot[] seq, Follower follower, ShootSide shootSide, boolean mapDistToShoot){
        this.spindexer = spindexer;
        this.shooter = shooter;
        this.seq = seq;
        this.follower = follower;
        this.shootSide = shootSide;
        this.mapDistToShoot = mapDistToShoot;
        addRequirements(spindexer, shooter);
    }

    @Override
    public void execute(){
        if(currBallIndex >= seq.length) return;
        follower.update();
        Pose robotPose = follower.getPose();
        double distToGoal = shooter.getDistance(robotPose, shootSide);

        if(mapDistToShoot){
            shooter.setFlywheelsPower(distToGoal);
        }
        else{
            if(distToGoal >= 85){
                shooter.setFlywheelsPower(TwoWheelShooter.ShootDist.Far);
            }
            else{
                shooter.setFlywheelsPower(TwoWheelShooter.ShootDist.Close);
            }
        }


        SpindexerSpot currSpot = seq[currBallIndex];

        if(!goingToSpot){
            spindexer.goToSpot(currSpot, SpotType.OUTTAKE, CRServoEx2.RunMode.OptimizedPositionalControl);
            goingToSpot = true;
        }

        Angle diff = spindexer.getCurrentAngle().sub(currSpot.getOuttakeAngle());
        if(diff.toDegrees() > angleTolerance){
            spindexer.removeBall(currSpot.getIndex());
            if(currBallIndex != seq.length -1){
                shooter.triggerBallShot();
            }
            currBallIndex++;
            goingToSpot = false;
        }
    }

    @Override
    public boolean isFinished(){
        return currBallIndex >= seq.length;
    }

    @Override
    public void end(boolean interrupted){
        shooter.stopFlywheels();
        spindexer.getTurner().stop();
    }
}
