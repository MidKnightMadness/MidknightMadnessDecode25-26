package org.firstinspires.ftc.teamcode.commands.shooter;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.Timer;

public class FlywheelShootTimed extends CommandBase {

    TwoWheelShooter shooter;

    Follower follower;
    ShootSide shootSide;
    TwoWheelShooter.ShootDist shootDist;
    boolean mapDistToShoot;
    Timer timer;
    double startTime;
    boolean endFlywheelPower;
    double totalTime;
    public FlywheelShootTimed(TwoWheelShooter shooter, Follower follower, ShootSide shootSide, TwoWheelShooter.ShootDist shootDist, boolean mapDistToShoot, double totalTime, boolean endFlywheelPower){
        this.shooter = shooter;
        this.shootSide = shootSide;
        this.follower = follower;
        this.shootDist = shootDist;
        this.totalTime = totalTime;
        this.mapDistToShoot = mapDistToShoot;
        this.endFlywheelPower = endFlywheelPower;
    }
    @Override
    public void initialize(){
        timer = new Timer();
        startTime = timer.getTime();
    }

    @Override
    public void execute(){
        if(mapDistToShoot){
            follower.update();
            Pose robotPose = follower.getPose();
            double distToGoal = shooter.getDistance(robotPose, shootSide);
            shooter.setFlywheelStaticLUT(distToGoal, false, 0);
        }
        else{
            shooter.setFlywheelStaticPresets(shootDist, false, 0);
        }
    }

    @Override
    public boolean isFinished(){
        if (timer.getTime() - startTime >= totalTime) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        if(!endFlywheelPower) {
             shooter.stopFlywheels();
        }

    }
}
