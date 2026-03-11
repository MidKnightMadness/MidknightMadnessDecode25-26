package org.firstinspires.ftc.teamcode.commands.shooter;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.Timer;

public class WaitUntilShootReadyCommand extends CommandBase {

    TwoWheelShooter shooter;
    double endTime;
    Timer timer;
    boolean ready;
    double lowTolerance;
    double highTolerance;
    public WaitUntilShootReadyCommand(TwoWheelShooter shooter, double endTime, double lowTolerance, double highTolerance){
        this.shooter = shooter;
        this.endTime = endTime;
        this.lowTolerance = lowTolerance;
        this.highTolerance = highTolerance;
        timer = new Timer();
        addRequirements(shooter);
    }


    @Override
    public void initialize(){
        timer.restart();
    }



    @Override
    public void execute() {
//        if(shooter.readyToShoot(lowTolerance, highTolerance)){
//            ready = true;
//        }
    }

    @Override
    public boolean isFinished(){
        return (shooter.readyToShoot(lowTolerance, highTolerance) || timer.getTime() >= endTime);
    }

    @Override
    public void end(boolean interrupted){
    }
}
