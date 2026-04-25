package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.Timer;


public class TurretGotoPositionSmooth extends CommandBase {
    private final double targetPosition;//position in 0-1
    private final Turret turret;
    boolean first;

    double totalTime = 0;
    double previousSetPosition = 0;
    public double dir = 0;

    Timer timer;
    double startValue;
    public TurretGotoPositionSmooth(Turret turret, double position, double totalTime) {
        this.targetPosition = position;
        this.turret = turret;
        this.totalTime = totalTime;
        addRequirements(this.turret);
        timer = new Timer();
    }

    @Override
    public void initialize(){
        previousSetPosition = turret.getServoLeftPosition();
        startValue = previousSetPosition;
        timer.restart();
    }

    @Override
    public void execute() {
        double time = timer.getTime() / 1000.0;
        turret.setServos(startValue + (targetPosition - startValue) * time/totalTime);
    }

    @Override
    public boolean isFinished() {
        return timer.getTime() / 1000.0 > totalTime || turret.isAtPosition(targetPosition, true);
    }

    @Override
    public void end(boolean interrupted){
        if(!interrupted) {//TODO: SEE IF WORKS
            turret.setServos(targetPosition);
        }
    }


}
