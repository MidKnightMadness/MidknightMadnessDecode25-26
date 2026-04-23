package org.firstinspires.ftc.teamcode.commands.spindexer;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.game.SpindexerSpotNonCR;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerNonCR;
import org.firstinspires.ftc.teamcode.util.Timer;

public class OutakeSpotsRotation extends CommandBase {

    SpindexerNonCR spindexer;
    long spotTime;
    Timer shootTimer;
    int startSpot;

    public OutakeSpotsRotation(SpindexerNonCR spindexerNonCR, int startSpot, long spotTime){
        this.spindexer = spindexerNonCR;
        this.spotTime = spotTime;
        this.startSpot = startSpot;
        this.currSpot = startSpot;
        shootTimer = new Timer();
        addRequirements(this.spindexer);
    }

    public int currSpot;

    double shootTimerTime = 0;
    double startPosition = SpindexerSpotNonCR.getPositionFromIndex(currSpot, SpotType.INTAKE) - 60.0 / SpindexerNonCR.totalDegrees;
    double interval = 120.0 / SpindexerNonCR.totalDegrees;

    boolean firstRun = true;

    @Override
    public void initialize(){
        shootTimer = new Timer();
    }

    @Override
    public void execute() {
        if (firstRun) {
            shootTimer.restart();
            firstRun = false;
        }
        if(shootTimer.getTime() - shootTimerTime >= spotTime){
            spindexer.setDirectPosition(startPosition - interval * (startSpot - currSpot));
            shootTimerTime = shootTimer.getTime();
            currSpot = currSpot -1;
        }
    }

    public double getShootTimerTime(){
        return shootTimer.getTime();
    }
    @Override
    public boolean isFinished() {
        return currSpot == startSpot - 3;
    }
}
