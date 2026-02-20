package org.firstinspires.ftc.teamcode.commands.spindexer;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpindexerSpotNonCR;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerNonCR;
import org.firstinspires.ftc.teamcode.util.Timer;

public class OutakeSpotsRotation extends CommandBase {
    SpindexerSpotNonCR spindexerSpot;
    int currSpindexerOutakeSpot;
    SpindexerNonCR spindexer;
    int dir;
    double timeWait;

    Timer timer;
    double prevTime = 0;
    boolean first = true;
    double numSpot = 1;
    public OutakeSpotsRotation(SpindexerNonCR spindexerNonCR, SpindexerSpotNonCR startSpot, int dir, double timeWait){
        this.spindexer = spindexerNonCR;
        this.spindexerSpot = startSpot;
        this.dir = dir;
        this.timeWait = timeWait;
        currSpindexerOutakeSpot = startSpot.getIndex();
        addRequirements(this.spindexer);
    }
    @Override
    public void initialize() {
        timer.restart();
        prevTime = 0;
    }

    @Override
    public void execute() {
        if(first){
            spindexer.setDirectPosition(SpindexerSpotNonCR.fromIndex(currSpindexerOutakeSpot).getOuttakePositionSolo());
            first = false;
            prevTime = timer.getTime();
        }

        if(timer.getTime() - prevTime >= timeWait && numSpot < 3){
            currSpindexerOutakeSpot += dir;
            if(currSpindexerOutakeSpot < 0){
                currSpindexerOutakeSpot += 3;
            } else if(currSpindexerOutakeSpot > 3){
                currSpindexerOutakeSpot -= 3;
            }
            spindexer.setDirectPosition(SpindexerSpotNonCR.fromIndex(currSpindexerOutakeSpot).getOuttakePositionSolo());
            prevTime = timer.getTime();
            numSpot++;
        }

        else if(timer.getTime() - prevTime >= timeWait && numSpot == 3){
            numSpot = 4;
        }

    }
    @Override
    public boolean isFinished() {
        return numSpot == 4;
    }
}
