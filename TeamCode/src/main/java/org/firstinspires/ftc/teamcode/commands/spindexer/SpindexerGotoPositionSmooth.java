package org.firstinspires.ftc.teamcode.commands.spindexer;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.game.SpindexerSpotNonCR;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerNonCR;
import org.firstinspires.ftc.teamcode.util.Timer;


public class SpindexerGotoPositionSmooth extends CommandBase {
    private final double targetPosition;//position in 0-1
    private final SpindexerNonCR spindexer;

    double totalTime = 0;

    Timer timer;
    double startPosition;
    //total time in sec
    public SpindexerGotoPositionSmooth(SpindexerNonCR spindexer, int startSpot, int targetSpot, double totalTime) {
        this.targetPosition = SpindexerSpotNonCR.getPositionFromIndex(targetSpot, SpotType.INTAKE);
        this.startPosition = SpindexerSpotNonCR.getPositionFromIndex(startSpot, SpotType.INTAKE);
        this.spindexer = spindexer;
        this.totalTime = totalTime;
        addRequirements(this.spindexer);
        timer = new Timer();
    }

    @Override
    public void initialize(){
        timer.restart();
    }

    boolean start = false;
    @Override
    public void execute() {
        if(!start){
            timer.restart();
            start = true;
        }

        double time = timer.getTime() / 1000.0;

        spindexer.setDirectPosition(startPosition + (targetPosition - startPosition) * time/totalTime);
    }

    @Override
    public boolean isFinished() {
        return timer.getTime() / 1000.0 > totalTime;
    }

    @Override
    public void end(boolean interrupted){
        if(!interrupted) {//TODO: SEE IF WORKS
            spindexer.setDirectPosition(targetPosition);
        }
    }


}
