package org.firstinspires.ftc.teamcode.commands.spindexer;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.game.SpindexerSpotNonCR;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerNonCR;
import org.firstinspires.ftc.teamcode.util.Timer;

public class OutakeSpotsRotation extends CommandBase {

    SpindexerNonCR spindexer;
    long timePerShot;
    Timer shootTimer;

    double startPosition;

    public OutakeSpotsRotation(SpindexerNonCR spindexerNonCR, int startPosition, long spotTime){
        this.spindexer = spindexerNonCR;
        this.timePerShot = spotTime;
        this.startPosition = SpindexerSpotNonCR.getPositionFromIndex(startPosition, SpotType.INTAKE) - 60.0 / SpindexerNonCR.totalDegrees;

        shootTimer = new Timer();
        addRequirements(this.spindexer);
    }

    public int rotationIndex = 0;
    double lastShotTime = 0;
    double interval = 120.0 / SpindexerNonCR.totalDegrees;
    @Override
    public void initialize(){
        shootTimer = new Timer();
    }

    boolean firstRun = true;

    @Override
    public void execute() {
        if (firstRun) {
            shootTimer.restart();
            firstRun = false;
        }

        if(shootTimer.getTime() - lastShotTime >= timePerShot){
            rotationIndex++;
            lastShotTime = shootTimer.getTime();
        }

        spindexer.setPosition(startPosition - interval * rotationIndex);
    }

    public double getLastShotTime(){
        return shootTimer.getTime();
    }
    @Override
    public boolean isFinished() {
        return rotationIndex >= 2;
    }
}
