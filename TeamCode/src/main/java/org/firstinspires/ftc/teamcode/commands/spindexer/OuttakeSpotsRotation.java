package org.firstinspires.ftc.teamcode.commands.spindexer;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.game.SpindexerSpotNonCR;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerNonCR;
import org.firstinspires.ftc.teamcode.util.Timer;

public class OuttakeSpotsRotation extends CommandBase {

    SpindexerNonCR spindexer;
    long timePerShot;
    Timer shootTimer;

    double startSpot;
    boolean waitEnd;
    double waitTime;

    public OuttakeSpotsRotation(SpindexerNonCR spindexerNonCR, int startSpot, long spotTime, long waitTime){
        this.spindexer = spindexerNonCR;
        this.timePerShot = spotTime;
        this.startSpot = SpindexerSpotNonCR.getPositionFromIndex(startSpot, SpotType.INTAKE) - 60.0 / SpindexerNonCR.totalDegrees;
        this.waitTime = waitTime;
        shootTimer = new Timer();
        addRequirements(this.spindexer);
    }

    public OuttakeSpotsRotation(SpindexerNonCR spindexerNonCR, int startSpot, long spotTime){
        this(spindexerNonCR, startSpot, spotTime, 0);
    }

    public int rotationIndex = 0;
    double lastShotTime = 0;
    double interval = 120.0 / SpindexerNonCR.totalDegrees;
    @Override
    public void initialize(){ }

    boolean firstRun = true;
    double endTime = -1;


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

        spindexer.setPosition(startSpot - interval * rotationIndex);

        if (rotationIndex >= 2 && endTime == -1) {
            endTime = shootTimer.getTime();
        }
    }

    public double getLastShotTime(){
        return shootTimer.getTime();
    }
    @Override
    public boolean isFinished() {
        return rotationIndex >= 2 && endTime != -1 && shootTimer.getTime() - endTime >= waitTime;
    }
}
