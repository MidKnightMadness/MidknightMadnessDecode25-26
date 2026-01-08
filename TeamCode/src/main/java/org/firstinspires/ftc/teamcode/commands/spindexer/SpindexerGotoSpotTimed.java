package org.firstinspires.ftc.teamcode.commands.spindexer;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.Timer;

public class SpindexerGotoSpotTimed extends CommandBase {
    private boolean wasFinished;
    SpindexerSpot spot;
    private final Spindexer spindexer;
    private SpotType spotType;
    private CRServoEx2.RunMode runMode;
    private final Timer finishedTimer;
    private final double finishedTimeThreshold;
    double maxTime;

    public SpindexerGotoSpotTimed(
            Spindexer spindexer,
            SpindexerSpot spot,
            SpotType spotType,
            CRServoEx2.RunMode runMode,
            double finishedTimeThreshold,
            double maxTime
    ) {
        this.spot = spot;
        this.spindexer = spindexer;
        this.runMode = runMode;
        this.spotType = spotType;
        this.finishedTimeThreshold = finishedTimeThreshold;
        finishedTimer = new Timer();
        addRequirements(this.spindexer);
    }


    @Override
    public void initialize(){
        finishedTimer.restart();
    }
    @Override
    public void execute() {
        spindexer.goToSpot(spot, spotType, runMode);
    }

    @Override
    public boolean isFinished() {
        boolean atSpot = spindexer.isAtSpot(spot, spotType);
//        if (atSpot) {
//            if (runMode == CRServoEx2.RunMode.RawPower) {
//                runMode = CRServoEx2.RunMode.OptimizedPositionalControl;
//            }
//            if (!wasFinished) finishedTimer.restart();
//            if (finishedTimer.getTime() > finishedTimeThreshold) {
//                spindexer.getTurner().stop();
//                return true;
//            }
//        }
        wasFinished = atSpot;

        return atSpot || finishedTimer.getTime() >= maxTime;
    }


    @Override
    public void end(boolean interrupted){
//        spindexer.getTurner().setRunMode(CRServoEx2.RunMode.RawPower);
        spindexer.getTurner().getServo().setPower(0);
    }
}
