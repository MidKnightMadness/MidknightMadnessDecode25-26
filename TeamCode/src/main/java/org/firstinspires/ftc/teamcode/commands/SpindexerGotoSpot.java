package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.Timer;

public class SpindexerGotoSpot extends CommandBase {
    private boolean wasFinished;
    private final int spot;
    private final Spindexer spindexer;
    private CRServoEx2.RunMode runMode;
    private final Timer finishedTimer;
    private final double finishedTimeThreshold;

    public SpindexerGotoSpot(
            Spindexer spindexer,
            int spot,
            CRServoEx2.RunMode runMode,
            double finishedTimeThreshold
    ) {
        this.spot = spot;
        this.spindexer = spindexer;
        this.runMode = runMode;
        this.finishedTimeThreshold = finishedTimeThreshold;
        finishedTimer = new Timer();
        addRequirements(this.spindexer);
    }

    @Override
    public void execute() {
        spindexer.goToSpot(spot, runMode);
    }

    @Override
    public boolean isFinished() {
        boolean atSpot = spindexer.isAtSpot(spot);
        if (atSpot) {
            if (runMode == CRServoEx2.RunMode.RawPower) {
                runMode = CRServoEx2.RunMode.OptimizedPositionalControl;
            }
            if (!wasFinished) finishedTimer.restart();
            if (finishedTimer.getTime() > finishedTimeThreshold) {
                spindexer.getTurner().getServo().setPower(0);
                return true;
            }
        }
        wasFinished = atSpot;
        return false;
    }
}
