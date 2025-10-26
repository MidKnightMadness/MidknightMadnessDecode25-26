package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

public class SpindexerGoto extends CommandBase {
    private final int spot;
    private final Spindexer spindexer;
    private final CRServoEx.RunMode runMode;

    public SpindexerGoto(Spindexer spindexer, int spot, CRServoEx.RunMode runMode) {
        this.spot = spot;
        this.spindexer = spindexer;
        this.runMode = runMode;
        addRequirements(this.spindexer);
    }

    @Override
    public void execute() {
        spindexer.goToSpot(spot, runMode);
    }

    @Override
    public boolean isFinished() {
        return spindexer.isAtSpot(spot);
    }
}
