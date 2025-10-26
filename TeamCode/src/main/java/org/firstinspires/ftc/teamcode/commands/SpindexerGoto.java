package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

public class SpindexerGoto extends CommandBase {
    private final int spot;
    private final Spindexer spindexer;

    public SpindexerGoto(Spindexer spindexer, int spot) {
        this.spot = spot;
        this.spindexer = spindexer;
        addRequirements(this.spindexer);
    }

    @Override
    public void execute() {
        spindexer.goToSpot(spot);
    }

    @Override
    public boolean isFinished() {
        return spindexer.isAtSpot(spot);
    }
}
