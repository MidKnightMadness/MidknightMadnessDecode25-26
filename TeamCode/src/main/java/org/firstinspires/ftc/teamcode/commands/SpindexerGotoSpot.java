package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

public class SpindexerGotoSpot extends CommandBase {
    private final int spot;
    private final Spindexer spindexer;
    private final CRServoEx2.RunMode runMode;

    public SpindexerGotoSpot(Spindexer spindexer, int spot, CRServoEx2.RunMode runMode) {
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
        // Make sure you stop no matter what
        return spindexer.isAtSpot(spot);
    }
}
