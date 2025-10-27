package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.Angle;

public class SpindexerGotoAngle extends CommandBase {
    private final Angle angle;
    private final Spindexer spindexer;
    private final CRServoEx.RunMode runMode;

    public SpindexerGotoAngle(Spindexer spindexer, Angle angle, CRServoEx.RunMode runMode) {
        this.angle = angle;
        this.spindexer = spindexer;
        this.runMode = runMode;
        addRequirements(this.spindexer);
    }

    @Override
    public void execute() {
        spindexer.goToAngle(angle, runMode);
    }

    @Override
    public boolean isFinished() {
        // Make sure you stop no matter what
        spindexer.goToAngle(angle, CRServoEx.RunMode.OptimizedPositionalControl);
        return spindexer.isAtAngle(angle);
    }
}
