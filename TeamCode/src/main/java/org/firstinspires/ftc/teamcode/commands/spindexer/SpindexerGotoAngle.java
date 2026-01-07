package org.firstinspires.ftc.teamcode.commands.spindexer;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.Angle;

public class SpindexerGotoAngle extends CommandBase {
    private final Angle angle;//convert angle from 0 - 360 to -180 - 180
    private final Spindexer spindexer;
    private final CRServoEx2.RunMode runMode;

    public SpindexerGotoAngle(Spindexer spindexer, Angle angle, CRServoEx2.RunMode runMode) {
        this.angle = angle.wrap();//normalize angle
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
        return spindexer.isAtAngle(angle);
    }


}
