package org.firstinspires.ftc.teamcode.commands.spindexer;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerNonCR;
import org.firstinspires.ftc.teamcode.util.Angle;

public class SpindexerGotoPosition extends CommandBase {
    private final double position;//position in 0-1
    private final SpindexerNonCR spindexer;
    boolean first;

    public SpindexerGotoPosition(SpindexerNonCR spindexer, double position) {
        this.position = position;
        this.spindexer = spindexer;
        addRequirements(this.spindexer);
    }

    @Override
    public void execute() {
        if(!first) {
            spindexer.setDirectPosition(position);
            first = true;
        }
    }

    @Override
    public boolean isFinished() {
        return spindexer.isAtPosition(position);
    }


}
