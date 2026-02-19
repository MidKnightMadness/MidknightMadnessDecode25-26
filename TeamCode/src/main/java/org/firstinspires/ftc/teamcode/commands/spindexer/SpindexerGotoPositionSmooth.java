package org.firstinspires.ftc.teamcode.commands.spindexer;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerNonCR;
import org.firstinspires.ftc.teamcode.util.Angle;


public class SpindexerGotoPositionSmooth extends CommandBase {
    private final double targetPosition;//position in 0-1
    private final SpindexerNonCR spindexer;
    boolean first;

    double intermediateStep = 0;
    double previousSetPosition = 0;
    public double dir = 0;

    public SpindexerGotoPositionSmooth(SpindexerNonCR spindexer, double position, double intermediateStep) {
        this.targetPosition = position;
        this.spindexer = spindexer;
        this.intermediateStep = intermediateStep;
        addRequirements(this.spindexer);
    }

    @Override
    public void initialize(){
        previousSetPosition = spindexer.getServo().getPosition();
        dir = (previousSetPosition <= targetPosition) ? 1 : -1;
    }

    @Override
    public void execute() {
        double proposedPosition = previousSetPosition + intermediateStep * dir;
        if(dir == 1) {
            spindexer.setPosition(Math.min(proposedPosition, targetPosition));
        } else{
            spindexer.setPosition(Math.max(proposedPosition, targetPosition));
        }
        previousSetPosition = proposedPosition;
    }

    @Override
    public boolean isFinished() {
        return spindexer.isAtPosition(targetPosition);
    }


}
