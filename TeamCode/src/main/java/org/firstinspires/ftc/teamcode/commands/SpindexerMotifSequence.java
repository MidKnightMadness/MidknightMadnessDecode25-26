package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.game.MotifEnums;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

public class SpindexerMotifSequence extends SequentialCommandGroup {
    Spindexer spindexer;
    MotifEnums.Motif motif;
    CRServoEx2.RunMode runMode;
    double finishedTimeThreshold;
    SpindexerRawSequence command;

    public SpindexerMotifSequence(
            Spindexer spindexer,
            MotifEnums.Motif motif,
            CRServoEx2.RunMode runMode,
            double finishedTimeThreshold
    ) {
        this.spindexer = spindexer;
        this.motif = motif;
        this.runMode = runMode;
        this.finishedTimeThreshold = finishedTimeThreshold;
    }

    // Build everything at runtime
    @Override
    public void initialize() {
        SpindexerSpot[] sequence = spindexer.getOptimalSequence(motif);
        addCommands(
                command = new SpindexerRawSequence(
                spindexer, sequence, SpotType.OUTTAKE,
                runMode, finishedTimeThreshold)
        );;
        super.initialize();
    }

    @Override
    public boolean isFinished(){
        return command.isFinished();
    }
}
