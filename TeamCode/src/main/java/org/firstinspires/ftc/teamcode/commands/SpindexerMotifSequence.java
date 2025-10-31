package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.motif.MotifEnums;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

public class SpindexerMotifSequence extends SequentialCommandGroup {
    Spindexer spindexer;
    MotifEnums.Motif motif;
    CRServoEx2.RunMode runMode;
    double finishedTimeThreshold;

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
        int[] sequence = spindexer.getOptimalSequence(motif);
        addCommands(new SpindexerRawSequence(
                spindexer, sequence,
                runMode, finishedTimeThreshold)
        );;
        super.initialize();
    }
}
