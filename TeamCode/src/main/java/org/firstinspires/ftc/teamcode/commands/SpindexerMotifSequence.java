package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.motif.MotifEnums;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

public class SpindexerMotifSequence extends SpindexerRawSequence {
    public static long waitMs = 200;
    public static long finalWaitMs = 500;

    public SpindexerMotifSequence(
            Spindexer spindexer,
            MotifEnums.Motif motif,
            CRServoEx2.RunMode runMode
    ) {
        super(spindexer, spindexer.getOptimalSequence(motif), runMode, 0);
    }
}
