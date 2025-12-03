package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

public class SpindexerRawSequence extends SequentialCommandGroup {
    public static long waitMs = 200;
    public static long finalWaitMs = 500;

    public SpindexerRawSequence(
            Spindexer spindexer,
            SpindexerSpot[] sequence,
            SpotType spotType,
            CRServoEx2.RunMode runMode,
            double finishedTimeThreshold
    ) {
        for (int i = 0; i < sequence.length; i++) {
            if (i > 0) addCommands(new WaitCommand(waitMs));
            SpindexerSpot spot = sequence[i];
            addCommands(
                    new SpindexerGotoSpot(spindexer, spot, spotType, runMode, finishedTimeThreshold),
                    new InstantCommand(() -> spindexer.removeBall(spot.getIndex()))
            );
        }
        addCommands(new WaitCommand(finalWaitMs));}
}
