package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.motif.MotifEnums;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;

public class ShootSequence extends SequentialCommandGroup {
    public static long waitMs = 200;
    public static long finalWaitMs = 500;

    public ShootSequence(
            Spindexer spindexer,
            TwoWheelShooter shooter,
            MotifEnums.Motif motif,
            double dist
    ) {
        int[] sequence = spindexer.getOptimalSequence(motif);
        addCommands(new InstantCommand(() -> shooter.setFlywheels(dist)));
        for (int i = 0; i < sequence.length; i++) {
            if (i > 0) addCommands(new WaitCommand(waitMs));
            addCommands(new SpindexerGoto(spindexer, sequence[i]));
        }
        addCommands(
                new WaitCommand(finalWaitMs),
                new InstantCommand(shooter::stopFlywheels)
        );

        addRequirements(spindexer, shooter);
    }
}
