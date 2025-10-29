package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
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
            CRServoEx2.RunMode runMode,
            double dist
    ) {
        int[] sequence = spindexer.getOptimalSequence(motif);
        addCommands(new InstantCommand(() -> shooter.setFlywheelsPower(dist)));
        for (int i = 0; i < sequence.length; i++) {
            if (i > 0) addCommands(new WaitCommand(waitMs));
            int spot = sequence[i];
            addCommands(
                    new SpindexerGotoSpot(spindexer, spot, runMode),
                    new InstantCommand(() -> spindexer.removeBall(spot))
            );
        }
        addCommands(
                new WaitCommand(finalWaitMs),
                new InstantCommand(shooter::stopFlywheels)
        );

        addRequirements(spindexer, shooter);
    }
}
