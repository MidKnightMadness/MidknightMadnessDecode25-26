package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.motif.MotifEnums;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.Angle;

public class ShootHardcode extends SequentialCommandGroup {
    // Expects this initial spot position:
    //     Shooter
    //   P(0) G(120)
    //     P(240)
    Spindexer spindexer;
    TwoWheelShooter shooter;
    boolean isClose;

    public ShootHardcode(
            Spindexer spindexer,
            TwoWheelShooter shooter,
            MotifEnums.Motif motif,
            boolean isClose
    ) {
        spindexer.initAngle(Angle.fromDegrees(60));
        int[] sequence;
        if (motif == MotifEnums.Motif.GPP || motif == MotifEnums.Motif.NONE) {
            sequence = new int[] { 1, 2, 0 };
        } else if (motif == MotifEnums.Motif.PGP) {
            sequence = new int[] { 0, 1, 2 };
        } else {
            sequence = new int[] { 0, 2, 1 };
        }
        addCommands(
                new InstantCommand(() -> shooter.setFlywheelsPower(isClose)),
                new SpindexerRawSequence(spindexer, sequence, CRServoEx2.RunMode.OptimizedPositionalControl, 0),
                new WaitCommand(500),
                new InstantCommand(shooter::stopFlywheels)
        );

        addRequirements(spindexer, shooter);
    }
}
