package org.firstinspires.ftc.teamcode.commands.shooter;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.spindexer.SpindexerRawSequence;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.game.MotifEnums;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;

public class ShootHardcode extends SequentialCommandGroup {
    // Expects this initial spot position:
    //     Shooter
    //   P(0) G(120)
    //     P(240)

    public ShootHardcode(//first balls to shoot
            Spindexer spindexer,
            TwoWheelShooter shooter,
            MotifEnums.Motif motif,
            double distToGoal
    ) {
        SpindexerSpot[] sequence;
        int momentum;
        if (motif == MotifEnums.Motif.GPP || motif == MotifEnums.Motif.NONE) {
            sequence = new SpindexerSpot[] {SpindexerSpot.fromIndex(1), SpindexerSpot.fromIndex(2), SpindexerSpot.fromIndex(0)};
            momentum = 1;
        } else if (motif == MotifEnums.Motif.PGP) {
            sequence = new SpindexerSpot[] {SpindexerSpot.fromIndex(0), SpindexerSpot.fromIndex(1), SpindexerSpot.fromIndex(2)};
            momentum = 1;
        } else {
            sequence = new SpindexerSpot[] {SpindexerSpot.fromIndex(0), SpindexerSpot.fromIndex(2), SpindexerSpot.fromIndex(1)};
            momentum = -1;
        }
        addCommands(
                new InstantCommand(() -> {
                    shooter.setFlywheelStaticLUT(distToGoal, false, 0);
                }),
                new SpindexerRawSequence(spindexer, sequence, SpotType.OUTTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 0),
                new InstantCommand(() -> spindexer.spin(momentum)),
                new WaitCommand(250),
                new InstantCommand(() -> spindexer.getTurner().getServo().setPower(0)),
                new InstantCommand(shooter::stopFlywheels)
        );

        addRequirements(spindexer, shooter);
    }
}
