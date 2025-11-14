package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.game.MotifEnums;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.game.ShootSide;

public class SpindexerShootContinuous extends SequentialCommandGroup {
    private final boolean useShooter;
    private final Spindexer spindexer;
    private final TwoWheelShooter shooter;
    private final Follower follower;
    private final ShootSide shootSide;

    // Expects this initial spot position:
    // Shooter
    //   P(0) P(120)
    //     G(240)
    public SpindexerShootContinuous(
            Spindexer spindexer,
            TwoWheelShooter shooter,
            Follower follower,
            MotifEnums.Motif motif,
            ShootSide shootSide,
            boolean useShooter
    ) {
        this.spindexer = spindexer;
        this.shooter = shooter;
        this.follower = follower;
        this.shootSide = shootSide;
        this.useShooter = useShooter;
        addCommands(
                new InstantCommand(() -> spindexer.initAngle(Angle.fromDegrees(60)))
        );
    }

    public SpindexerShootContinuous(Spindexer spindexer, MotifEnums.Motif motif) {
        this(spindexer, null, null, motif, null, false);
    }

    public SpindexerShootContinuous(Spindexer spindexer){
        this(spindexer, null, null, null, null, false);
    }

    @Override
    public void initialize() {
        if (useShooter) {
            addCommands(
                new InstantCommand(() -> shooter.setFlywheelsPower(follower.getPose(), shootSide)),
                new WaitCommand(1000)
            );
        }
        addCommands(new SpindexerSpinAngle(spindexer, Angle.fromDegrees(-480), 1));
        if (useShooter) new InstantCommand(shooter::stopFlywheels);

        super.initialize();
    }
}
