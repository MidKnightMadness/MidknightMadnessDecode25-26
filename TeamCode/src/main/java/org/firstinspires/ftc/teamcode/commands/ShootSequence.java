package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.teamcode.motif.MotifEnums;
import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.ShootSide;

@Config
@Configurable
public class ShootSequence extends CommandBase {
    public static long powerFlywheelWaitMs = 200;
    public static long finalWaitMs = 900;
    public static long swapWaitMs = 700;

    public ShootSequence(
            Spindexer spindexer,
            TwoWheelShooter shooter,
            Ramp ramp,
            MotifEnums.Motif motif,
            CRServoEx.RunMode runMode,
            Pose robotPose,
            ShootSide side
    ) {
        int[] sequence = spindexer.getOptimalSequence(motif);

        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(() -> shooter.setFlywheelPower(robotPose, side)),
                new InstantCommand(ramp::setRestPos)
            ),
            new WaitCommand(powerFlywheelWaitMs)
        );
        for (int i = 0; i < sequence.length; i++) {
            if (i > 0)
                new ParallelCommandGroup(
                    new InstantCommand(ramp::setRestPos),
                    new WaitCommand(swapWaitMs)
                );
            int spot = sequence[i];
            new SequentialCommandGroup(
                new SpindexerGotoSpot(spindexer, spot, runMode),
                new ParallelCommandGroup(
                    new InstantCommand(ramp::setLowerPos),
                    new InstantCommand(() -> spindexer.removeBall(spot))
                )
            );
        }
        new SequentialCommandGroup(
                new WaitCommand(finalWaitMs),
                new InstantCommand(shooter::stopFlywheels)
        );

        addRequirements(spindexer, shooter, ramp);
    }
}
