package org.firstinspires.ftc.teamcode.commands.shooter;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.spindexer.SpindexerGotoSpot;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.Angle;


public class ShootColor extends SequentialCommandGroup {
    public ShootColor(
            Spindexer spindexer,
            TwoWheelShooter shooter,
            CRServoEx2.RunMode runMode,
            double dist,
            double finishedTimeThreshold
    ) {
        SpindexerSpot spot = spindexer.getNearestSpot(Angle.fromDegrees(0), SpotType.OUTTAKE);
        addCommands(
                new InstantCommand(() -> shooter.setFlywheelStaticLUT(dist, false, 0)),
                new SpindexerGotoSpot(spindexer, spot, SpotType.OUTTAKE, runMode, finishedTimeThreshold),
                new InstantCommand(shooter::stopFlywheels)
        );
    }
}
