package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.Angle;

public class ShootColor extends SequentialCommandGroup {
    public static long finalWaitMs = 500;

    public ShootColor(
            Spindexer spindexer,
            TwoWheelShooter shooter,
            CRServoEx2.RunMode runMode,
            double dist
    ) {
        int spot = spindexer.getNearestSpotIndex(Angle.fromDegrees(0));
        addCommands(
                new InstantCommand(() -> shooter.setFlywheelsPower(dist)),
                new SpindexerGotoSpot(spindexer, spot, runMode),
                new WaitCommand(finalWaitMs),
                new InstantCommand(shooter::stopFlywheels)
        );

        addRequirements(spindexer, shooter);
    }
}
