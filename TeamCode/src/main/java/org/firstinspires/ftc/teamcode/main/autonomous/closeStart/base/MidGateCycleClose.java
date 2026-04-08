package org.firstinspires.ftc.teamcode.main.autonomous.closeStart.base;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.game.IntakeLine;
import org.firstinspires.ftc.teamcode.game.SpotType;

public class MidGateCycleClose extends BaseAutoCloseFunctions{
    @Override
    protected Command postMotifSequence(){
        //temporarily turn it off to hand to localizer
        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        //GPP so that G is on the right side
                        shootPresetUnsorted(),

                        intake(IntakeLine.MID, false),
                        shootClose(IntakeLine.MID, maxWaitTillShoot, false),

                        intakeFromGate(),
                        shootFromGate(),


                        intake(IntakeLine.CLOSE, false),
                        shootClose(IntakeLine.CLOSE, maxWaitTillShoot, true),

                        new InstantCommand(()-> spindexer.setDirectPosition(0))
                ),
                new RunCommand(()-> follower.update())
        );
    }

    //only use limelight, no arducam setup
    @Override
    public void setupVision(){
    }

    @Override
    public void initialize_loop(){
        telemetry.addData("Motif Pattern", motifPattern);
        telemetry.addData("Spindexer spot", spindexer.getNearestIntakePosition(SpotType.INTAKE));
        telemetry.update();
    }
}
