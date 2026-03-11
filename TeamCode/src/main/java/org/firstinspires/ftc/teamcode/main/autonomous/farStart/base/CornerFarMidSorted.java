package org.firstinspires.ftc.teamcode.main.autonomous.farStart.base;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;


import org.firstinspires.ftc.teamcode.game.IntakeLine;
import org.firstinspires.ftc.teamcode.tests.camera.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.util.ConfigNames;


public class CornerFarMidSorted extends BaseAutoFarFunctions {

    @Override
    protected Command postMotifSequence(){
//        limelight.stop();
//        limelight.shutdown();
        arducam.stop();
        //temporarily turn it off to hand to localizer
        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        //GPP so that G is on the right side
                        shootPresetSorted(),

                        intakeCorner(true),
                        shootFromLines(IntakeLine.CORNER, maxWaitTillShoot),

                        intake(IntakeLine.FAR, true),
                        shootFromLines(IntakeLine.FAR, maxWaitTillShoot),

                        intake(IntakeLine.MID, true),
                        shootFromLines(IntakeLine.MID, maxWaitTillShoot),

                        new ParallelCommandGroup(
                                park(),
                                new InstantCommand(()-> spindexer.setDirectPosition(0))//reset spindexer
                        )
                ),
                new RunCommand(()-> follower.update())
        );

    }

    @Override
    public void setupVision(){
        arducam = new AprilTagWebcam();
        arducam.init(hardwareMap, ConfigNames.arducam);
//
//        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);//init limelight
//        limelight.pipelineSwitch(objectDetectionPipeline);
//        limelight.start();

    }

}
