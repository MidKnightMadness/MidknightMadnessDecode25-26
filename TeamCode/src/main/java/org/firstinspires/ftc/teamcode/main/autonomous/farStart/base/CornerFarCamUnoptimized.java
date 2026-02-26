package org.firstinspires.ftc.teamcode.main.autonomous.farStart.base;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;


import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommandNonCR;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.MotifEnums;
import org.firstinspires.ftc.teamcode.game.SpindexerSpotNonCR;
import org.firstinspires.ftc.teamcode.commands.pathing.BuildPath;
import org.firstinspires.ftc.teamcode.game.IntakeLine;
import org.firstinspires.ftc.teamcode.util.ConfigNames;


//contrary to base corner far auto(also corner far cam optimized) -> doens't do path splitting for intake camera work
//straight line from strafe to end
public class CornerFarCamUnoptimized extends BaseAutoFarFunctions {
   @Override
   public Command postMotifSequence(){
       //temporarily turn it off to hand to localizer
       return new ParallelCommandGroup(
               new SequentialCommandGroup(
                       //GPP so that G is on the right side
                       shootPresetUnsorted(),
//                    shootPresetSorted(),

                       intakeCorner(false),
                       shootFromLines(IntakeLine.CORNER, maxWaitTillShoot),

                       intake(IntakeLine.FAR, false),
                       shootFromLines(IntakeLine.FAR, maxWaitTillShoot),

                       cameraWork(),
//                       cameraWork(),
                       //come back second time(if have time)

                       new ParallelCommandGroup(
                               park(),
                               new InstantCommand(()-> spindexer.setDirectPosition(0))//reset spindexer
                       )
               ),
               new RunCommand(()-> follower.update())
       );
   }

   @Override
   public Command intakeCameraBalls() {
       buildPath = new BuildPath(follower, cam, targetx1, shootPose);
       return new SequentialCommandGroup(
               new InstantCommand(() -> pushUpServo.setDown()),
               new InstantCommand(()-> spindexer.setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE})),
               new ParallelRaceGroup(
                       new AutoIntakeCommandNonCR(spindexer, intake, intakePower, inBetweenTime, true, hardwareMap, SpindexerSpotNonCR.SPOT1, 1),
                       new SequentialCommandGroup(
                               new DeferredCommand(() -> buildPath, null),
                               new DeferredCommand(()-> new ConditionalCommand(
                                       new InstantCommand(()-> cameraForwardPathChain1 = buildPath.getPathChain()),
                                       new InstantCommand(),
                                       () -> buildPath.pathCreated), null),
                               new DeferredCommand(() -> new FollowPathCommand(follower, cameraForwardPathChain1, true, autoCameraDrivePower), null).withTimeout(3000)
                           )
               )
       );
   }

    //only use limelight, no arducam setup
    @Override
    public void setupVision(){
        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);//init limelight
        limelight.pipelineSwitch(objectDetectionPipeline);
        limelight.start();
    }

    @Override
    public void initialize_loop(){
    }
}
