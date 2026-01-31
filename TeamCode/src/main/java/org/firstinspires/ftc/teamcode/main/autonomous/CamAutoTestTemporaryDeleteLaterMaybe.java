package org.firstinspires.ftc.teamcode.main.autonomous;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.geometry.BezierLine;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.camera.CamCommand;
import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.pathing.SchedulePathTo;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.ConfigNames;

@Autonomous(name = "Camtestautomaybe", group = "Test")
public class CamAutoTestTemporaryDeleteLaterMaybe extends BaseAuto {

    @Override
    protected Command postMotifSequence() {
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new CamCommand(limelight, follower),                          // run camera
                    new WaitUntilCommand(() -> !CamCommand.finalBallList.isEmpty()) // wait for results
                ),
                buildBallPathSequence()
        );
    }

    private Command buildBallPathSequence() {
        // Create a sequential command group
        return new ParallelCommandGroup(
            new AutoIntakeCommand(spindexer, intake, 0.5, 0.5),
            new FollowPathCommand(follower, CamCommand.getList(follower))
        );
    }

    @Override
    protected void initializeMechanisms() {
        intake = new Intake(hardwareMap, Intake.RunMode.RawPower);
        spindexer = new Spindexer(hardwareMap, true);
    }

    @Override
    protected void setupVision() {
        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);
        limelight.pipelineSwitch(0);
        limelight.start();
    }
    @Override
    public void run(){
        super.run();
        if(!gameTimerStarted){
            gameTimer.restart();
            gameTimerStarted = true;
        }
        update();
        if(!prevVisionComplete && isVisionComplete()){
//            if(postMotifSequence() != null) {
            preMotifSeq.cancel();
            follower.breakFollowing();
            schedule(new CamCommand(limelight, follower));
            schedule(postMotifSequence());
//            }
            prevVisionComplete = true;
        }

        //   if(postMotifSequence().isFinished()){
//            if(goToIntakeLine()!= null){
//                schedule(goToIntakeLine());
//            }
        //    }
//        if (timer.getTime() >= maxTimeMs) requestOpModeStop();
        writeValues();
        updateTelemetry();
    }


}
