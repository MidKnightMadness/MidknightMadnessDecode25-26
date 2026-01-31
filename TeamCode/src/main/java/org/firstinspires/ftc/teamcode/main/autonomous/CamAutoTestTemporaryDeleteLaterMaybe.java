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
    int limelightPipelineStart = 0;//CHANGE THIS

    @Override
    protected Command postMotifSequence() {
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new CamCommand(limelight, follower),//looks for balls, updates finalballlist
                    new WaitUntilCommand(() -> !CamCommand.finalBallList.isEmpty())//doesnt get past race group until cam finds things
                    //new RepeatCommand(strafe left and right) do this after testing if the auto actually works
                ),
                buildBallPathSequence()//drive and intake the balls
        );
    }

    private Command buildBallPathSequence() {
        return new ParallelCommandGroup(
            new AutoIntakeCommand(spindexer, intake, 0.5, 0.5),//intake while moving yk
            new FollowPathCommand(follower, CamCommand.getList(follower))//this gets patchain and stuff so its everything
        );
    }
    private Command strafeLeftRight(){
        Pose botPose = follower.getPose();
        Pose
    }

    @Override
    protected void initializeMechanisms() {
        intake = new Intake(hardwareMap, Intake.RunMode.RawPower);//init intak
        spindexer = new Spindexer(hardwareMap, true);//init spindexer
    }

    @Override
    protected void setupVision() {
        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);//init limelight
        limelight.pipelineSwitch(limelightPipelineStart);
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
        if(!prevVisionComplete && isVisionComplete()){//will true
//            if(postMotifSequence() != null) {
            preMotifSeq.cancel();
            follower.breakFollowing();
            schedule(new CamCommand(limelight, follower));//do it once and update finalballlist before, MAYBE DONT NEED
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
