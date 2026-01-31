package org.firstinspires.ftc.teamcode.main.autonomous;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.camera.CamCommand;
import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.pathing.SchedulePathTo;
import org.firstinspires.ftc.teamcode.main.autonomous.BaseAuto;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.ConfigNames;

import java.util.ArrayList;
import java.util.Comparator;

public class CamAutoTest extends BaseAuto {//THIS ONES CHATGPt dont use it its not good use the other one

    @Override
    protected void setupVision() {
        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);

    }

    @Override
    protected boolean isVisionComplete() {
        // Finish when Limelight has detected balls
        return !CamCommand.finalBallList.isEmpty();
    }

    @Override
    protected Command postMotifSequence() {
        Pose robotPose = follower.getPose();

        // Copy detected balls and clear the global list
        ArrayList<Pose> balls = new ArrayList<>(CamCommand.finalBallList);
        CamCommand.finalBallList.clear();

        // Sort balls by distance from robot
        balls.sort(Comparator.comparingDouble(a ->
                Math.hypot(a.getX() - robotPose.getX(), a.getY() - robotPose.getY())
        ));

        // Only take the 3 closest balls
        int ballsToTake = Math.min(3, balls.size());
        SequentialCommandGroup seq = new SequentialCommandGroup();
        for (int i = 0; i < ballsToTake; i++) {
            Pose ballPose = balls.get(i);

            // Move to ball
            SchedulePathTo moveCommand = new SchedulePathTo(follower, ballPose).setMaxPower(0.5);

            // Intake while moving (parallel)
            AutoIntakeCommand intakeCommand = new AutoIntakeCommand(spindexer, intake, 0.8, 0.5);

            seq.addCommands(new ParallelCommandGroup(moveCommand, intakeCommand));
        }
        ;
        return seq;
    }

    @Override
    protected Pose getStartPose() {
        return new Pose(0, 0, 0); // adjust to your field start position
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
