package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

public class SchedulePathTo extends SequentialCommandGroup {
    Follower follower;
    Pose targetPose;
    double headingConstraint;
    double timeOutConstraint;
    double translationalConstraint;
    public SchedulePathTo(Follower follower, Pose targetPose, double headingConstraint, double timeOutConstraint, double translationalConstraint){
        this.follower = follower;
        this.targetPose = targetPose;
        this.headingConstraint = headingConstraint;
        this.timeOutConstraint = timeOutConstraint;
        this.translationalConstraint = translationalConstraint;
    }

    FollowPathCommand followCommand;
    @Override
    public void initialize(){
        follower.update();;
        Pose currentPose = follower.getPose();

        PathChain pathChain = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, targetPose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
                .setHeadingConstraint(headingConstraint)
                .setTimeoutConstraint(timeOutConstraint)
                .setTranslationalConstraint(translationalConstraint)
                .build();

        followCommand = new FollowPathCommand(follower, pathChain);
        addCommands(followCommand);
        super.initialize();
    }

    @Override
    public boolean isFinished(){
        return followCommand.isFinished();
    }
}
