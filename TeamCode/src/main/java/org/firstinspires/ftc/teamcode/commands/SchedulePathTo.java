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
    Pose startPose;
    double headingConstraint;
    double timeOutConstraint;
    double translationalConstraint;
    boolean maxPowerUse = false;
    double maxPower = 0;
    public SchedulePathTo(Follower follower, Pose startPose, Pose targetPose, double headingConstraint, double timeOutConstraint, double translationalConstraint){
        this.follower = follower;
        this.startPose = startPose;
        this.targetPose = targetPose;
        this.headingConstraint = headingConstraint;
        this.timeOutConstraint = timeOutConstraint;
        this.translationalConstraint = translationalConstraint;
    }

    public SchedulePathTo(Follower follower, Pose targetPose, double headingConstraint, double timeOutConstraint, double translationalConstraint){
        this.follower = follower;
        this.targetPose = targetPose;
        this.headingConstraint = headingConstraint;
        this.timeOutConstraint = timeOutConstraint;
        this.translationalConstraint = translationalConstraint;
        this.startPose = null;
    }

    FollowPathCommand followCommand;

    public SchedulePathTo setMaxPower(double maxPower){
        this.maxPower = maxPower;
        maxPowerUse = true;
        return this;
    }
    @Override
    public void initialize(){
        follower.update();
        Pose currentPose = startPose == null ? follower.getPose() : startPose;

        PathChain pathChain = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, targetPose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
                .setHeadingConstraint(headingConstraint)
                .setTimeoutConstraint(timeOutConstraint)
                .setTranslationalConstraint(translationalConstraint)
                .build();

        if(maxPowerUse) {
            followCommand = new FollowPathCommand(follower, pathChain, false ).setGlobalMaxPower(maxPower);
        }
        else{
            followCommand = new FollowPathCommand(follower, pathChain, false);
        }
        addCommands(followCommand);
        super.initialize();
    }

    @Override
    public boolean isFinished(){
        return followCommand.isFinished();
    }
}
