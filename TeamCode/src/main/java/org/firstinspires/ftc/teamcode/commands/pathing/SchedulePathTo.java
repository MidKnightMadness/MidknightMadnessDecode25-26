package org.firstinspires.ftc.teamcode.commands.pathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
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
    double vel;
    double maxPower = 0;
    public SchedulePathTo(Follower follower, Pose startPose, Pose targetPose){
        this.follower = follower;
        this.startPose = startPose;
        this.targetPose = targetPose;
    }
    public SchedulePathTo(Follower follower, Pose startPose, Pose endPose, double headingConstraint){
        this.follower = follower;
        this.startPose = startPose;
        this.targetPose = endPose;
        this.headingConstraint = headingConstraint;
    }
    public SchedulePathTo(Follower follower, Pose startPose, Pose targetPose, double headingConstraint, double timeOutConstraint, double translationalConstraint){
        this.follower = follower;
        this.startPose = startPose;
        this.targetPose = targetPose;
        this.headingConstraint = headingConstraint;
        this.timeOutConstraint = timeOutConstraint;
        this.translationalConstraint = translationalConstraint;
    }
    public SchedulePathTo(Follower follower, Pose startPose, Pose targetPose, double headingConstraint, double timeOutConstraint, double translationalConstraint, double velConstraint){
        this.follower = follower;
        this.startPose = startPose;
        this.targetPose = targetPose;
        this.headingConstraint = headingConstraint;
        this.timeOutConstraint = timeOutConstraint;
        this.translationalConstraint = translationalConstraint;
        this.vel = velConstraint;
    }

    public SchedulePathTo(Follower follower, Pose targetPose, double headingConstraint, double timeOutConstraint, double translationalConstraint){
        this.follower = follower;
        this.targetPose = targetPose;
        this.headingConstraint = headingConstraint;
        this.timeOutConstraint = timeOutConstraint;
        this.translationalConstraint = translationalConstraint;
    }
    public SchedulePathTo(Follower follower, Pose targetPose){
        this.follower = follower;
        this.targetPose = targetPose;
        this.startPose = follower.getPose();
    }

    FollowPathCommand followCommand;

    public SchedulePathTo setMaxPower(double maxPower){
        this.maxPower = maxPower;
        maxPowerUse = true;
        return this;
    }

    public SchedulePathTo setHeadingConstraint(double headingConstraint){
        this.headingConstraint = headingConstraint;
        return this;
    }
    public SchedulePathTo setTimeoutConstraint(double timeOut){
        this.timeOutConstraint = timeOut;
        return this;
    }
    public SchedulePathTo setTranslationalError(double translationalError){
        this.translationalConstraint = translationalError;
        return this;
    }
    public SchedulePathTo setVel(double velConstraint){
        this.vel = velConstraint;
        return this;
    }

    @Override
    public void initialize(){
        //follower.update();
        Pose currentPose = startPose == null ? follower.getPose() : startPose;

        PathChain pathChain;
        if(headingConstraint != 0 && timeOutConstraint != 0 && translationalConstraint != 0 && vel != 0){
            pathChain = follower.pathBuilder()
                    .addPath(new BezierLine(currentPose, targetPose))
                    .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
                    .setHeadingConstraint(headingConstraint)
                    .setTimeoutConstraint(timeOutConstraint)
                    .setTranslationalConstraint(translationalConstraint)
                    .setVelocityConstraint(vel)
                    .build();
        }
        else if(headingConstraint != 0 && timeOutConstraint != 0 && translationalConstraint != 0 ){
            pathChain = follower.pathBuilder()
                    .addPath(new BezierLine(currentPose, targetPose))
                    .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
                    .setHeadingConstraint(headingConstraint)
                    .setTimeoutConstraint(timeOutConstraint)
                    .setTranslationalConstraint(translationalConstraint)
                    .build();
        } else if(headingConstraint != 0 && timeOutConstraint != 0){
            pathChain = follower.pathBuilder()
                    .addPath(new BezierLine(currentPose, targetPose))
                    .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
                    .setHeadingConstraint(headingConstraint)
                    .setTimeoutConstraint(timeOutConstraint)
                    .build();
        } else if(headingConstraint != 0){
                pathChain = follower.pathBuilder()
                        .addPath(new BezierLine(currentPose, targetPose))
                        .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
                        .setHeadingConstraint(headingConstraint)
                        .build();
        } else{
            pathChain = follower.pathBuilder()
                    .addPath(new BezierLine(currentPose, targetPose))
                    .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
                    .build();
        }

        if(maxPowerUse) {
            followCommand = new FollowPathCommand(follower, pathChain, true).setGlobalMaxPower(maxPower);
        }
        else{
            followCommand = new FollowPathCommand(follower, pathChain, true);
        }
        addCommands(followCommand);
        super.initialize();
    }

    @Override
    public boolean isFinished(){
        return followCommand.isFinished();
    }
}
