package org.firstinspires.ftc.teamcode.commands.pathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.CamCommand;

public class BuildPath extends SequentialCommandGroup {
    Follower follower;
    CamCommand camCommand;

    double targetX;
    PathChain pathChain;
    PathChain pathChain2;
    public boolean pathCreated;
    Pose failsafePose;
    double distSecond = 0;
    public BuildPath(Follower follower, CamCommand camCommand, double targetX, Pose targetFailsafePose){
        this.follower = follower;
        this.camCommand = camCommand;
        this.targetX = targetX;
        this.failsafePose = targetFailsafePose;
    }
    public BuildPath(Follower follower, CamCommand camCommand, double targetX, double distSecond, Pose targetFailsafePose){
        this.follower = follower;
        this.camCommand = camCommand;
        this.targetX = targetX;
        this.failsafePose = targetFailsafePose;
        this.distSecond = distSecond;
    }
    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        follower.update();
        PathChain possiblePathChain = camCommand.getList(follower, targetX);
//        if(possiblePathChain!= null && possiblePathChain.getPath(0)!= null) {
        pathChain = possiblePathChain;
        pathCreated = true;
        if(distSecond!= 0){
            pathChain2 = follower.pathBuilder()
                    .addPath(new BezierLine(possiblePathChain.endPose(), new Pose(distSecond, possiblePathChain.endPose().getY(), possiblePathChain.endPose().getHeading())))
                    .setLinearHeadingInterpolation(possiblePathChain.endPose().getHeading(), possiblePathChain.endPose().getHeading())
                    .build();
        }

//        }
//        else{
//            Pose currPose = follower.getPose();
//            pathChain = follower.pathBuilder()
//                    .addPath(new BezierLine(currPose, new Pose(targetX, currPose.getY(), currPose.getHeading())))
//                   .setLinearHeadingInterpolation(currPose.getHeading(), currPose.getHeading())
//                   .build();
//            pathCreated = true;
//        }
    }

    public PathChain getPathChain(){
        return pathChain;
    }
    public PathChain getPathChain2(){
        return pathChain2;
    }

    @Override
    public boolean isFinished(){
        return pathCreated;
    }
    @Override
    public void end(boolean interrupted) {
//        if(!pathCreated){
//            pathChain = follower.pathBuilder()
//                    .addPath(new BezierLine(follower.getPose(), failsafePose))
//                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), failsafePose.getHeading())
//                    .build();
//        }
    }
}
