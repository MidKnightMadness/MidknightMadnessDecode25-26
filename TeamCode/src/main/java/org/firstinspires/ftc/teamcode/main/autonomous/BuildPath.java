package org.firstinspires.ftc.teamcode.main.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.CommandOpMode;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.tests.camera.CamCommand;
import org.firstinspires.ftc.teamcode.util.Timer;

public class BuildPath extends SequentialCommandGroup {
    Follower follower;
    CamCommand camCommand;

    double targetX;
    PathChain pathChain;
    public boolean pathCreated;
    Pose failsafePose;
    public BuildPath(Follower follower, CamCommand camCommand, double targetX, Pose targetFailsafePose){
        this.follower = follower;
        this.camCommand = camCommand;
        this.targetX = targetX;
        this.failsafePose = targetFailsafePose;
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
