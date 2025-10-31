package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;


import org.firstinspires.ftc.teamcode.commands.FacePose;
import org.firstinspires.ftc.teamcode.commands.MotifWriteCommand;
import org.firstinspires.ftc.teamcode.motif.MotifEnums;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsOldBot;
import org.firstinspires.ftc.teamcode.util.ShootSide;

@Config
@Configurable
@Autonomous(name = "Back Left")
public class ThreeBallBackLeftAuto extends BaseAuto {
    public static double motifDetectionTimeMs = 5000;
    int startPipeline = 1;

    public static Pose startPose = new Pose(56, 8, Math.toRadians(90));
//    public static Pose startToShootControlPose = new Pose(54, 19);
    public static Pose shootPose = new Pose(60, 17, Math.toRadians(112));
    public static Pose leavePose = new Pose(58, 38, Math.toRadians(112));
    PathChain toShootingPath;
    PathChain leaveBasePath;
    MotifEnums.Motif motifPattern;
    MotifWriteCommand motifCommand;

    ShootSide shootSide = ShootSide.LEFT;
    Pose currentPose;

    double speed;
    double acc;

    public static long waitTime = 5000;
    public static double pathDistThresholdMin = 3;
    public static double headingErrorMax = 5;
    @Override
    protected Pose getStartPose(){
        return startPose;
    }

    @Override
    protected void setupVision(){
        limelight.pipelineSwitch(startPipeline);
        limelight.start();
    }

    @Override
    protected void buildPaths(){
        toShootingPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .setHeadingConstraint(headingErrorMax)
                .setTimeoutConstraint(3000)
                .setTranslationalConstraint(pathDistThresholdMin)
                .build();
        leaveBasePath = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, leavePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), leavePose.getHeading())
                .setHeadingConstraint(headingErrorMax)
                .setTimeoutConstraint(3000)
                .setTranslationalConstraint(pathDistThresholdMin)
                .build();
    }


    @Override
    protected boolean isVisionComplete(){
        motifPattern = motifCommand.getDetected();
        if(motifPattern != MotifEnums.Motif.NONE){
            ConstantsOldBot.motifIsBusy = false;
            return true;
        }
        ConstantsOldBot.motifIsBusy = true;
        return false;
    }

    @Override
    protected Command preMotifSequence(){
        motifCommand = new MotifWriteCommand(limelight, motifDetectionTimeMs);
        return new SequentialCommandGroup(
                motifCommand
        );

    }
    @Override
    protected Command postMotifSequence(){
        return new SequentialCommandGroup(
                new FollowPathCommand(follower, toShootingPath).setGlobalMaxPower(0.6),
                new WaitCommand( waitTime),
                new FacePose(follower, leftTargetPose),
//                new ShootSequence(spindexer, shooter, ramp, motifPattern, CRServoEx.RunMode.OptimizedPositionalControl, startPose, shootSide),
                new WaitCommand(waitTime),
                new FollowPathCommand(follower, leaveBasePath, false)
        );

    }

    protected void updateTelemetry(){
        follower.update();
        currentPose = follower.getPose();
        timer.getTime();
        addStringToTelem("Motif Pattern", String.valueOf(motifPattern));
        addToTelemGraph("Update Rate", 1/timer.getDeltaTime());
        addToTelemGraph("Pose(X)", currentPose.getX());
        addToTelemGraph("Pose(Y)", currentPose.getY());
        addToTelemGraph("Pose(Heading)", currentPose.getHeading());
        addToTelemGraph("Speed(in/s)", (speed != 0 ? speed : 0));
        addToTelemGraph("Acc(in/s^2)", (acc != 0 ? acc : 0));
        telemetry.update();
        graphManager.update();;
        telemetryManager.update();;
    }



    public void addStringToTelem(String s, String o){
        telemetry.addLine(s + o);
    }
    public void addToTelemGraph(String s, Number o){
        telemetry.addData(s, o);
        telemetryManager.addData(s, o);
        graphManager.addData(s, o);
    }



}

