
package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.MotifReadCommand;
import org.firstinspires.ftc.teamcode.commands.ShootSequence;
import org.firstinspires.ftc.teamcode.motif.MotifEnums;
import org.firstinspires.ftc.teamcode.util.ShootSide;

@Config
@Configurable
@Autonomous(name = "Close Left")
public class ThreeBallCloseLeftAuto extends BaseAuto {
    public static double motifDetectionTimeMs = 5000;
    int startPipeline = 1;
    public static Pose startPose = new Pose(56, 135, Math.toRadians(90));
    public static Pose motifDetectionPose = new Pose(56, 112, Math.toRadians(80));
    public static Pose startToShootControlPose = new Pose(37, 107);
    public static Pose shootPose = new Pose(58, 58, Math.toRadians(130));
    public static Pose leavePose = new Pose(58, 66, Math.toRadians(90));
    Path toMotifDetectionPath;
    Path toShootingPath;
    Path leaveBasePath;
    MotifEnums.Motif motifPattern;
    MotifReadCommand motifCommand;

    ShootSide shootSide = ShootSide.RIGHT;
    Pose currentPose;
    double speed;
    double acc;

    @Override
    protected Pose getStartPose(){
        return startPose;
    }

    @Override
    protected void setupVision(){
        limelight.start();
        limelight.pipelineSwitch(startPipeline);
    }

    @Override
    protected void buildPaths(){
        toMotifDetectionPath = new Path(new BezierLine(startPose, motifDetectionPose));
        toShootingPath = new Path(new BezierCurve(motifDetectionPose, startToShootControlPose, shootPose));
        leaveBasePath = new Path(new BezierLine(shootPose, leavePose));
    }


    @Override
    protected boolean isVisionComplete(){
        motifPattern = motifCommand.getDetected();
        if(motifPattern != MotifEnums.Motif.NONE){
            return true;
        }
        return false;
    }
    @Override
    protected Command preMotifSequence(){
        motifCommand = new MotifReadCommand(limelight, motifDetectionTimeMs);
        return new SequentialCommandGroup(
                new FollowPathCommand(follower, toMotifDetectionPath).setGlobalMaxPower(0.5),
                motifCommand,
                new FollowPathCommand(follower, toShootingPath).setGlobalMaxPower(0.5),
                new WaitCommand( 2000)
        );

    }
    @Override
    protected Command postMotifSequence(){
        return new SequentialCommandGroup(
                new ShootSequence(spindexer, shooter, ramp, motifPattern, CRServoEx.RunMode.OptimizedPositionalControl, startPose, shootSide),
                new WaitCommand(1000),
                new FollowPathCommand(follower, leaveBasePath, false)
        );

    }

    protected void updateTelemetry(){
        addStringToTelem("Motif Pattern", String.valueOf(motifPattern));
        addToTelemGraph("Pose(X)", currentPose.getX());
        addToTelemGraph("Pose(Y)", currentPose.getY());
        addToTelemGraph("Pose(Heading)", currentPose.getHeading());
        addToTelemGraph("Speed(in/s)", (speed != 0 ? speed : 0));
        addToTelemGraph("Acc(in/s^2)", (acc != 0 ? acc : 0));
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

