package org.firstinspires.ftc.teamcode.autonomous;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.Style;
import com.bylazar.graph.GraphManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.commands.MotifReadCommand;
import org.firstinspires.ftc.teamcode.commands.ShootSequence;
import org.firstinspires.ftc.teamcode.motif.MotifEnums;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.util.DashboardDrawing;
import org.firstinspires.ftc.teamcode.util.PanelsDrawing;
import org.firstinspires.ftc.teamcode.util.ShootSide;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

import kotlinx.coroutines.selects.SelectUnbiasedKt;

@Config
@Configurable
@Autonomous(name = "Back Left")
public class ThreeBallBackLeftAuto extends BaseAuto {
    public static double motifDetectionTimeMs = 5000;
    int startPipeline = 1;

    public static Pose startPose = new Pose(56, 8, Math.toRadians(90));
    public static Pose startToShootControlPose = new Pose(62, 15);
    public static Pose shootPose = new Pose(67, 15, Math.toRadians(112));
    public static Pose leavePose = new Pose(58, 38, Math.toRadians(112));
    Path toShootingPath;
    Path leaveBasePath;
    MotifEnums.Motif motifPattern;
    MotifReadCommand motifCommand;

    ShootSide shootSide = ShootSide.LEFT;
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
        toShootingPath = new Path(new BezierCurve(startPose, startToShootControlPose, shootPose));
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
                motifCommand,
                new FollowPathCommand(follower, toShootingPath).setGlobalMaxPower(0.5),
                new WaitCommand( 2000)
        );

    }
    @Override
    protected Command postMotifSequence(){
        return new SequentialCommandGroup(
//                new ShootSequence(spindexer, shooter, ramp, motifPattern, CRServoEx.RunMode.OptimizedPositionalControl, startPose, shootSide),
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

