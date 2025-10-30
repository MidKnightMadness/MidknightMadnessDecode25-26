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
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.ShootSequence;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.motif.MotifEnums;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.commands.MotifReadCommand;
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

@Config
@Configurable
@Autonomous
public class ThreeBallCloseLeftAuto extends CommandOpMode {
    Limelight3A limelight;
    public static double motifDetectionTimeMs = 5000;
    TelemetryManager telemetryManager;
    GraphManager graphManager;
    Follower follower;
    Timer timer;
    int startPipeline = 1;

    public static Pose startPose = new Pose(88, 135, Math.toRadians(90));
    public static Pose motifDetectionPose = new Pose(88, 112, Math.toRadians(100));
    public static Pose startToShootControlPose = new Pose(107, 107);
    public static Pose shootPose = new Pose(86, 86, Math.toRadians(50));
    public static Pose leavePose = new Pose(86, 66, Math.toRadians(90));
    Path toMotifDetectionPath;
    Path toShootingPath;
    Path leaveBasePath;
    Spindexer spindexer;
    Ramp ramp;
    TwoWheelShooter shooter;
    MotifEnums.Motif motifPattern = MotifEnums.Motif.NONE;
    ShootSide shootSide = ShootSide.LEFT;
    boolean postMotifCommandsInitalized = false;
    MotifReadCommand motifCommand;
    @Override
    public void initialize() {
        super.reset();
        timer = new Timer();
        timer.restart();
        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);
        follower = Constants.createKalmanPinpointAprilFollower(hardwareMap, startPose, telemetry);
        follower.setStartingPose(startPose);

        buildPaths();

        limelight.start();
        limelight.pipelineSwitch(startPipeline);

        spindexer = new Spindexer(hardwareMap);
        ramp = new Ramp(hardwareMap);
        shooter = new TwoWheelShooter(hardwareMap, TwoWheelShooter.RunMode.VelocityControl);

        SequentialCommandGroup commands = new SequentialCommandGroup(
                new FollowPathCommand(follower, toMotifDetectionPath).setGlobalMaxPower(0.5),
                motifCommand = new MotifReadCommand(limelight, motifDetectionTimeMs),
                new FollowPathCommand(follower, toShootingPath).setGlobalMaxPower(0.5),
                new WaitCommand( 2000)
        );
        schedule(commands);
    }



    private void buildPaths() {
        toMotifDetectionPath = new Path(new BezierLine(startPose, motifDetectionPose));
        toShootingPath = new Path(new BezierCurve(motifDetectionPose, startToShootControlPose, shootPose));
        leaveBasePath = new Path(new BezierLine(shootPose, leavePose));
    }



    Pose currentPose;
    double speed;
    double acc;

    @Override
    public void run(){
        super.run();

        motifPattern = motifCommand.getDetected();
        if(motifPattern != MotifEnums.Motif.NONE && !postMotifCommandsInitalized){//have detected motif pattern
            schedule(createSequencePostMotif());//add the rest of the commands
            postMotifCommandsInitalized = true;
        }


        currentPose = follower.getPose();
        speed = follower.getVelocity().getMagnitude();
        acc = follower.getAcceleration().getMagnitude();

        PanelsDrawing.drawRobot(follower.getPose());

        addStringToTelem("Motif Pattern", motifPattern.toString());
        addToTelemGraph("Pose(X)", currentPose.getX());
        addToTelemGraph("Pose(Y)", currentPose.getY());
        addToTelemGraph("Pose(Heading)", currentPose.getHeading());
        addToTelemGraph("Speed(in/s)", speed);
        addToTelemGraph("Acc(in/s^2)", acc);
    }

    public SequentialCommandGroup createSequencePostMotif(){
        return new SequentialCommandGroup(
                new ShootSequence(spindexer, shooter, ramp, motifPattern, CRServoEx2.RunMode.OptimizedPositionalControl, startPose, shootSide, 0),
                new WaitCommand(1000),
                new FollowPathCommand(follower, leaveBasePath, false)
        );
    }
    public void addToTelemGraph(String s, Number o){
        telemetry.addData(s, o);
        telemetryManager.addData(s, o);
        graphManager.addData(s, o);
    }

    public void addStringToTelem(String s, String o){
        telemetry.addData(s, o);
        telemetryManager.addData(s, o);
    }


}
