package org.firstinspires.ftc.teamcode.main.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.PoseWriteCommand;
import org.firstinspires.ftc.teamcode.commands.SideWriteCommand;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.util.Timer;

@Config
@Configurable
public abstract class BaseAuto extends CommandOpMode {
    Follower follower;
    Timer gameTimer;
    Pose startPose;

    Limelight3A limelight;
    Spindexer spindexer;
    TwoWheelShooter shooter;
    Intake intake;
    TelemetryManager telemetryManager;
    GraphManager graphManager;
    boolean prevVisionComplete = false;
    public static Pose leftTargetPose = new Pose(12, 132, 0);
    public static Pose rightTargetPose = new Pose(132, 132, 0);
    FtcDashboard dashboard;
    TelemetryPacket dashboardPacket;

    public static double maxTimeMs = 29500;
    public static double maxWritePoseTimeMs = 200;
    public static double maxSideWriteTimeMs = 200;
    public static double[] pidBotGainsShooter = new double[]{0.0004, 0, 0.00001};
    public static double[] kBotGainsShooter = new double[]{0, 0.00005, 0};
    public static double[] pidTopGainsShooter = new double[]{0.0004, 0, 0.00001};
    public static double[] kTopGainsShooter = new double[]{0.02, 0.00005, 0};

    boolean stopEnd = false;
    ShootSide side;
    boolean postMotif = false;

    @Override
    public void initialize() {

//        CommandScheduler.getInstance().setBulkReading(
//                hardwareMap, LynxModule.BulkCachingMode.MANUAL // Scheduler will clean cache for you
//        );

        CommandScheduler.getInstance().cancelAll();
        super.reset();

        gameTimer = new Timer();

        initializeMechanisms();


        startPose = getStartPose();
        follower = ConstantsBot.createPinpointFollowerCustom(hardwareMap, new Pose(0, 0, 0));
        follower.setPose(startPose);

        dashboard = FtcDashboard.getInstance();
        dashboardPacket = new TelemetryPacket();

        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        graphManager = PanelsGraph.INSTANCE.getManager();
        buildPaths();
        setupVision();
        if(preMotifSequence() != null) {
            schedule(preMotifSequence());

        }

        gameTimer.restart();

    }

    protected void initializeMechanisms() {
//        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);

    }

//    protected BallColor[] getStartBallColors(){
//        return null;
//    }

    protected ShootSide getSide(){
        return ShootSide.LEFT;
    }

    @Override
    public void run(){
        super.run();
        update();
        if(!prevVisionComplete && isVisionComplete()){
            if(postMotifSequence() != null) {
                schedule(postMotifSequence());
            }
            prevVisionComplete = true;
        }

     //   if(postMotifSequence().isFinished()){
//            if(goToIntakeLine()!= null){
//                schedule(goToIntakeLine());
//            }
    //    }
//        if (timer.getTime() >= maxTimeMs) requestOpModeStop();
        updateTelemetry();
        writeValues();
    }

    public void update(){

    }
    public void writeValues() {
        if(gameTimer.getTime() >= maxTimeMs & !stopEnd) {
            CommandScheduler.getInstance().cancelAll();
            schedule(new ParallelCommandGroup(
                    new PoseWriteCommand(follower.getPose(), maxWritePoseTimeMs),
                    new SideWriteCommand(getSide(), maxSideWriteTimeMs)));
            stopEnd = true;
            requestOpModeStop();
        }
    }

//    public Command goToIntakeLine(){
//        return null;
//    }


    protected Command postMotifSequence() {
        return null;
    }

    protected Command preMotifSequence() {
        return null;
    }

    protected boolean isVisionComplete() {
        return true;
    }

    protected void setupVision() {
    }

    protected void buildPaths() {
    }

    protected void updateTelemetry(){
    }

    protected Pose getStartPose() {
        return null;
    }
}
