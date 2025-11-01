package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commands.PoseWriteCommand;
import org.firstinspires.ftc.teamcode.commands.SideWriteCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsOldBot;
import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.util.ShootSide;
import org.firstinspires.ftc.teamcode.util.Timer;

@Config
@Configurable
public class BaseAuto extends CommandOpMode {
    Follower follower;
    Timer timer;
    Pose startPose;
    Limelight3A limelight;
    Spindexer spindexer;
    Ramp ramp;
    TwoWheelShooter shooter;

    TelemetryManager telemetryManager;
    GraphManager graphManager;
    boolean postMotifFinished = false;
    public static Pose leftTargetPose = new Pose(0, 144, 0);
    public static Pose rightTargetPose = new Pose(144, 144, 0);

    public static double maxTimeMs = 29500;
    public static double maxWritePoseTimeMs = 200;
    public static double maxSideWriteTimeMs = 200;

    boolean stopEnd = false;
    ShootSide side;
    @Override
    public void initialize() {
        super.reset();
        timer = new Timer();
        timer.restart();

        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);
//        spindexer = new Spindexer(hardwareMap);
//        ramp = new Ramp(hardwareMap);
//        shooter = new TwoWheelShooter(hardwareMap, TwoWheelShooter.RunMode.VelocityControl);

        startPose = getStartPose();
        follower = ConstantsOldBot.createKalmanPinpointAprilFollower(hardwareMap, startPose, telemetry);

        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        graphManager = PanelsGraph.INSTANCE.getManager();
        buildPaths();
        setupVision();
        schedule(preMotifSequence());
    }

    protected ShootSide getSide(){
        return null;
    }

    @Override
    public void run(){
        super.run();
        if(!postMotifFinished && isVisionComplete()){
            schedule(postMotifSequence());
            postMotifFinished = true;
        }

        updateTelemetry();
        end();
    }

    public void end(){
        if(timer.getTime() >= maxTimeMs & !stopEnd){
            CommandScheduler.getInstance().cancelAll();
            schedule(new PoseWriteCommand(follower.getPose(), maxWritePoseTimeMs));
            schedule(new SideWriteCommand(getSide(), maxSideWriteTimeMs));
            stopEnd = true;
        }
    }
    protected Command postMotifSequence() {
        return null;
    }

    protected boolean isVisionComplete() {
        return false;
    }

    protected Command preMotifSequence() {
        return null;
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
