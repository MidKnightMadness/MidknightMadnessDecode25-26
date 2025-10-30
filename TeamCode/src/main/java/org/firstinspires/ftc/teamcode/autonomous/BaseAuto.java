package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.motif.MotifEnums;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.util.ShootSide;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.io.File;
import java.io.FileWriter;
import java.util.Map;


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

    @Override
    public void initialize() {
        super.reset();
        timer = new Timer();
        timer.restart();

        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);
        spindexer = new Spindexer(hardwareMap);
        ramp = new Ramp(hardwareMap);
        shooter = new TwoWheelShooter(hardwareMap, TwoWheelShooter.RunMode.VelocityControl);

        startPose = getStartPose();
        follower = Constants.createKalmanPinpointAprilFollower(hardwareMap, startPose, telemetry);
        follower.setStartingPose(startPose);

        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        graphManager = PanelsGraph.INSTANCE.getManager();
        buildPaths();
        setupVision();
        schedule(preMotifSequence());
    }


    @Override
    public void run(){
        super.run();
        if(isVisionComplete() && !postMotifFinished){
            schedule(postMotifSequence());
            postMotifFinished = true;
        }

        updateTelemetry();
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
