package org.firstinspires.ftc.teamcode.templates;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.PanelsDrawing;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.concurrent.TimeUnit;

@Configurable
@Autonomous
public class PedroPanelsTemplate extends OpMode {
    TelemetryManager telemetryM;
    GraphManager graphM;
    Follower follower;
    Timer timer;
    PathChain path;

    public static Pose startPose = new Pose(0, 72, Math.toRadians(0));
    public static Pose endPose = new Pose(72, 72, Math.toRadians(0));
    Pose currentPose;
    double speed;
    double acceleration;


    @Override
    public void init() {
        PanelsDrawing.init();
        timer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        graphM = PanelsGraph.INSTANCE.getManager();

        follower = Constants.createPinpointFollower(hardwareMap);
        follower.setStartingPose(startPose);
        path = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
    }

    @Override
    public void init_loop() {
        updateData();
        updateTelemetry();
    }

    @Override
    public void start() {
        timer.restartTimer();
    }

    @Override
    public void loop() {
        updateData();
        updateTelemetry();
        follower.followPath(path);
    }

    public void updateData() {
        follower.update();
        currentPose = follower.getPose();
        speed = follower.getVelocity().getMagnitude();
        acceleration = follower.getAcceleration().getMagnitude();
    }

    public void addDataTelemetryGraph(String key, Number value) {
        telemetryM.addData(key, value);
        graphM.addData(key, value);
    }

    public void updateTelemetry() {
        // Field
        PanelsDrawing.drawRobot(currentPose);
        PanelsDrawing.drawPoseHistory(follower.getPoseHistory());
        PanelsDrawing.sendPacket();

        // Telemetry
        addDataTelemetryGraph("Loop time (ms)", timer.getDeltaTime(TimeUnit.MILLISECONDS));
        telemetryM.addData("Pose X (in)", currentPose.getX());
        telemetryM.addData("Pose Y (in)", currentPose.getY());
        telemetryM.addData("Pose Heading (rad)", currentPose.getHeading());
        addDataTelemetryGraph("Speed (in/s)", speed);
        addDataTelemetryGraph("Acceleration (in/s^2)", acceleration);

        // Updates
        telemetryM.update(telemetry);
        graphM.update();
    }
}