package org.firstinspires.ftc.teamcode.OutdatedPrograms;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.PanelsDrawing;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.concurrent.TimeUnit;

@Deprecated
@Configurable
@Autonomous
public class BackSixBallAutoFSM extends OpMode {
    public enum Motif {
        GPP,
        PGP,
        PPG
    }

    public enum PathState {
        shootLoaded,
        driveToPickup,
        waitForHumanLoad,
        driveToScore
    }

    TelemetryManager telemetryM;
    GraphManager graphM;
    Follower follower;
    Timer timer;

    Path driveToPickupPath;
    Path drivePickupToScorePath;

    public static Pose startPose = new Pose(84, 7, Math.toRadians(90));
    public static Pose pickupPose = new Pose(120, 7, Math.toRadians(90));
    public static Pose endPose = new Pose(120, 7, Math.toRadians(90));
    Pose currentPose;
    double speed;
    double acceleration;

    Motif motif;
    PathState pathState;
    int shootingState; // How many times you've shot from zone

    @Override
    public void init() {
        PanelsDrawing.init();
        timer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        graphM = PanelsGraph.INSTANCE.getManager();

        follower = Constants.createPinpointFollower(hardwareMap);
        follower.setStartingPose(startPose);
        pathState = PathState.shootLoaded;
        shootingState = 0;
    }

    @Override
    public void init_loop() {
        updateData();
        updateTelemetry();
    }

    @Override
    public void start() {
        timer.restart();
    }

    @Override
    public void loop() {
        updateData();
        updateTelemetry();
        pathUpdate();
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
        addDataTelemetryGraph("Loop time (ms)", timer.getDeltaTime());
        telemetryM.addData("Pose X (in)", currentPose.getX());
        telemetryM.addData("Pose Y (in)", currentPose.getY());
        telemetryM.addData("Pose Heading (rad)", currentPose.getHeading());
        addDataTelemetryGraph("Speed (in/s)", speed);
        addDataTelemetryGraph("Acceleration (in/s^2)", acceleration);

        // Updates
        telemetryM.update(telemetry);
        graphM.update();
    }

    public void setPathState(PathState newPathState) {
        pathState = newPathState;
        timer.restart();
    }

    // Will return true if currently shooting, false if done shooting
    public boolean shootMotif(Motif motif) {
        // Placeholder to simulate time
        return timer.getTime() < 2000;
    }

    public void buildPaths() {
        driveToPickupPath = new Path(new BezierLine(startPose, pickupPose));
        drivePickupToScorePath = new Path(new BezierLine(pickupPose, endPose));
    }

    public void pathUpdate() {
        switch (pathState) {
            case shootLoaded:
                shootingState++;
                // Only transition if you've shot the first time
                if (!shootMotif(motif) && shootingState < 2) {
                    follower.followPath(driveToPickupPath);
                    setPathState(PathState.driveToPickup);
                }
                break;

            case driveToPickup:
                if(!follower.isBusy()) {
                    setPathState(PathState.waitForHumanLoad);
                }
                break;

            case waitForHumanLoad:
                if (timer.getTime() > 5000) {
                    follower.followPath(drivePickupToScorePath);
                    setPathState(PathState.driveToScore);
                }
                break;

            case driveToScore:
                if(!follower.isBusy()) {
                    setPathState(PathState.shootLoaded);
                }
                break;
        }
    }
}