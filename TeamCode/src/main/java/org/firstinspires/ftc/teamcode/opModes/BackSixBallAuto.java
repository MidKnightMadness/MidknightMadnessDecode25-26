package org.firstinspires.ftc.teamcode.oldOpModes;

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
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;

import java.util.concurrent.TimeUnit;

@Configurable
@Autonomous
public class BackSixBallAuto extends CommandOpMode {
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

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new WaitCommand(2000),
                new FollowPathCommand(follower, driveToPickupPath, 1.0),
                new WaitCommand(2000),
                new FollowPathCommand(follower, drivePickupToScorePath, 1.0),
                new WaitCommand(2000)
        );

        schedule(autonomousSequence);
    }

    @Override
    public void run() {
        super.run();
        updateData();
        updateTelemetry();
    }

    public void updateData() {
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
        Drawing.drawRobot(currentPose);
        Drawing.drawPoseHistory(follower.getPoseHistory());
        Drawing.sendPacket();

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

    public void buildPaths() {
        driveToPickupPath = new Path(new BezierLine(startPose, pickupPose));
        drivePickupToScorePath = new Path(new BezierLine(pickupPose, endPose));
    }
}