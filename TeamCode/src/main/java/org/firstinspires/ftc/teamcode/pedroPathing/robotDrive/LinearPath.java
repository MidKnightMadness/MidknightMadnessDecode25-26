package org.firstinspires.ftc.teamcode.pedroPathing.robotDrive;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsOldBot;
import org.firstinspires.ftc.teamcode.util.PanelsDrawing;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.concurrent.TimeUnit;

@Configurable
@Autonomous(name = "LinearPath", group = "Tuning")
public class LinearPath extends CommandOpMode {
    TelemetryManager telemetryM;
    GraphManager graphM;
    Follower follower;
    Timer timer;
    Path path1;
    Path path2;
  //  public static Pose startPose = new Pose(88, 8, Math.toRadians(90));
 // public static Pose startPose = new Pose(88, 8, Math.toRadians(90));
  //  public static Pose motifDetectionPose = new Pose(87, 94, Math.toRadians(100));
   // public static Pose shootPose = new Pose(84, 17, Math.toRadians(243));

    public static Pose startPose = new Pose(84, 8, Math.toRadians(90));
    public static Pose motifDetectionPose = new Pose(88, 12, Math.toRadians(90));
    public static Pose shootPose = new Pose(84, 17, Math.toRadians(243));
    Pose currentPose;
    double speed;
    double acceleration;
    boolean startedPath = false;


    public static double timeOutConstraint = 1000;
    public static double headingConstraint = 0.03;
    public static double pathDistThresholdMin = 1;

    @Override
    public void initialize() {
        super.reset();
        PanelsDrawing.init();
        timer = new Timer();
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//        graphM = PanelsGraph.INSTANCE.getManager();

        follower = ConstantsBot.createPinpointFollower(hardwareMap);
        follower.setPose(startPose);
        path1 = new Path(new BezierLine(startPose, motifDetectionPose));
        path1.setLinearHeadingInterpolation(startPose.getHeading(), motifDetectionPose.getHeading());
        path2 = new Path(new BezierLine(motifDetectionPose, shootPose));
        path2.setLinearHeadingInterpolation(motifDetectionPose.getHeading(), shootPose.getHeading());

    }

    boolean firstPathDone = false;
    boolean secondPathDone = false;
    double time = 0;


    @Override
    public void run() {
        super.run();
        updateData();
        updateTelemetry();
        if (!follower.isBusy() && !startedPath && !firstPathDone) {
            startedPath = true;
            schedule(new FollowPathCommand(follower, path1, true));
            firstPathDone = true;
        }

        if(!follower.isBusy() && startedPath && firstPathDone && !secondPathDone){
            schedule(new FollowPathCommand(follower, path2, true));
            secondPathDone = true;
        }

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
//        PanelsDrawing.drawRobot(currentPose);
//        PanelsDrawing.drawPoseHistory(follower.getPoseHistory());
//        PanelsDrawing.sendPacket();

        // Telemetry
//        addDataTelemetryGraph("Loop time (ms)", timer.getDeltaTime(TimeUnit.MILLISECONDS));
        telemetry.addData("Pose X (in)", currentPose.getX());
        telemetry.addData("Pose Y (in)", currentPose.getY());
        telemetry.addData("Pose Heading (rad)", currentPose.getHeading());
//        telemetry.addData("Device Status", ConstantsBot.deviceStatus)
//        addDataTelemetryGraph("Speed (in/s)", speed);
//        addDataTelemetryGraph("Acceleration (in/s^2)", acceleration);

        // Updates
//        telemetryM.update(telemetry);
//        graphM.update();
    }

}