package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.hardware.LimelightDetector;
import org.firstinspires.ftc.teamcode.localization.camera.BallPather;
import org.firstinspires.ftc.teamcode.pedroPathing.motorTesting.WheelControl2;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerNonCR;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.Arrays;
import java.util.function.Function;
import java.util.function.Predicate;

public class AutoDriveToBallsSpindexer extends CommandBase {
    Pose robotPose;
    Follower follower;
    BallPather ballPather;
    LimelightDetector limelightDetector;
    SpindexerNonCR spindexer;
    WheelControl2 drive;
    double drivePower;

    TelemetryManager telemetryM;
    Predicate<Pose> constraints;

    Pose[] currentPath;
    int numBalls = -1;
    double maxGapMillis;
    Timer timer;


    public AutoDriveToBallsSpindexer(
            Follower follower,
            BallPather ballPather,
            LimelightDetector limelightDetector,
            WheelControl2 drive,
            SpindexerNonCR spindexer,
            double maxGapMillis,
            double drivePower
    ) {
        this.follower = follower;
        this.ballPather = ballPather;
        this.limelightDetector = limelightDetector;
        this.drive = drive;
        this.drivePower = drivePower;
        this.spindexer = spindexer;
        this.telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        this.maxGapMillis = maxGapMillis;
        this.timer = new Timer();

        addRequirements(limelightDetector);
    }

    public AutoDriveToBallsSpindexer withConstraints(Predicate<Pose> constraints) {
        this.constraints = constraints;
        return this;
    }

    @Override
    public void initialize() {
        limelightDetector.start();
    }

    @Override
    public void execute() {
        follower.update();
        robotPose = follower.getPose();

        if (numBalls == -1 || spindexer.getBallCount() != numBalls || timer.getTime() > maxGapMillis) {
            timer.restart();
            Vector2d[] ballPixels = limelightDetector.getBallPixels();
            Pose[] ballPoses = limelightDetector.getBallPoses(robotPose, ballPixels);
            currentPath = ballPather.findPath(robotPose, ballPoses, 3);
        }

        numBalls = spindexer.getBallCount();
//        if (constraints == null) {
//            ballPoses = limelightDetector.getBallPoses(robotPose, ballPixels);
//        } else {
//            ballPoses = Arrays.stream(limelightDetector.getBallPoses(robotPose, ballPixels)
//                    .filter(pose -> constraints.apply(pose)));
//        }
        if (currentPath != null && currentPath.length > 0) {
            Pose nextPose = currentPath[0];
            double targetHeading = nextPose.minus(robotPose).getAsVector().getTheta();
            Pose targetPose = nextPose.withHeading(targetHeading);
//            follower.followPath(new Path(new BezierLine(robotPose, targetPose)));
            drive.pid(robotPose, targetPose, drivePower);

            telemetryM.addData("Target heading", targetHeading);
            telemetryM.addData("Target pose", targetPose);
        } else {
            drive.stop();
        }

        telemetryM.addData("Robot pose", robotPose);
    }

    @Override
    public void end(boolean interrupted) {
        limelightDetector.close();
    }
}
