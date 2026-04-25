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

public class AutoDriveToBalls extends CommandBase {
    Pose robotPose;
    Follower follower;
    BallPather ballPather;
    LimelightDetector limelightDetector;
    WheelControl2 drive;
    double drivePower;

    TelemetryManager telemetryM;

    public AutoDriveToBalls(
            Follower follower,
            BallPather ballPather,
            LimelightDetector limelightDetector,
            WheelControl2 drive,
            double drivePower
    ) {
        this.follower = follower;
        this.ballPather = ballPather;
        this.limelightDetector = limelightDetector;
        this.drive = drive;
        this.drivePower = drivePower;
        this.telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        addRequirements(limelightDetector);
    }

    @Override
    public void initialize() {
        limelightDetector.start();
    }

    @Override
    public void execute() {
        follower.update();
        robotPose = follower.getPose();

        Vector2d[] ballPixels = limelightDetector.getBallPixels();
        Pose[] ballPoses = limelightDetector.getBallPoses(robotPose, ballPixels);
        Pose[] nextPoses = ballPather.findPath(robotPose, ballPoses, 3);
        if (nextPoses.length > 0) {
            Pose nextPose = nextPoses[0];
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
