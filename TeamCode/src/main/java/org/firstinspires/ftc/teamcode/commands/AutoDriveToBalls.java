package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.hardware.LimelightDetector;
import org.firstinspires.ftc.teamcode.localization.camera.Pather;
import org.firstinspires.ftc.teamcode.pedroPathing.motorTesting.WheelControl2;

import java.util.Arrays;
import java.util.List;

public class AutoDriveToBalls extends CommandBase {
    Pose robotPose;
    Follower follower;
    Pather pather;
    LimelightDetector limelightDetector;
    WheelControl2 drive;
    double drivePower;

    TelemetryManager telemetryM;

    public AutoDriveToBalls(
            Follower follower,
            Pather pather,
            LimelightDetector limelightDetector,
            WheelControl2 drive,
            double drivePower
    ) {
        this.follower = follower;
        this.pather = pather;
        this.limelightDetector = limelightDetector;
        this.drive = drive;
        this.drivePower = drivePower;
        this.telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        addRequirements();
    }

    @Override
    public void initialize() {
        limelightDetector.start();
    }

    @Override
    public void execute() {
        robotPose = follower.getPose();

        Vector2d[] ballPixels = limelightDetector.getBallPixels();
        Pose[] ballPoses = limelightDetector.getBallPoses(robotPose, ballPixels);
        Pose nextPose = Pather.patherGreedy(robotPose, ballPoses);
        double targetHeading = nextPose.minus(robotPose).getAsVector().getTheta();
        drive.pid(robotPose, nextPose.withHeading(targetHeading), drivePower);

        telemetryM.addData("Robot pose", robotPose);
        telemetryM.addData("Target heading", targetHeading);
        telemetryM.addData("Next pose", nextPose);
    }

    @Override
    public void end(boolean interrupted) {
        limelightDetector.close();
    }
}
