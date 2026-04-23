package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.localization.camera.Camera;
import org.firstinspires.ftc.teamcode.localization.camera.Pather;
import org.firstinspires.ftc.teamcode.localization.camera.Plane;

import java.util.List;

public class LimelightDetector extends SubsystemBase {
    public static double fovX = 54.5, fovY = 42;
    public static int resX = 2592, resY = 1944;
    public static double pitch = 0, roll = 0;
    public static double z = 11.4;
    public static double drivePower = 0.5;

    Limelight3A limelight;
    Pather pather;
    boolean isOn = false;
    Plane floor = new Plane(0, 0, 1, 2.5);

    public LimelightDetector(Limelight3A limelight) {
        this.limelight = limelight;
        this.pather = new Pather(fovX, fovY, resX, resY, pitch, roll, z);
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    @Override
    public void periodic() {
    }

    public void start() {
        limelight.setPollRateHz(200);
        limelight.start();
        limelight.pipelineSwitch(3);
    }

    public void close() {
        limelight.close();
    }

    public Vector2d[] getBallPixels() {
        LLResult result = limelight.getLatestResult();
        if (result == null) {
            return new Vector2d[] {};
        }
        List<LLResultTypes.DetectorResult> rawDetections = result.getDetectorResults();
        if (rawDetections == null || rawDetections.isEmpty()) {
            return new Vector2d[] {};
        }

        return rawDetections
                .stream()
                .map(detection -> new Vector2d(
                        detection.getTargetXPixels(),
                        detection.getTargetYPixels()
                ))
                .toArray(Vector2d[]::new);
    }

    public Pose[] getBallPoses(Pose robotPose, Vector2d[] ballPixels) {
        return pather.ballPoses(robotPose, ballPixels);
    }
}
