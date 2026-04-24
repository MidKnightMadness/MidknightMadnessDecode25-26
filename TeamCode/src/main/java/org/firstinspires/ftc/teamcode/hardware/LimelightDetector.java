package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.localization.camera.BallLocator;
import org.firstinspires.ftc.teamcode.localization.camera.Plane;

import java.util.List;
import java.util.function.Supplier;

public class LimelightDetector extends SubsystemBase {
    public static double fovX = Math.toRadians(54.5), fovY = Math.toRadians(42);
    public static int resX = 2592, resY = 1944;
    public static double pitch = 0, roll = 0;
    public static double z = 11.4;
    public static Plane floor = new Plane(0, 0, 1, -2.5);

    Limelight3A limelight;
    BallLocator locator;
    LLResult prevResult;
    Supplier<Pose> poseSupplier;
    Vector2d[] cachedBallPixels;
    Pose[] cachedBallPoses;

    public LimelightDetector(Limelight3A limelight, Supplier<Pose> poseSupplier) {
        this.limelight = limelight;
        this.poseSupplier = poseSupplier;
        this.locator = new BallLocator(fovX, fovY, resX, resY, pitch, roll, z, floor);
    }

    @Override
    public void periodic() {
        LLResult result = limelight.getLatestResult();

        if (result == null) {
            cachedBallPixels = new Vector2d[]{};
            cachedBallPoses = new Pose[]{};
            prevResult = null;
            return;
        }

//        if (result == prevResult) {
//            return;
//        }

        prevResult = result;

        List<LLResultTypes.DetectorResult> rawDetections = result.getDetectorResults();

        if (rawDetections == null || rawDetections.isEmpty()) {
            cachedBallPixels = new Vector2d[]{};
            cachedBallPoses = new Pose[]{};
            return;
        }

        cachedBallPixels = rawDetections.stream()
                .map(d -> new Vector2d(
                        d.getTargetXPixels(),
                        d.getTargetYPixels()
                ))
                .toArray(Vector2d[]::new);

        cachedBallPoses = locator.ballPoses(poseSupplier.get(), cachedBallPixels);
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    public void start() {
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(3);
    }

    public void close() {
        limelight.close();
    }

    public Vector2d[] getBallPixels() {
        return cachedBallPixels;
    }

    public Pose[] getBallPoses() {
        return cachedBallPoses;
    }
}
