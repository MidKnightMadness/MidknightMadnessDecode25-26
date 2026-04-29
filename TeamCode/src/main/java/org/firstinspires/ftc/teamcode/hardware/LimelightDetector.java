package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.localization.camera.BallLocator;
import org.firstinspires.ftc.teamcode.localization.camera.Plane;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.ArrayList;
import java.util.List;

public class LimelightDetector extends SubsystemBase {
    public static double fovX = Math.toRadians(54.5), fovY = Math.toRadians(42);
    public static int resX = 2592, resY = 1944;
    public static double pitch = 0, roll = 0;
    public static double z = 11.4;
    public static Plane floor = new Plane(0, 0, 1, -2.5);

    long lastUpdateMillis;
    Limelight3A limelight;
    BallLocator locator;
    LLResult prevResult;
    Timer timer;
    Vector2d[] cachedBallPixels;
    PoseHistory poseHistory;
    Pose[] cachedPoses;

    public static class PoseHistory {
        private static final int MAX_SIZE = 300;

        ArrayList<Pose> poses = new ArrayList<>();
        ArrayList<Long> times = new ArrayList<>();

        public void addPose(Pose pose) {
            poses.add(pose);
            times.add(System.currentTimeMillis());

            if (poses.size() > MAX_SIZE) {
                poses.remove(0);
                times.remove(0);
            }
        }

        public Pose poseAtTime(long t) {
            int n = times.size();
            if (n == 0) return null;

            // Clamp edge cases
            if (t <= times.get(0)) return poses.get(0);
            if (t >= times.get(n - 1)) return poses.get(n - 1);

            // Binary search for first index i such that times[i] >= t
            int lo = 0, hi = n - 1;
            while (lo < hi) {
                int mid = (lo + hi) >>> 1;
                if (times.get(mid) < t) {
                    lo = mid + 1;
                } else {
                    hi = mid;
                }
            }

            int i = lo;

            // Now interpolate between i-1 and i
            long t0 = times.get(i - 1);
            long t1 = times.get(i);

            Pose p0 = poses.get(i - 1);
            Pose p1 = poses.get(i);

            if (t1 == t0) return p0;
            double alpha = (double) (t - t0) / (double) (t1 - t0);
            return lerpPose(p0, p1, alpha);
        }

        private Pose lerpPose(Pose a, Pose b, double t) {
            // Assuming Pose has x, y, heading
            double x = a.getX() + (b.getX() - a.getX()) * t;
            double y = a.getY() + (b.getY() - a.getY()) * t;

            // IMPORTANT: handle angle wraparound properly
            double dTheta = b.getHeading() - a.getHeading();
            double heading = normalizeAngle(a.getHeading() + dTheta * t);

            return new Pose(x, y, heading);
        }

        private double normalizeAngle(double angle) {
            while (angle > Math.PI) angle -= 2 * Math.PI;
            while (angle < -Math.PI) angle += 2 * Math.PI;
            return angle;
        }
    }

    public LimelightDetector(Limelight3A limelight) {
        this.limelight = limelight;
        this.locator = new BallLocator(fovX, fovY, resX, resY, pitch, roll, z, floor);
        this.poseHistory = new PoseHistory();
        this.timer = new Timer();
        timer.restart();
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
        LLResult result = limelight.getLatestResult();

        if (result == null) {
            return new Vector2d[] {};
        }
//        if (prevResult != null && result.getCaptureLatency() == prevResult.getCaptureLatency()) {
//            return cachedBallPixels;
//        }
//        lastUpdateMillis = System.currentTimeMillis();
//
//        lastUpdateMillis = result.getControlHubTimeStamp();
//                - (long)(result.getCaptureLatency() + result.getTargetingLatency());
//        prevResult = result;

        List<LLResultTypes.DetectorResult> rawDetections = result.getDetectorResults();
        if (rawDetections == null || rawDetections.isEmpty()) {
            cachedBallPixels = new Vector2d[] {};
            return new Vector2d[] {};
        }

        cachedBallPixels = rawDetections
                .stream()
                .map(detection -> new Vector2d(
                        detection.getTargetXPixels(),
                        detection.getTargetYPixels()
                ))
                .toArray(Vector2d[]::new);

        return cachedBallPixels;
    }

    public Pose[] getBallPoses(Pose robotPose, Vector2d[] ballPixels) {
//        poseHistory.addPose(robotPose);
//        return locator.ballPoses(poseHistory.poseAtTime(lastUpdateMillis), ballPixels);
//        if (cachedPoses == null || timer.getTime() > 500) {
//            cachedPoses = locator.ballPoses(robotPose, ballPixels);
//        }
        return locator.ballPoses(robotPose, ballPixels);
    }
}
