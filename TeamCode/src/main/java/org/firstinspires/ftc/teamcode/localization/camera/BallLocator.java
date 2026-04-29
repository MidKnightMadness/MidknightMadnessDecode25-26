package org.firstinspires.ftc.teamcode.localization.camera;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.Stream;

public class BallLocator {
    public Pose[] balls;
    public Camera camera;
    public double pitch, roll, z;
    public Plane floor;

    private static class Result {
        ArrayList<Integer> perm;
        double dist;

        Result(ArrayList<Integer> perm, double dist) {
            this.perm = perm;
            this.dist = dist;
        }
    }

    public BallLocator(
            double fovX, double fovY, int resX, int resY,
            double pitch, double roll, double z, Plane floor
    ) {
        this.camera = new Camera(
                fovX, fovY,
                resX, resY,
                pitch, 0, roll,
                new Vec3D(0, 0, z)
        );
        this.pitch = pitch;
        this.roll = roll;
        this.z = z;
        this.floor = floor;
    };

    public Pose[] ballPoses(Pose robotPose, Vector2d[] ballPixels) {
        double x = robotPose.getX();
        double y = robotPose.getY();
        double heading = robotPose.getHeading();

        camera.setPos(new Vec3D(x, y, z));
        camera.setRot(pitch, heading, roll);

        return Arrays.stream(ballPixels)
                .flatMap(pixel -> {
                    Vec3D point = camera.project(pixel.getX(), pixel.getY(), floor);
                    if (point == null) {
                        return Stream.empty();
                    }
                    return Stream.of(new Pose(point.x, point.y));
                })
                .toArray(Pose[]::new);
    }

    private static int closestBallIdx(Pose robotPose, Pose[] ballPoses) {
        int bestIdx = 0;
        double minDist = Double.MAX_VALUE;

        for (int i = 0; i < ballPoses.length; i++) {
            double dist = robotPose.distanceFrom(ballPoses[i]);
            if (dist < minDist) {
                minDist = dist;
                bestIdx = i;
            }
        }

        return bestIdx;
    }

    public static ArrayList<int[]> generatePermutations(int n, int k) {
        return generatePermutationsImpl(n, k, new ArrayList<>(), new boolean[n]);
    }

    private static ArrayList<int[]> generatePermutationsImpl(int n, int k, ArrayList<Integer> current, boolean[] used) {
        if (current.size() == k) {
            ArrayList<int[]> base = new ArrayList<>();
            base.add(current.stream().mapToInt(i -> i).toArray());
            return base;
        }

        ArrayList<int[]> result = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            if (used[i]) continue;

            used[i] = true;
            current.add(i);
            result.addAll(generatePermutationsImpl(n, k, current, used));
            current.remove(current.size() - 1);
            used[i] = false;
        }

        return result;
    }

    public static Pose patherGreedy(Pose robotPose, Pose[] ballPoses) {
        if (ballPoses.length == 0) {
            return robotPose;
        }
        return ballPoses[closestBallIdx(robotPose, ballPoses)];
    }

    public static Pose[] patherOptimal(Pose robotPose, Pose[] ballPoses, int nBalls) {
        return patherOptimal(robotPose, ballPoses, nBalls, ballPoses.length);
    }

    public static Pose[] patherOptimal(
            Pose robotPose,
            Pose[] ballPoses,
            int nBalls,
            int topK
    ) {
        if (ballPoses.length == 0 || nBalls == 0) {
            return new Pose[] { robotPose };
        }

        int n = ballPoses.length;

        double[][] dist = new double[n][n];
        double[] robotDist = new double[n];

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                dist[i][j] = ballPoses[i].distanceFrom(ballPoses[j]);
            }
            robotDist[i] = robotPose.distanceFrom(ballPoses[i]);
        }

        boolean[] used = new boolean[n];
        int[] path = new int[nBalls];
        int[] bestPath = new int[nBalls];

        double[] bestDist = { Double.MAX_VALUE };

        dfs(
                0,
                path,
                used,
                dist,
                robotDist,
                topK,
                0.0,
                bestDist,
                bestPath
        );

        return Arrays.stream(bestPath)
                .mapToObj(i -> ballPoses[i])
                .toArray(Pose[]::new);
    }

    private static void dfs(
            int depth,
            int[] path,
            boolean[] used,
            double[][] dist,
            double[] robotDist,
            int topK,
            double currentDist,
            double[] bestDist,
            int[] bestPath
    ) {
        if (depth == path.length) {
            if (currentDist < bestDist[0]) {
                bestDist[0] = currentDist;
                System.arraycopy(path, 0, bestPath, 0, path.length);
            }
            return;
        }

        int n = used.length;

        int[] candidates = new int[n];
        int cSize = 0;

        for (int i = 0; i < n; i++) {
            if (!used[i]) {
                candidates[cSize++] = i;
            }
        }

        int last = (depth == 0) ? -1 : path[depth - 1];

        int limit = Math.min(topK, cSize);

        // top K selection
        for (int i = 0; i < limit; i++) {
            int best = i;

            for (int j = i + 1; j < cSize; j++) {
                if (score(candidates[j], last, robotDist, dist)
                        < score(candidates[best], last, robotDist, dist)) {
                    best = j;
                }
            }

            int tmp = candidates[i];
            candidates[i] = candidates[best];
            candidates[best] = tmp;
        }

        // dfs
        for (int idx = 0; idx < limit; idx++) {
            int i = candidates[idx];

            double added = (depth == 0)
                    ? robotDist[i]
                    : dist[last][i];

            double newDist = currentDist + added;

            if (newDist >= bestDist[0]) continue;

            used[i] = true;
            path[depth] = i;

            dfs(
                    depth + 1,
                    path,
                    used,
                    dist,
                    robotDist,
                    topK,
                    newDist,
                    bestDist,
                    bestPath
            );

            used[i] = false;
        }
    }

    private static double score(
            int i,
            int last,
            double[] robotDist,
            double[][] dist
    ) {
        if (last == -1) {
            return robotDist[i];
        }
        return dist[last][i];
    }
}
