package org.firstinspires.ftc.teamcode.localization.camera;

import com.pedropathing.geometry.Pose;

import java.util.Arrays;

public class NormalPather implements BallPather {
    int topK;

    public NormalPather() {
        this.topK = Integer.MAX_VALUE;
    }

    public NormalPather(int topK) {
        this.topK = topK;
    }

    @Override
    public Pose[] findPath(
            Pose robotPose,
            Pose[] ballPoses,
            int nBalls
    ) {
        if (ballPoses.length == 0 || nBalls == 0) {
            return new Pose[] {};
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
                0.0,
                bestDist,
                bestPath
        );

        return Arrays.stream(bestPath)
                .mapToObj(i -> ballPoses[i])
                .toArray(Pose[]::new);
    }

    private void dfs(
            int depth,
            int[] path,
            boolean[] used,
            double[][] dist,
            double[] robotDist,
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
