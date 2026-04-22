package org.firstinspires.ftc.teamcode.localization.camera;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.SortedSet;

public class Pather {
    public Pose[] balls;
    public Camera camera;
    public static Plane floor = new Plane(0, 0, 1, -2.5); // Plane 2.5 inches above ground
    public double pitch, roll, z;

    public Pather(
            double fov, int resX, int resY,
            double pitch, double roll, double z
    ) {
        this.camera = new Camera(
                fov, resX, resY,
                pitch, 0, roll,
                new Vec3D(0, 0, z)
        );
        this.pitch = pitch;
        this.roll = roll;
        this.z = z;
    };

    public Pose[] ballPoses(Pose robotPose, Vector2d[] ballPixels) {
        double x = robotPose.getX();
        double y = robotPose.getY();
        double heading = robotPose.getHeading();

        camera.setPos(new Vec3D(x, y, z));
        camera.setRot(pitch, heading, roll);

        return Arrays.stream(ballPixels)
                .map(pixel -> {
                    Vec3D point = camera.project(
                            pixel.getX(),
                            pixel.getY(),
                            floor
                    );
                    return new Pose(x + point.x, y + point.y);
                })
                .toArray(Pose[]::new);
    }

    private int closestBallIdx(Pose robotPose, Pose[] ballPoses) {
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

    public Pose patherGreedy(Pose robotPose, Pose[] ballPoses) {
        return ballPoses[closestBallIdx(robotPose, ballPoses)];
    }

    public Pose[] patherOptimal(Pose robotPose, Pose[] ballPoses, int nBalls) {
        double[][] dist = new double[nBalls][nBalls];
        ArrayList<int[]> permutations = generatePermutations(ballPoses.length, nBalls);

        for (int i = 0; i < nBalls; i++) {
            for (int j = 0; j < i; j++) {
                dist[i][j] = ballPoses[i].distanceFrom(ballPoses[j]);
            }
        }

        double minDist = Double.MAX_VALUE;
        int[] bestPerm = new int[nBalls];
        for (int[] perm : permutations) {
            double currentDist = 0;
            for (int i = 0; i < nBalls - 1; i++) {
                currentDist += dist[i+1][i];
            }

//            if (dist < minDist) {
//                minDist = dist;
//                bestPerm = perm;
//            }
        }

        return (Pose[]) Arrays.stream(bestPerm).mapToObj(i -> ballPoses[i]).toArray();
    }
}
