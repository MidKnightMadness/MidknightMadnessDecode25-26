package org.firstinspires.ftc.teamcode.localization.camera;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;

class BallLocatorTest {
    public static double fovX = Math.toRadians(54.5), fovY = Math.toRadians(42);
    public static int resX = 2592, resY = 1944;
    public static double pitch = 0, roll = 0;
    public static double z = 11.4;
    public static Plane floor = new Plane(0, 0, 1, -2.5);

    @Test
    void generatePermutations() {
        int n = 5, k = 3;
        ArrayList<int[]> permutations = BallLocator.generatePermutations(n, k);
        for (int[] arr : permutations) {
            System.out.println(Arrays.toString(arr));
        }
        System.out.println(permutations.size());
    }

    @Test
    void pather() {
        BallLocator locator = new BallLocator(fovX, fovY, resX, resY, pitch, roll, z, floor);
        Pose robotPose = new Pose(0, 0, 0);

        Vector2d[] ballPixels = new Vector2d[] {
                new Vector2d(1259.5, 1697),
        };

        Pose[] ballPoses = locator.ballPoses(robotPose, ballPixels);
        for (int i = 0; i < ballPoses.length; i++) {
            System.out.println(ballPoses[i]);
        }

        Pose nextPose = BallLocator.patherGreedy(robotPose, ballPoses);
        System.out.println("-------");
        System.out.println(nextPose);

        Pose[] optimal = BallLocator.patherOptimal(robotPose, ballPoses, 3);
        System.out.println("-------");
        System.out.println(optimal[0]);
        System.out.println(optimal[1]);
        System.out.println(optimal[2]);
    }

}