package org.firstinspires.ftc.teamcode.main.autonomous.cameraTest;

import com.pedropathing.geometry.Pose;
import java.util.ArrayList;

public class CamRegionTest {

    public static int NUM_REGIONS = 5;

    //5 x based regions (forward distance from robot)
    public static double regionStartX = 80;   // closest region start
    public static double regionWidth = 10;    // size of each region

    private ArrayList<Pose> detections;

    public CamRegionTest(ArrayList<Pose> detections) {
        this.detections = detections;
    }

    // count balls per region
    public int[] getRegionCounts() {
        int[] counts = new int[NUM_REGIONS];

        if (detections == null) return counts;

        for (Pose pose : detections) {
            double x = pose.getX();

            int index = (int) ((x - regionStartX) / regionWidth); //

            if (index >= 0 && index < NUM_REGIONS) {
                counts[index]++;
            }
        }

        return counts;
    }

    // pick best region (most balls)
    public int getBestRegion() {
        int[] counts = getRegionCounts();

        int bestIndex = 0;
        int max = -1;

        for (int i = 0; i < counts.length; i++) {
            if (counts[i] > max) {
                max = counts[i];
                bestIndex = i;
            }
        }

        return bestIndex;
    }

    // get center X of a region
    public double getRegionCenterX(int regionIndex) {
        return regionStartX + regionWidth * regionIndex + (regionWidth / 2.0);
    }

    // OPTIONAL: prioritize closer regions if tied
    public int getBestRegionWeighted() {
        int[] counts = getRegionCounts();

        int bestIndex = 0;
        double bestScore = -1;

        for (int i = 0; i < counts.length; i++) {
            double centerX = getRegionCenterX(i);

            // Closer regions = higher score
            double score = counts[i] / (1 + centerX);

            if (score > bestScore) {
                bestScore = score;
                bestIndex = i;
            }
        }

        return bestIndex;
    }
}