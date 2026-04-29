package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.LimelightDetector.PoseHistory;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

import com.pedropathing.geometry.Pose;

class PoseHistoryTest {

    private static final double EPS = 1e-6;

    @Test
    void returnsNullWhenEmpty() {
        PoseHistory history = new PoseHistory();
        assertNull(history.poseAtTime(System.currentTimeMillis()));
    }

    @Test
    void returnsFirstPoseWhenTimeBeforeRange() throws InterruptedException {
        PoseHistory history = new PoseHistory();

        history.addPose(new com.pedropathing.geometry.Pose(0, 0, 0));
        Thread.sleep(2);
        history.addPose(new Pose(10, 10, 1));

        long before = System.currentTimeMillis() - 1000;
        Pose p = history.poseAtTime(before);

        assertEquals(0, p.getX(), EPS);
        assertEquals(0, p.getY(), EPS);
    }

    @Test
    void returnsLastPoseWhenTimeAfterRange() throws InterruptedException {
        PoseHistory history = new PoseHistory();

        history.addPose(new Pose(0, 0, 0));
        Thread.sleep(2);
        history.addPose(new Pose(10, 10, 1));

        long after = System.currentTimeMillis() + 1000;
        Pose p = history.poseAtTime(after);

        assertEquals(10, p.getX(), EPS);
        assertEquals(10, p.getY(), EPS);
    }

    @Test
    void interpolatesMidpointCorrectly() throws InterruptedException {
        PoseHistory history = new PoseHistory();

        history.addPose(new Pose(0, 0, 0));
        long t0 = System.currentTimeMillis();

        Thread.sleep(10);

        history.addPose(new Pose(10, 10, 0));
        long t1 = System.currentTimeMillis();

        long mid = (t0 + t1) / 2;
        Pose p = history.poseAtTime(mid);

        assertEquals(5, p.getX(), 1.0); // allow timing jitter
        assertEquals(5, p.getY(), 1.0);
    }

    @Test
    void interpolatesHeadingWithWraparound() throws InterruptedException {
        PoseHistory history = new PoseHistory();

        // From near +pi to -pi (should go the short way)
        history.addPose(new Pose(0, 0, Math.PI - 0.1));
        long t0 = System.currentTimeMillis();

        Thread.sleep(10);

        history.addPose(new Pose(0, 0, -Math.PI + 0.1));
        long t1 = System.currentTimeMillis();

        long mid = (t0 + t1) / 2;
        Pose p = history.poseAtTime(mid);

        // Should be near pi (not swing all the way around)
        assertTrue(Math.abs(Math.abs(p.getHeading()) - Math.PI) < 0.2);
    }

    @Test
    void exactTimestampReturnsExactPose() throws InterruptedException {
        PoseHistory history = new PoseHistory();

        history.addPose(new Pose(1, 2, 3));
        Thread.sleep(5);
        history.addPose(new Pose(4, 5, 6));

        long exact = history.times.get(1);
        Pose p = history.poseAtTime(exact);

        assertEquals(4, p.getX(), EPS);
        assertEquals(5, p.getY(), EPS);
        assertEquals(6, p.getHeading(), EPS);
    }

    @Test
    void respectsMaxSize() {
        PoseHistory history = new PoseHistory();

        for (int i = 0; i < 400; i++) {
            history.addPose(new Pose(i, i, i));
        }

        assertEquals(300, history.poses.size());
        assertEquals(300, history.times.size());

        // Oldest should be 100 now
        Pose first = history.poses.get(0);
        assertEquals(100, first.getX(), EPS);
    }
}