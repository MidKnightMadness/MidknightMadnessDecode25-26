package org.firstinspires.ftc.teamcode.localization.camera;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class CameraTest {

    @Test
    void project() {
        Camera cam = new Camera(
                Math.toRadians(120),
                1920,
                1080,
                Math.toRadians(-10),
                Math.toRadians(90),
                Math.toRadians(0),
                new Vec3D(0, 0, 1)
        );
        Plane floor = new Plane(0, 0, 1, 0);
        Vec3D projected = cam.project(960, 1000, floor);
        Vec3D expected = new Vec3D(0, 1.754, 0);
        assertEquals(expected.x, projected.x, 0.01);
        assertEquals(expected.y, projected.y, 0.01);
        assertEquals(expected.z, projected.z, 0.01);
    }
}