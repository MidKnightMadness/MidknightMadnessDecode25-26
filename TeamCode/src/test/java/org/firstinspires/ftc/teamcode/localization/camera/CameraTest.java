package org.firstinspires.ftc.teamcode.localization.camera;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class CameraTest {

    @Test
    void project() {
        Camera cam = new Camera(
                Math.toRadians(54.5),
                Math.toRadians(42),
                1920,
                1080,
                Math.toRadians(0),
                Math.toRadians(0),
                Math.toRadians(0),
                new Vec3D(0, 0, 1)
        );
        Plane floor = new Plane(0, 0, 1, 0);
        Vec3D projected = cam.project(960, 1000, floor);
        Vec3D expected = new Vec3D(3.058148, 0, 0);
        assertEquals(expected.x, projected.x, 0.01);
        assertEquals(expected.y, projected.y, 0.01);
        assertEquals(expected.z, projected.z, 0.01);
    }
}