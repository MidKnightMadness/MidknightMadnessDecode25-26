package org.firstinspires.ftc.teamcode.localization.camera;

public class Ray {
    public Vec3D endpoint;
    public Vec3D direction;

    public Ray(Vec3D endpoint, Vec3D direction) {
        this.endpoint = endpoint;
        this.direction = direction;
    }
}
