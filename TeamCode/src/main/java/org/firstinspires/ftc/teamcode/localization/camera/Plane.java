package org.firstinspires.ftc.teamcode.localization.camera;

public class Plane {
    public double a;
    public double b;
    public double c;
    public double d;

    public Plane(double a, double b, double c, double d) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
    }

    public static Plane fromNormal(Vec3D pos, Vec3D normal) {
        double a = normal.x, b = normal.y, c = normal.z;
        double d = -(a * pos.x + b * pos.y + c * pos.z);
        return new Plane(a, b, c, d);
    }

    public Vec3D intersect(Ray ray) {
        Vec3D n = new Vec3D(a, b, c);
        Vec3D l0 = ray.endpoint;
        Vec3D v = ray.direction;
        double dotVN = v.dot(n);

        if (Math.abs(dotVN) < 1e-6) {
            return null; // Parallel
        }

        double t = -(n.dot(l0) + d) / dotVN;
        if (t < 0) {
            return null; // Behind the ray
        }

        return l0.plus(v.times(t));
    }
}
