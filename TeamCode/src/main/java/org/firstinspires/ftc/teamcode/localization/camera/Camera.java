package org.firstinspires.ftc.teamcode.localization.camera;

import com.pedropathing.math.Matrix;

public class Camera {
    private double fov;
    private int resX, resY;
    private double pitch, yaw, roll;
    private Vec3D pos;

    private double fPix;
    private double centerX, centerY;
    private Matrix rot;

    public Camera(
            double fov, int resX, int resY,
            double pitch, double yaw, double roll,
            Vec3D pos
    ) {
        this.fov = fov;
        this.resX = resX;
        this.resY = resY;
        this.pitch = pitch;
        this.yaw = yaw;
        this.roll = roll;
        this.pos = pos;

        this.fPix = this.resX / (2 * Math.tan(this.fov / 2));
        this.centerX = this.resX / 2.0;
        this.centerY = this.resY / 2.0;
        this.rot = rotMatrix();
    }

    public void setFov(double fov) {
        this.fov = fov;
        this.fPix = this.resX / (2 * Math.tan(this.fov / 2));
    }

    public void setRes(int resX, int resY) {
        this.resX = resX;
        this.resY = resY;
        this.fPix = this.resX / (2 * Math.tan(this.fov / 2));
        this.centerX = this.resX / 2.0;
        this.centerY = this.resY / 2.0;
    }

    public void setRot(double pitch, double yaw, double roll) {
        this.pitch = pitch;
        this.yaw = yaw;
        this.roll = roll;
        this.rot = rotMatrix();
    }

    public void setPos(Vec3D pos) {
        this.pos = pos;
    }

    public Matrix rotMatrix() {
        Matrix rRoll = new Matrix(new double[][] {
                new double[] {1, 0, 0},
                new double[] {0, Math.cos(roll), -Math.sin(roll)},
                new double[] {0, Math.sin(roll), Math.cos(roll)}
        });
        Matrix rPitch = new Matrix(new double[][] {
                new double[] {Math.cos(pitch), 0, Math.sin(pitch)},
                new double[] {0, 1, 0},
                new double[] {-Math.sin(pitch), 0, Math.cos(pitch)}
        });
        Matrix rYaw = new Matrix(new double[][]{
                new double[] {Math.cos(yaw), -Math.sin(yaw), 0},
                new double[] {Math.sin(yaw), Math.cos(yaw), 0},
                new double[] {0, 0, 1}
        });

        return rYaw.multiply(rPitch).multiply(rRoll);
    }

    // Project a pixel position to the plane
    // (0, 0) is top left corner
    public Vec3D project(double u, double v, Plane plane) {
        Matrix directionCam = new Vec3D(fPix, centerX - u, centerY - v)
                .toMatrix()
                .transposed();

        Matrix directionWorld = rot.multiply(directionCam);
        Ray ray = new Ray(pos, Vec3D.fromMatrix(directionWorld));
        return plane.intersect(ray);
    }
}
