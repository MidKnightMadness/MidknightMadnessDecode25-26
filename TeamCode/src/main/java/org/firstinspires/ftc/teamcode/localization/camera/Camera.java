package org.firstinspires.ftc.teamcode.localization.camera;

import com.pedropathing.math.Matrix;
import com.seattlesolvers.solverslib.geometry.Vector2d;

public class Camera {
    private double fovX, fovY;
    private int resX, resY;
    private double pitch, yaw, roll;
    private Vec3D pos;

    private double fPixX, fPixY;
    private double centerX, centerY;
    private Matrix rot;

    public Camera(
            double fovX, double fovY,
            int resX, int resY,
            double pitch, double yaw, double roll,
            Vec3D pos
    ) {
        this.fovX = fovX;
        this.fovY = fovY;
        this.resX = resX;
        this.resY = resY;
        this.pitch = pitch;
        this.yaw = yaw;
        this.roll = roll;
        this.pos = pos;

        this.fPixX = Math.abs(this.resX / (2 * Math.tan(this.fovX / 2)));
        this.fPixY = Math.abs(this.resY / (2 * Math.tan(this.fovY / 2)));
        this.centerX = this.resX / 2.0;
        this.centerY = this.resY / 2.0;
        this.rot = rotMatrix();
    }

    public void setFov(double fovX, double fovY) {
        this.fovX = fovX;
        this.fovY = fovY;
        this.fPixX = Math.abs(this.resX / (2 * Math.tan(this.fovX / 2)));
        this.fPixY = Math.abs(this.resY / (2 * Math.tan(this.fovY / 2)));
    }

    public void setRes(int resX, int resY) {
        this.resX = resX;
        this.resY = resY;
        this.fPixX = Math.abs(this.resX / (2 * Math.tan(this.fovX / 2)));
        this.fPixY = Math.abs(this.resY / (2 * Math.tan(this.fovY / 2)));
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

    private Matrix rotMatrix() {
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
    public Vec3D project(double pixX, double pixY, Plane plane) {
        Matrix directionCam = new Vec3D(1, (centerX - pixX) / fPixX, (centerY - pixY) / fPixY)
                .toMatrix()
                .transposed();

        System.out.println(fPixY);
        System.out.println((centerY - pixY) / fPixY);

        Matrix directionWorld = rot.multiply(directionCam);
        Ray ray = new Ray(pos, Vec3D.fromMatrix(directionWorld));
        return plane.intersect(ray);
    }
}
