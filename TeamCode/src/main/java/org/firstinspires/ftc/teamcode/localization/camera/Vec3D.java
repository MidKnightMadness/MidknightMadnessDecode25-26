package org.firstinspires.ftc.teamcode.localization.camera;

import com.pedropathing.math.Matrix;

public class Vec3D {
    public double x, y, z;

    public Vec3D(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Matrix toMatrix() {
        return new Matrix(new double[][] {new double[] {x, y, z}});
    }

    public static Vec3D fromMatrix(Matrix mat) {
        assert (mat.getRows() == 1 && mat.getColumns() == 3) ||
                (mat.getRows() == 3 && mat.getColumns() == 1): "Matrix must be 1x3 or 3x1";

        double[] row = mat.getRows() == 1 ? mat.getRow(0) : mat.getCol(0);
        return new Vec3D(row[0], row[1], row[2]);
    }

    public double dot(Vec3D other) {
        return this.x * other.x + this.y * other.y + this.z * other.z;
    }

    public Vec3D plus(Vec3D other) {
        return new Vec3D(this.x + other.x, this.y + other.y, this.z + other.z);
    }

    public Vec3D times(double c) {
        return new Vec3D(c * x, c * y, c * z);
    }
}
