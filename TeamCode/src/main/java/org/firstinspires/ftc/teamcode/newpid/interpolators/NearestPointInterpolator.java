package org.firstinspires.ftc.teamcode.newpid.interpolators;

import org.json.JSONException;

import java.io.IOException;

public class NearestPointInterpolator implements Interpolator2D {
    @Override
    public Interpolator2D fromFile(String fileName) throws IOException, JSONException {
        return null;
    }

    @Override
    public void toFile(String fileName) throws IOException {

    }

    @Override
    public double getZ(double x, double y) {
        return 0;
    }
}
