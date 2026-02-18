package org.firstinspires.ftc.teamcode.newpid.interpolators;

import org.json.JSONException;

import java.io.IOException;

public interface Interpolator2D {
    Interpolator2D fromFile(String fileName) throws IOException, JSONException;

    void toFile(String fileName) throws IOException;

    double getZ(double x, double y);
}
