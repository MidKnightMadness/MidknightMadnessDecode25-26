package org.firstinspires.ftc.teamcode.newpid;

import org.json.JSONException;

import java.io.IOException;

public interface Interpolator2D<T extends Interpolator2D<T>> {
    T fromFile(String fileName) throws IOException, JSONException;

    void toFile(String fileName) throws IOException;
}
