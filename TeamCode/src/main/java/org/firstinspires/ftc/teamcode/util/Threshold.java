package org.firstinspires.ftc.teamcode.util;

public class Threshold {
    public final float min;
    public final float max;

    public Threshold(float min, float max) {
        this.min = min;
        this.max = max;
    }

    public boolean contains(float value) {
        return value >= min && value <= max;
    }
}