package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import java.util.Arrays;

public class Color {
    private final ColorSpace space;
    private final double[] values;

    public Color(ColorSpace space, double... values) {
        this.space = space;
        this.values = values;
    }

    public ColorSpace getSpace() { return space; }
    public double[] getValues() { return values; }

    public Color to(ColorSpace target) {
        if (target == space) return this;
        return new Color(target, space.to(target, values));
    }

    public Color toRgb() {
        if (space == ColorSpace.RGB) return this;
        return new Color(ColorSpace.RGB, space.toRgb(values));
    }

    public Color fromRgb() {
        if (space == ColorSpace.RGB) return this;
        return new Color(space, ColorSpace.RGB.to(space, values));
    }

    @NonNull
    @Override
    public String toString() {
        return space + Arrays.toString(values);
    }
}
