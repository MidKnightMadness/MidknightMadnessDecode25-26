package org.firstinspires.ftc.teamcode.colors;

import androidx.annotation.NonNull;

import org.opencv.core.Core;
import org.opencv.core.Mat;

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
        return new Color(target, space.to(target, values));
    }

    public Color toRgb() {
        return new Color(ColorSpace.RGB, space.toRgb(values));
    }

    public static Color from(ColorSpace source, ColorSpace target, double... color) {
        return new Color(target, target.from(source));
    }

    public static Color fromRgb(ColorSpace target, double... color) {
        return new Color(target, target.fromRgb(color));
    }

    public static boolean isWithin(Threshold[] thresholds, double[] color) {
        assert thresholds.length == color.length:
                "Thresholds length must be equal to color space length of " +
                        color.length + " got: " + thresholds.length;

        boolean isWithin = true;
        for (int i = 0; i < color.length; i++) {
            if (!thresholds[i].contains(color[i])) {
                isWithin = false;
                break;
            }
        }
        return isWithin;
    }

    public boolean isWithin(Threshold[] thresholds) {
        return Color.isWithin(thresholds, values);
    }

    @NonNull
    @Override
    public String toString() {
        return space + Arrays.toString(values);
    }
}
