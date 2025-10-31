package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.colors.Threshold;

import java.util.Map;

public abstract class ColorDetector<T> {
    protected Map<T, Threshold[]> thresholds;
    protected T defaultValue;

    public ColorDetector(Map<T, Threshold[]> thresholds, T defaultColor) {
        this.thresholds = thresholds;
        this.defaultValue = defaultColor;
    }

    public abstract float[] readRawColor();

    public T readColor() {
        float[] color = readRawColor();

        for (Map.Entry<T, Threshold[]> entry : thresholds.entrySet()) {
            boolean inThreshold = true;
            for (int i = 0; i < color.length; i++) {
                if (!entry.getValue()[i].contains(color[i])) {
                    inThreshold = false;
                    break;
                }
            }
            if (inThreshold) return entry.getKey();
        }
        return defaultValue;
    }
}
