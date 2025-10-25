package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.util.Threshold;

import java.util.Map;

public abstract class ColorDetector<T extends Enum<T>> {

    protected Map<T, Threshold[]> thresholds; // 3 thresholds per color
    protected T defaultColor;

    public ColorDetector(Map<T, Threshold[]> thresholds, T defaultColor) {
        this.thresholds = thresholds;
        this.defaultColor = defaultColor;
    }

    public abstract float[] readRawColor(); // implementer provides sensor reading

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
        return defaultColor;
    }
}
