package org.firstinspires.ftc.teamcode.colors;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

public class ColorClassifier<T> {
    protected final Map<T, Threshold[]> thresholds;
    protected T defaultValue;

    public ColorClassifier(Map<T, Threshold[]> thresholds, T defaultValue) {
        this.thresholds = thresholds;
        this.defaultValue = defaultValue;
    }

    public T classify(double... color) {
        for (Map.Entry<T, Threshold[]> entry : thresholds.entrySet()) {
            if (Threshold.isWithin(entry.getValue(), color)) return entry.getKey();
        }
        return defaultValue;
    }

    public Set<T> getLabels() {
        return thresholds.keySet();
    }

    public void addThreshold(T value, Threshold... threshold) {
        thresholds.put(value, threshold);
    }

    public void removeThreshold(T value) {
        thresholds.remove(value);
    }

    public Builder<T> toBuilder() {
        Builder<T> builder = new Builder<>();
        builder.thresholds.putAll(this.thresholds);
        builder.defaultValue = this.defaultValue;
        return builder;
    }

    public static class Builder<T> {
        private final Map<T, Threshold[]> thresholds = new HashMap<>();
        private T defaultValue;

        public Builder<T> add(T label, Threshold... threshold) {
            thresholds.put(label, threshold);
            return this;
        }

        public Builder<T> setDefault(T defaultValue) {
            this.defaultValue = defaultValue;
            return this;
        }

        public ColorClassifier<T> build() {
            if (defaultValue == null) {
                throw new IllegalStateException("Default value must be set");
            }
            return new ColorClassifier<>(thresholds, defaultValue);
        }
    }
}
