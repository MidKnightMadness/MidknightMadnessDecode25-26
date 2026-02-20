package org.firstinspires.ftc.teamcode.hardware.color;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

/**
 * A color classifier that applies {@link Threshold} to sort into labels.
 *
 * @param <T> The classified type (e.g. BallColor enum)
 * @author Daniel - FTC 7854
 */
public class ColorClassifier<T> {
    protected final Map<T, Threshold[]> thresholdMap;
    protected T defaultLabel;

    /**
     * Constructor for ColorClassifier.
     * You may want to use {@link ColorClassifier.Builder} for easier construction
     *
     * @param thresholds Map between the labels and the thresholds
     * @param defaultLabel The default label to return
     */
    public ColorClassifier(Map<T, Threshold[]> thresholds, T defaultLabel) {
        this.thresholdMap = thresholds;
        this.defaultLabel = defaultLabel;
    }

    /**
     * Classify a color into the classified label
     *
     * @param color Color tuple
     * @return The label of the color
     */
    public T classify(double... color) {
        for (Map.Entry<T, Threshold[]> entry : thresholdMap.entrySet()) {
            if (Threshold.colorWithin(entry.getValue(), color)) return entry.getKey();
        }
        return defaultLabel;
    }

    /**
     * Gets all the possible labels known to the classifier.
     *
     * @return The set of possible labels
     */
    public Set<T> getLabels() {
        return thresholdMap.keySet();
    }

    /**
     * Add a label to the classifier
     *
     * @param label The label to add
     * @param thresholds The thresholds for the label
     */
    public void addLabel(T label, Threshold... thresholds) {
        thresholdMap.put(label, thresholds);
    }

    /**
     * Remove a label from being clasisied
     *
     * @param label The label to remove
     */
    public void removeLabel(T label) {
        thresholdMap.remove(label);
    }

    /**
     * Set the default value for the classifier.
     *
     * @param defaultLabel The label to set as default
     */
    public void setDefault(T defaultLabel) {
        this.defaultLabel = defaultLabel;
    }

    /**
     * A convenience builder for {@link ColorClassifier}
     *
     * @param <T> The classified type (e.g. BallColor enum)
     */
    public static class Builder<T> {
        private final Map<T, Threshold[]> thresholdMap = new HashMap<>();
        private T defaultLabel;

        /**
         * Add a label to the classifier.
         *
         * @param label The label to add
         * @param thresholds The thresholds for the label
         */
        public Builder<T> add(T label, Threshold... thresholds) {
            thresholdMap.put(label, thresholds);
            return this;
        }

        /**
         * Set the default value for the classifier.
         *
         * @param defaultLabel The label to set as default
         */
        public Builder<T> setDefault(T defaultLabel) {
            this.defaultLabel = defaultLabel;
            return this;
        }

        /**
         * Build the color classifier
         *
         * @return The color classifier
         */
        public ColorClassifier<T> build() {
            if (defaultLabel == null) {
                throw new IllegalStateException("Default value must be set");
            }
            return new ColorClassifier<>(thresholdMap, defaultLabel);
        }
    }
}
