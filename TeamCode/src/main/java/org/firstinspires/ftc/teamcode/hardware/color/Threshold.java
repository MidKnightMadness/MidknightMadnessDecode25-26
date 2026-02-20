package org.firstinspires.ftc.teamcode.hardware.color;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

/**
 * Thresholding made simple.
 *
 * @author Daniel - 7854
 */
public class Threshold {
    public static final int GLOBAL_MIN = 0;
    public static final int GLOBAL_MAX = 255;

    public final boolean inverted;
    public final double low;
    public final double high;

    /**
     * Construct a threshold
     *
     * @param low The lower bound
     * @param high The upper bound
     * @param invert Whether to invert it (e.g. from 30-100 to 0-30 + 100-255)
     */
    public Threshold(double low, double high, boolean invert) {
        assert low >= GLOBAL_MIN && low <= GLOBAL_MAX:
                "Min threshold must be between " + GLOBAL_MIN + " and " + GLOBAL_MAX;
        assert high >= GLOBAL_MIN && high <= GLOBAL_MAX:
                "Max threshold must be between " + GLOBAL_MIN + " and " + GLOBAL_MAX;
        assert low <= high:
                "Min threshold must be less than or equal to high threshold";

        this.low = low;
        this.high = high;
        this.inverted = invert;
    }

    /**
     * Construct a threshold
     *
     * @param low The lower bound
     * @param high The upper bound
     */
    public Threshold(double low, double high) {
        this(low, high, false);
    }

    /**
     * Construct a threshold from {@link Threshold#GLOBAL_MIN} to {@link Threshold#GLOBAL_MAX}
     *
     * @return The desired threshold
     */
    public static Threshold any() {
        return new Threshold(GLOBAL_MIN, GLOBAL_MAX);
    }

    /**
     * Threshold from ratios of the total range
     *
     * @param lowRatio The lower bound ratio
     * @param highRatio The higher bound ratio
     * @param invert Invert the range
     * @return The desired threshold
     */
    public static Threshold fromRatios(double lowRatio, double highRatio, boolean invert) {
        return new Threshold(
                lowRatio * (GLOBAL_MAX - GLOBAL_MIN) + GLOBAL_MIN,
                highRatio * (GLOBAL_MAX - GLOBAL_MIN) + GLOBAL_MIN,
                invert
        );
    }

    /**
     * Threshold from ratios of the total range
     *
     * @param lowRatio The lower bound ratio
     * @param highRatio The higher bound ratio
     * @return The desired threshold
     */
    public static Threshold fromRatios(double lowRatio, double highRatio) {
        return fromRatios(lowRatio, highRatio, false);
    }

    /**
     * Gets the ratio of the lower bound vs. the total range
     *
     * @return The desired ratio
     */
    public double lowRatio() {
        return (low - GLOBAL_MIN) / (GLOBAL_MAX - GLOBAL_MIN);
    }

    /**
     * Gets the ratio of the upper bound vs. the total range
     *
     * @return The desired ratio
     */
    public double highRatio() {
        return (high - GLOBAL_MIN) / (GLOBAL_MAX - GLOBAL_MIN);
    }

    /**
     * Generates a random value within the threshold
     * @return The random value
     */
    public double randomValue() {
        if (!inverted) return low + Math.random() * (high - low);

        double lowerLength = low - GLOBAL_MIN;
        double higherLength = GLOBAL_MAX - high;
        double r = Math.random() * (lowerLength + higherLength);
        if (r < lowerLength) {
            return GLOBAL_MIN + r;
        } else {
            return high + (r - lowerLength);
        }
    }

    /**
     * Check if a threshold contains a value
     *
     * @param value The value to check
     * @return If the threshold contains the value
     */
    public boolean contains(double value) {
        if (!inverted) {
            return value >= low && value <= high;
        }

        return (value >= GLOBAL_MIN && value <= low) ||
                (value >= high && value <= GLOBAL_MAX);
    }

    /**
     * Get the range of the threshold
     *
     * @return The desired range
     */
    public double range() {
        if (inverted) {
            return (GLOBAL_MAX - GLOBAL_MIN) - (high - low);
        } else {
            return high - low;
        }
    }

    private static void maskNativeImpl(
            int depth, double[] lows, double[] highs,
            Threshold[] thresholds, Mat src, Mat dst
    ) {
        Mat mask = new Mat();
        Threshold threshold = thresholds[depth];
        if (!threshold.inverted) {
            lows[depth] = threshold.low;
            highs[depth] = threshold.high;
            if (depth >= thresholds.length - 1) {
                Core.inRange(src, new Scalar(lows), new Scalar(highs), mask);
                Core.max(dst, mask, dst);
            } else {
                maskNativeImpl(depth + 1, lows, highs, thresholds, src, dst);
            }
            return;
        }

        lows[depth] = GLOBAL_MIN;
        highs[depth] = threshold.low;
        if (depth >= thresholds.length - 1) {
            Core.inRange(src, new Scalar(lows), new Scalar(highs), mask);
            Core.max(dst, mask, dst);
        } else {
            maskNativeImpl(depth + 1, lows, highs, thresholds, src, dst);
        }

        lows[depth] = threshold.high;
        highs[depth] = GLOBAL_MAX;
        if (depth >= thresholds.length - 1) {
            Core.inRange(src, new Scalar(lows), new Scalar(highs), mask);
            Core.max(dst, mask, dst);
        } else {
            maskNativeImpl(depth + 1, lows, highs, thresholds, src, dst);
        }
    }

    /**
     * Recursively masks a {@link Mat} with {@link Threshold}
     *
     * @param thresholds The color thresholds
     * @param src The original mat
     * @param dst The output mat
     */
    public static void maskNative(Threshold[] thresholds, Mat src, Mat dst) {
        double[] firstPixel = src.get(0, 0);
        assert thresholds.length == firstPixel.length:
                "Thresholds length must be equal to src color space length of " +
                        firstPixel.length + " got: " + thresholds.length;

        maskNativeImpl(
                0, new double[thresholds.length],
                new double[thresholds.length], thresholds, src, dst
        );
    }

    /**
     * Check if a color is within thresholds
     *
     * @param thresholds The thresholds to check
     * @param color The color array
     * @return If the color is within the thresholds
     */
    public static boolean colorWithin(Threshold[] thresholds, double[] color) {
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
}