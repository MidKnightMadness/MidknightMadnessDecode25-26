package org.firstinspires.ftc.teamcode.util;

public class Threshold {
    public static final int GLOBAL_MIN = 0;
    public static final int GLOBAL_MAX = 255;

    public final boolean invert;
    public final double low;
    public final double high;

    public Threshold(double low, double high, boolean invert) {
        assert low >= GLOBAL_MIN && low <= GLOBAL_MAX:
                "Min threshold must be between " + GLOBAL_MIN + " and " + GLOBAL_MAX;
        assert high >= GLOBAL_MIN && high <= GLOBAL_MAX:
                "Max threshold must be between " + GLOBAL_MIN + " and " + GLOBAL_MAX;
        assert low <= high:
                "Min threshold must be less than or equal to high threshold";

        this.low = low;
        this.high = high;
        this.invert = invert;
    }

    public Threshold(double low, double high) {
        this(low, high, false);
    }

    public static Threshold any() {
        return new Threshold(GLOBAL_MIN, GLOBAL_MAX);
    }

    public static Threshold fromRatios(double lowRatio, double highRatio, boolean invert) {
        return new Threshold(
                lowRatio * (GLOBAL_MAX - GLOBAL_MIN) + GLOBAL_MIN,
                highRatio * (GLOBAL_MAX - GLOBAL_MIN) + GLOBAL_MIN,
                invert
        );
    }

    public static Threshold fromRatios(double lowRatio, double highRatio) {
        return fromRatios(lowRatio, highRatio, false);
    }

    public double lowRatio() {
        return (low - GLOBAL_MIN) / (GLOBAL_MAX - GLOBAL_MIN);
    }

    public double highRatio() {
        return (high - GLOBAL_MIN) / (GLOBAL_MAX - GLOBAL_MIN);
    }

    public double randomValue() {
        if (!invert) return low + Math.random() * (high - low);

        double lowerLength = low - GLOBAL_MIN;
        double higherLength = GLOBAL_MAX - high;
        double r = Math.random() * (lowerLength + higherLength);
        if (r < lowerLength) {
            return GLOBAL_MIN + r;
        } else {
            return high + (r - lowerLength);
        }
    }

    public boolean contains(double value) {
        if (!invert) {
            return value >= low && value <= high;
        }

        return (value >= GLOBAL_MIN && value <= low) ||
                (value >= high && value <= GLOBAL_MAX);
    }

    public double range() {
        return high - low;
    }
}