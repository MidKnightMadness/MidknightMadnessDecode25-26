package org.firstinspires.ftc.teamcode.util;

public class Range {
    public double low, high;

    public Range(double low, double high) {
        assert(high > low);
        this.low = low;
        this.high = high;
    }

    public double getValueRatio(double value) {
        return (value - low) / (high - low);
    }

    public double getValueFromRatio(double ratio) {
        return low + (high - low) * ratio;
    }

    public double convert(double value, Range newRange) {
        double ratio = this.getValueRatio(value);
        return newRange.getValueFromRatio(ratio);
    }

    public boolean contains(double value) {
        return value >= low && value <= high;
    }

    public double clip(double value) {
        return Math.max(Math.min(value, high), low);
    }

    public double range() {
        return high - low;
    }
}
