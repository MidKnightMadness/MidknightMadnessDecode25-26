package org.firstinspires.ftc.teamcode.newpid;

import java.util.ArrayDeque;

public class Buffer {
    public int sample_count;
    public double deriv_tolerance;
    public ArrayDeque<Double> times;
    public ArrayDeque<Double> samples;

    public Buffer(int sample_count, double deriv_tolerance) {
        this.sample_count = sample_count;
        this.deriv_tolerance = deriv_tolerance;
        times = new ArrayDeque<>();
        samples = new ArrayDeque<>();
    }

    public void clear() {
        samples.clear();
        times.clear();
    }

    public void add(double sample, double time) {
        times.addLast(time);
        samples.addLast(sample);
        if (samples.size() > sample_count) {
            times.removeFirst();
            samples.removeFirst();
        }
    }

    public double getDerivative() {
        if (samples.size() < 2) return 0;
        double sampleGap = samples.getLast() - samples.getFirst();
        double timeGap = times.getLast() - times.getFirst();
        return sampleGap / timeGap;
    }

    public boolean isStable() {
        if (samples.size() < sample_count) {
            return false;
        } else {
            return getDerivative() < deriv_tolerance;
        }
    }
}
