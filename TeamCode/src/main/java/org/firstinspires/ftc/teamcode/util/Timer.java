package org.firstinspires.ftc.teamcode.util;

import java.util.concurrent.TimeUnit;

public class Timer {
    private long startTime;
    private long previousTime;
    private final TimeUnit defaultUnit;

    public Timer() {
        this(TimeUnit.MILLISECONDS);
    }

    public Timer(TimeUnit defaultUnit) {
        this.defaultUnit = defaultUnit;
        restart();
    }

    public void setTime(double time) {
        setTime(time, defaultUnit);
    }

    public void setTime(double time, TimeUnit unit) {
        startTime = System.nanoTime() - (long) toNanos(time, unit);
        previousTime = startTime;
    }

    public void restart() {
        startTime = System.nanoTime();
        previousTime = startTime;
    }

    public double getTime() {
        return getTime(defaultUnit);
    }

    public double getTime(TimeUnit unit) {
        long elapsed = System.nanoTime() - startTime;
        return fromNanos(elapsed, unit);
    }

    public double getDeltaTime() {
        return getDeltaTime(defaultUnit);
    }

    public double getDeltaTime(TimeUnit unit) {
        long now = System.nanoTime();
        long delta = now - previousTime;
        previousTime = now;
        return fromNanos(delta, unit);
    }

    public double getPreviousTime() {
        return getPreviousTime(defaultUnit);
    }

    public double getPreviousTime(TimeUnit unit) {
        long elapsed = previousTime - startTime;
        return fromNanos(elapsed, unit);
    }

    private static double fromNanos(double nanos, TimeUnit unit) {
        switch (unit) {
            case NANOSECONDS: return nanos;
            case MICROSECONDS: return nanos / 1_000.0;
            case MILLISECONDS: return nanos / 1_000_000.0;
            case SECONDS: return nanos / 1_000_000_000.0;
            case MINUTES: return nanos / 60_000_000_000.0;
            case HOURS: return nanos / 3_600_000_000_000.0;
            case DAYS: return nanos / 86_400_000_000_000.0;
            default: throw new IllegalArgumentException("Unsupported TimeUnit");
        }
    }

    private static double toNanos(double time, TimeUnit unit) {
        switch (unit) {
            case NANOSECONDS: return time;
            case MICROSECONDS: return time * 1_000.0;
            case MILLISECONDS: return time * 1_000_000.0;
            case SECONDS: return time * 1_000_000_000.0;
            case MINUTES: return time * 60_000_000_000.0;
            case HOURS: return time * 3_600_000_000_000.0;
            case DAYS: return time * 86_400_000_000_000.0;
            default: throw new IllegalArgumentException("Unsupported TimeUnit");
        }
    }
}
