package org.firstinspires.ftc.teamcode.Util;

import java.util.concurrent.TimeUnit;

public class Timer {
    private long startTime;
    private long previousTime;

    public Timer() {
        restartTimer();
    }

    public void restartTimer() {
        startTime = System.nanoTime();
        previousTime = startTime;
    }

    public long getTime(TimeUnit unit) {
        long elapsed = System.nanoTime() - startTime;
        return unit.convert(elapsed, TimeUnit.NANOSECONDS);
    }

    public long getDeltaTime(TimeUnit unit) {
        long now = System.nanoTime();
        long delta = now - previousTime;
        previousTime = now;
        return unit.convert(delta, TimeUnit.NANOSECONDS);
    }

    public long getPreviousTime(TimeUnit unit) {
        return unit.convert(previousTime - startTime, TimeUnit.NANOSECONDS);
    }
}
