package org.firstinspires.ftc.teamcode.Util;

public class Timer {
    private long startTime;
    private long previousTime;

    public Timer() {
        resetTimer();
    }

    public void resetTimer() {
        startTime = System.currentTimeMillis();
        previousTime = startTime;
    }

    public long getElapsedTime() {
        previousTime = System.currentTimeMillis();
        return previousTime - startTime;
    }

    public long getLastUpdateTime() {
        return System.currentTimeMillis() - previousTime;
    }
}