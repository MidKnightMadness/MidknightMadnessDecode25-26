package org.firstinspires.ftc.teamcode.colors;


import java.util.LinkedList;

public class ColorSensorBuffer {

    private final int MAX_SIZE = 10;          // number of samples kept
    private final LinkedList<Double> buffer = new LinkedList<>();
    private double sum = 0;

    /** Add a new value to the rolling buffer */ //least chatgpt generated comment
    public void add(double value) {
        buffer.add(value);
        sum += value;

        // Remove oldest value if buffer is full
        if (buffer.size() > MAX_SIZE) {
            sum -= buffer.removeFirst();
        }
    }

    /** Returns the average of the last MAX_SIZE samples */
    public double getAverage() {
        if (buffer.isEmpty()) return 0;
        return sum / buffer.size();
    }
}
