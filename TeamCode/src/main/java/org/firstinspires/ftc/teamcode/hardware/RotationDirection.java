package org.firstinspires.ftc.teamcode.hardware;

public enum RotationDirection {
    //enum for direction
    FORWARD(1), REVERSE(-1);

    final private int val;

    RotationDirection(int multiplier) {
        val = multiplier;
    }

    public int getMultiplier() {
        return val;
    }
}
