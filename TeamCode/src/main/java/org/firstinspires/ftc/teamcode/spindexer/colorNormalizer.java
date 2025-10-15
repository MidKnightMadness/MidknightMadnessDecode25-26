package org.firstinspires.ftc.teamcode.spindexer;

public class colorNormalizer {
    double red;
    double green;
    double blue;
    double magnitude;
    public colorNormalizer(double r, double g, double b){
        red = r;
        green = g;
        blue = b;
        magnitude =  Math.sqrt((r*r) + (g*g) + (b*b));
    }

    public double normalizeRed() {
        return red / magnitude;
    }

    public double normalizeGreen() {
        return green / magnitude;
    }

    public double normalizeBlue() {
        return blue / magnitude;
    }
}
