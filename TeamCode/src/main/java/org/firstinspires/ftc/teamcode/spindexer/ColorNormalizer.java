package org.firstinspires.ftc.teamcode.spindexer;

public class ColorNormalizer {
    double red;
    double green;
    double blue;
    public ColorNormalizer(double r, double g, double b){
        red = r;
        green = g;
        blue = b;
    }
    public ColorNormalizer(){
        red = 0;
        green = 0;
        blue = 0;
    }

    public double magnitude(){
        return Math.sqrt((red*red) + (green*green) + (blue*blue));
    }
    public double normalizeRed() {
        return red / magnitude();
    }

    public double normalizeGreen() {
        return green / magnitude();
    }

    public double normalizeBlue() {
        return blue / magnitude();
    }
}
