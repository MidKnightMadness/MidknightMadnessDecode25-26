package org.firstinspires.ftc.teamcode.colors;

public class ColorNormalizer {
    public double red;
    public double green;
    public double blue;

    //constructor
    public ColorNormalizer(double red, double g, double b) {
        this.red = red;
        green = g;
        blue = b;
    }

    public ColorNormalizer() {
        red = 0;
        green = 0;
        blue = 0;
    }

    public static double[] getNormalizedRGB(double r, double g, double b){
        double[] normalizedValues = new double[]{normalizeRed(r,g,b), normalizeGreen(r, g, b), normalizeBlue(r, g, b)};
        return normalizedValues;
    }

    //getting magnitude
    public double magnitude() {
        return Math.sqrt((red * red) + (green * green) + (blue * blue));
    }

    //getting normalized values
    public double normalizeRed() {
        return red / magnitude();
    }

    public double normalizeGreen() {
        return green / magnitude();
    }

    public double normalizeBlue() {
        return blue / magnitude();
    }



    //making static ones because idk
    public static double magnitude(double red, double green, double blue){
        return Math.sqrt((red*red) + (green*green) + (blue*blue));
    }

    public static double normalizeRed(double red, double green, double blue) {
        return red / ColorNormalizer.magnitude(red, green, blue);
    }

    public static double normalizeGreen(double red, double green, double blue) {
        return green / magnitude(red, green, blue);
    }

    public static double normalizeBlue(double red, double green, double blue) {
        return blue / magnitude(red, green, blue);
    }
}