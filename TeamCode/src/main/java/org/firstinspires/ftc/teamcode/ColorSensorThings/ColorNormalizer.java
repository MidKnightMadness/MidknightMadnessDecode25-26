package org.firstinspires.ftc.teamcode.ColorSensorThings;

public class ColorNormalizer {
    public double red;
    public double green;
    public double blue;
    //constructer
    public ColorNormalizer(double red, double g, double b){
        this.red = red;
        green = g;
        blue = b;
    }
    public ColorNormalizer(){
        red = 0;
        green = 0;
        blue = 0;
    }

    //getting magnitude
    public double magnitude(){
        return Math.sqrt((red*red) + (green*green) + (blue*blue));
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
}
