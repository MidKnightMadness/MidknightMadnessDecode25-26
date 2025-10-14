package org.firstinspires.ftc.teamcode.Util;

public class Point {
    private double x;
    private double y;
    private double theta;

    public Point(){
        this.x = 0;
        this.y = 0;
        this.theta = 0;
    }

    public Point(double x, double y){
        this.x = x;
        this.y = y;
        this.theta = 0;
    }
    public Point(double x, double y, double theta){
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public Point(Point point){
        this.x = point.x;
        this.y = point.y;
        this.theta = point.theta;
    }

    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public double getThetaRadians(){
        return theta;
    }

    public double getThetaDegrees(){
        return theta * 180 / Math.PI;
    }

    public String toString(){
        return String.format("(%2f, %2f, %2f rad)", x, y, theta);
    }

}
