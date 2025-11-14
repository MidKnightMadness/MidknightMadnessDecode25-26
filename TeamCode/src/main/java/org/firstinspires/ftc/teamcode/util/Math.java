package org.firstinspires.ftc.teamcode.util;

public class Math {

    public static double clampOutput(double val, double minVal, double maxVal){
        if(val <= minVal) return minVal;
        if(val >= maxVal) return maxVal;
        return val;
    }
}
