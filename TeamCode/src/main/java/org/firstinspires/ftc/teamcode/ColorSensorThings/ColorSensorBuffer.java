package org.firstinspires.ftc.teamcode.ColorSensorThings;

import java.util.ArrayList;

public class ColorSensorBuffer {
    public ArrayList<Double> list;
    public double num = 0;
    public double sum = 0;
    public ColorSensorBuffer(){
        list = new ArrayList<>();
    }

    public void addList(double addNum){
        list.add(addNum);
        if (list.size() >= 3){
            for (int i = 0; i < list.size(); i++) {
                sum = sum + list.get(i);
            }
            num = sum/3;
            sum = 0;
            list.clear();
        }

    }
}