package org.firstinspires.ftc.teamcode.colors;

import java.util.ArrayList;

public class ColorSensorBuffer {
    public ArrayList<Double> list;
    public double avg = 0;
    public double sum = 0;

    final double MAX_SIZE = 10;
    public ColorSensorBuffer(){
        list = new ArrayList<>();
    }

    public void addList(double addNum){
        list.add(addNum);
        if (list.size() >= MAX_SIZE){
            for (int i = 0; i < list.size(); i++) {
                sum += list.get(i);
            }
            avg = sum/3;
            sum = 0;
            list.clear();
        }
    }

    public double getAverage(){
        return avg;
    }
}