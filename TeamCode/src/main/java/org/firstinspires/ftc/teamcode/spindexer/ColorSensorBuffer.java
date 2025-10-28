package org.firstinspires.ftc.teamcode.spindexer;

import java.util.ArrayList;

public class ColorSensorBuffer {
    ArrayList<Double> list;
    double num = 0;

    double sum = 0;
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
